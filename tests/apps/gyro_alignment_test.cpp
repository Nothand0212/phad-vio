#include "apps/gyro_alignment.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "apps/gyro_alignment_runner.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"

namespace
{

  using phad::apps::analyzeGyroAlignment;
  using phad::apps::GyroAlignmentAnalysis;
  using phad::apps::GyroAlignmentBlock;
  using phad::apps::GyroAlignmentEndpoint;
  using phad::apps::GyroAlignmentEndpointStatus;
  using phad::apps::GyroAlignmentErrorCode;
  using phad::apps::GyroAlignmentFit;
  using phad::apps::GyroAlignmentFitStatus;
  using phad::apps::GyroAlignmentInput;
  using phad::apps::GyroAlignmentInputIdentity;
  using phad::apps::GyroAlignmentPacket;
  using phad::apps::GyroAlignmentPacketStatus;
  using phad::apps::GyroAlignmentProtocolDescriptor;
  using phad::apps::GyroAlignmentReason;
  using phad::apps::GyroAlignmentResult;
  using phad::apps::GyroAlignmentStatus;
  using phad::apps::runGyroAlignment;

  constexpr std::int64_t kSecondNs = 1'000'000'000;
  constexpr double       kEpsPi    = 0x1.0p-26;
  static_assert( kEpsPi * kEpsPi ==
                 std::numeric_limits<double>::epsilon() );

  struct ScopedDirectory
  {
    explicit ScopedDirectory( std::string name )
        : path{ std::filesystem::temp_directory_path() /
                ( "phad_q3_" + std::move( name ) ) }
    {
      std::filesystem::remove_all( path );
      std::filesystem::create_directories( path );
    }

    ~ScopedDirectory() { std::filesystem::remove_all( path ); }

    ScopedDirectory( const ScopedDirectory& )            = delete;
    ScopedDirectory& operator=( const ScopedDirectory& ) = delete;

    std::filesystem::path path;
  };

  [[nodiscard]] phad::sensor::ImuMeasurement measurement(
      std::int64_t timestamp_ns, const Eigen::Vector3d& gyro_radps )
  {
    return phad::sensor::ImuMeasurement{
        .timestamp  = phad::common::Timestamp{ timestamp_ns },
        .accel_mps2 = { 0.0, 0.0, 9.81 },
        .gyro_radps = { gyro_radps.x(), gyro_radps.y(), gyro_radps.z() },
    };
  }

  [[nodiscard]] Eigen::Quaterniond rotationVectorQuaternion(
      const Eigen::Vector3d& rotation_vector )
  {
    const double theta = rotation_vector.norm();
    if ( theta == 0.0 )
    {
      return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond{
        Eigen::AngleAxisd{ theta, rotation_vector / theta } };
  }

  [[nodiscard]] GyroAlignmentEndpoint endpoint(
      std::int64_t                timestamp_ns,
      Eigen::Quaterniond          q_wb = Eigen::Quaterniond::Identity(),
      GyroAlignmentEndpointStatus status =
          GyroAlignmentEndpointStatus::kOk,
      std::uint32_t segment_id = 0U )
  {
    return GyroAlignmentEndpoint{
        .timestamp_ns = timestamp_ns,
        .status       = status,
        .segment_id   = segment_id,
        .q_WB         = status == GyroAlignmentEndpointStatus::kRejected
                            ? std::nullopt
                            : std::optional<Eigen::Quaterniond>{ std::move( q_wb ) },
    };
  }

  [[nodiscard]] GyroAlignmentPacket firstZero( std::int64_t t0_ns )
  {
    return GyroAlignmentPacket{
        .packet_index = 0U,
        .t_prev_ns    = t0_ns,
        .t_cur_ns     = t0_ns,
        .segment_id   = 0U,
        .imu_gap      = false,
        .status       = GyroAlignmentPacketStatus::kFirstZero,
        .samples      = {},
        .sum_dt_ns    = 0,
        .interval_ns  = 0,
    };
  }

  [[nodiscard]] GyroAlignmentPacket validPacket(
      std::uint64_t packet_index, std::int64_t t_prev_ns,
      std::int64_t t_cur_ns, const Eigen::Vector3d& gyro_radps,
      std::uint32_t segment_id = 0U )
  {
    return GyroAlignmentPacket{
        .packet_index = packet_index,
        .t_prev_ns    = t_prev_ns,
        .t_cur_ns     = t_cur_ns,
        .segment_id   = segment_id,
        .imu_gap      = false,
        .status       = GyroAlignmentPacketStatus::kValid,
        .samples      = { measurement( t_prev_ns, gyro_radps ),
                          measurement( t_cur_ns, gyro_radps ) },
        .sum_dt_ns    = t_cur_ns - t_prev_ns,
        .interval_ns  = t_cur_ns - t_prev_ns,
    };
  }

  [[nodiscard]] GyroAlignmentPacket gapPacket( std::uint64_t packet_index,
                                               std::int64_t  t_prev_ns,
                                               std::int64_t  t_cur_ns )
  {
    return GyroAlignmentPacket{
        .packet_index = packet_index,
        .t_prev_ns    = t_prev_ns,
        .t_cur_ns     = t_cur_ns,
        .segment_id   = 0U,
        .imu_gap      = true,
        .status       = GyroAlignmentPacketStatus::kGap,
        .samples      = {},
        .sum_dt_ns    = 0,
        .interval_ns  = t_cur_ns - t_prev_ns,
    };
  }

  [[nodiscard]] GyroAlignmentInput stationaryBiasFixture(
      const Eigen::Vector3d& fit_bias, const Eigen::Vector3d& validation_bias,
      std::int64_t fit_dt_ns = kSecondNs, std::size_t fit_packets = 100U,
      std::size_t validation_packets = 60U )
  {
    GyroAlignmentInput input;
    input.endpoints.push_back( endpoint( 0 ) );
    input.packets.push_back( firstZero( 0 ) );

    std::int64_t timestamp_ns = 0;
    for ( std::size_t i = 0; i < fit_packets; ++i )
    {
      const auto next = timestamp_ns + fit_dt_ns;
      input.packets.push_back(
          validPacket( input.packets.size(), timestamp_ns, next, fit_bias ) );
      input.endpoints.push_back( endpoint( next ) );
      timestamp_ns = next;
    }

    if ( timestamp_ns < 100 * kSecondNs )
    {
      input.packets.push_back( gapPacket( input.packets.size(), timestamp_ns,
                                          100 * kSecondNs ) );
      input.endpoints.push_back( endpoint( 100 * kSecondNs ) );
      timestamp_ns = 100 * kSecondNs;
    }

    for ( std::size_t i = 0; i < validation_packets; ++i )
    {
      const auto next = timestamp_ns + kSecondNs;
      input.packets.push_back( validPacket( input.packets.size(), timestamp_ns,
                                            next, validation_bias ) );
      input.endpoints.push_back( endpoint( next ) );
      timestamp_ns = next;
    }
    return input;
  }

  [[nodiscard]] GyroAlignmentInput mixedValidationFixture(
      const Eigen::Vector3d& fit_bias, std::size_t improved_blocks )
  {
    GyroAlignmentInput input =
        stationaryBiasFixture( fit_bias, fit_bias, kSecondNs, 100U, 60U );
    for ( std::size_t i = improved_blocks; i < 60U; ++i )
    {
      auto& packet   = input.packets.at( 101U + i );
      packet.samples = { measurement( packet.t_prev_ns,
                                      Eigen::Vector3d::Zero() ),
                         measurement( packet.t_cur_ns,
                                      Eigen::Vector3d::Zero() ) };
    }
    return input;
  }

  [[nodiscard]] GyroAlignmentInput axisValidationFixture(
      std::size_t x_worsened_blocks )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    GyroAlignmentInput    input = stationaryBiasFixture( bias, bias );
    for ( std::size_t i = 0; i < x_worsened_blocks; ++i )
    {
      auto&                 packet = input.packets.at( 160U - i );
      const Eigen::Vector3d gyro{ 0.0, bias.y(), bias.z() };
      packet.samples = { measurement( packet.t_prev_ns, gyro ),
                         measurement( packet.t_cur_ns, gyro ) };
    }
    return input;
  }

  [[nodiscard]] GyroAlignmentInput splitSupportFixture(
      std::int64_t fit_dt_ns, const Eigen::Vector3d& bias )
  {
    GyroAlignmentInput input;
    input.endpoints.push_back( endpoint( 0 ) );
    input.packets.push_back( firstZero( 0 ) );
    std::int64_t timestamp_ns = 0;
    for ( std::size_t half = 0; half < 2U; ++half )
    {
      for ( std::size_t i = 0; i < 50U; ++i )
      {
        const auto next = timestamp_ns + fit_dt_ns;
        input.packets.push_back( validPacket(
            input.packets.size(), timestamp_ns, next, bias ) );
        input.endpoints.push_back( endpoint( next ) );
        timestamp_ns = next;
      }
      const auto boundary =
          static_cast<std::int64_t>( half + 1U ) * 50 * kSecondNs;
      input.packets.push_back(
          gapPacket( input.packets.size(), timestamp_ns, boundary ) );
      input.endpoints.push_back( endpoint( boundary ) );
      timestamp_ns = boundary;
    }
    for ( std::size_t i = 0; i < 60U; ++i )
    {
      const auto next = timestamp_ns + kSecondNs;
      input.packets.push_back(
          validPacket( input.packets.size(), timestamp_ns, next, bias ) );
      input.endpoints.push_back( endpoint( next ) );
      timestamp_ns = next;
    }
    return input;
  }

  struct NonlinearRow
  {
    double          duration_s;
    Eigen::Vector3d gyro_radps;
    Eigen::Vector3d vis_rotation_vector;
  };

  [[nodiscard]] std::vector<NonlinearRow> frozenNonlinearRows()
  {
    const std::array<NonlinearRow, 3> kTypes{
        NonlinearRow{
            0x1.0p+1,
            Eigen::Vector3d{ 0x1.984f258aaf334p+0,
                             0x1.2d730db8f34dep+1,
                             -0x1.0b9619e91b800p-1 },
            Eigen::Vector3d{ 0x1.8faa398e0df06p-4,
                             -0x1.2cbcc1de2b48fp-3,
                             0x1.be079494ba76bp-4 } },
        NonlinearRow{
            0x1.0p-1,
            Eigen::Vector3d{ -0x1.37957f1813a8cp+2,
                             0x1.7ac69fb0e1dbcp+1,
                             0x1.fe961832a00b8p+0 },
            Eigen::Vector3d{ -0x1.0926c4fbefd61p-2,
                             0x1.a271cb88a9e7ap-1,
                             0x1.53acbdf44c5acp-2 } },
        NonlinearRow{
            0x1.0p+1,
            Eigen::Vector3d{ 0x1.cb88edb85fc74p+1,
                             -0x1.c159adf458a9ap+0,
                             -0x1.3b8aeb6d42d62p+1 },
            Eigen::Vector3d{ 0x1.bc362020823a6p+0,
                             -0x1.fdaa939c3efc7p-2,
                             0x1.f503683abb0f4p+0 } },
    };
    constexpr std::array<std::size_t, 3> kCounts{ 2U, 3U, 5U };
    constexpr std::size_t                kRepeats = 3U;

    std::vector<NonlinearRow> rows;
    rows.reserve( 30U );
    for ( std::size_t repeat = 0; repeat < kRepeats; ++repeat )
    {
      for ( std::size_t type = 0; type < kTypes.size(); ++type )
      {
        for ( std::size_t count = 0; count < kCounts[ type ]; ++count )
        {
          rows.push_back( kTypes[ type ] );
        }
      }
    }
    return rows;
  }

  [[nodiscard]] GyroAlignmentInput nonlinearRankSiblingFixture(
      bool nonlinear_early )
  {
    constexpr std::int64_t kRankDtNs = 20'000'000;
    const Eigen::Vector3d  kRankRate{
        2.0 * std::numbers::pi / 0.02, 0.0, 0.0 };
    const auto kNonlinearRows = frozenNonlinearRows();

    GyroAlignmentInput input;
    input.endpoints.push_back( endpoint( 0 ) );
    input.packets.push_back( firstZero( 0 ) );
    Eigen::Quaterniond q_wb         = Eigen::Quaterniond::Identity();
    std::int64_t       timestamp_ns = 0;
    for ( std::size_t half = 0; half < 2U; ++half )
    {
      const bool nonlinear_half = half == 0U ? nonlinear_early
                                             : !nonlinear_early;
      if ( nonlinear_half )
      {
        for ( const NonlinearRow& row : kNonlinearRows )
        {
          const auto duration_ns = static_cast<std::int64_t>(
              row.duration_s * static_cast<double>( kSecondNs ) );
          const auto next = timestamp_ns + duration_ns;
          q_wb            = q_wb *
                 rotationVectorQuaternion( row.vis_rotation_vector );
          input.packets.push_back(
              validPacket( input.packets.size(), timestamp_ns, next,
                           row.gyro_radps ) );
          input.endpoints.push_back( endpoint( next, q_wb ) );
          timestamp_ns = next;
        }
      }
      else
      {
        for ( std::size_t i = 0; i < 2'250U; ++i )
        {
          const auto next = timestamp_ns + kRankDtNs;
          input.packets.push_back( validPacket(
              input.packets.size(), timestamp_ns, next, kRankRate ) );
          input.endpoints.push_back( endpoint( next, q_wb ) );
          timestamp_ns = next;
        }
      }

      if ( half == 0U )
      {
        constexpr std::int64_t kCrossStartNs = 46'500'000'000;
        constexpr std::int64_t kCrossEndNs   = 50'500'000'000;
        if ( timestamp_ns < kCrossStartNs )
        {
          input.packets.push_back( gapPacket(
              input.packets.size(), timestamp_ns, kCrossStartNs ) );
          input.endpoints.push_back( endpoint( kCrossStartNs, q_wb ) );
          timestamp_ns = kCrossStartNs;
        }
        input.packets.push_back(
            validPacket( input.packets.size(), timestamp_ns, kCrossEndNs,
                         Eigen::Vector3d::Zero() ) );
        input.endpoints.push_back( endpoint( kCrossEndNs, q_wb ) );
        timestamp_ns = kCrossEndNs;
      }
      else
      {
        constexpr std::int64_t kFitEndNs = 100'000'000'000;
        input.packets.push_back(
            gapPacket( input.packets.size(), timestamp_ns, kFitEndNs ) );
        input.endpoints.push_back( endpoint( kFitEndNs, q_wb ) );
        timestamp_ns = kFitEndNs;
      }
    }
    return input;
  }

  [[nodiscard]] GyroAlignmentInput halfBiasFixture( double late_delta_x )
  {
    const Eigen::Vector3d early_bias{ 0.002, -0.003, 0.001 };
    GyroAlignmentInput    input = stationaryBiasFixture(
        early_bias, early_bias, kSecondNs, 100U, 60U );
    const Eigen::Vector3d late_bias =
        early_bias + Eigen::Vector3d{ late_delta_x, 0.0, 0.0 };
    for ( std::size_t i = 51U; i <= 100U; ++i )
    {
      auto& packet   = input.packets.at( i );
      packet.samples = { measurement( packet.t_prev_ns, late_bias ),
                         measurement( packet.t_cur_ns, late_bias ) };
    }
    const Eigen::Vector3d full_bias =
        early_bias + Eigen::Vector3d{ 0.5 * late_delta_x, 0.0, 0.0 };
    for ( std::size_t i = 101U; i < input.packets.size(); ++i )
    {
      auto& packet   = input.packets.at( i );
      packet.samples = { measurement( packet.t_prev_ns, full_bias ),
                         measurement( packet.t_cur_ns, full_bias ) };
    }
    return input;
  }

  [[nodiscard]] GyroAlignmentInput nearPiPacketFixture(
      double duration_s, const Eigen::Vector3d& axis, double signed_theta )
  {
    const auto duration_ns =
        static_cast<std::int64_t>( duration_s * 1.0e9 );
    GyroAlignmentInput input;
    input.endpoints = {
        endpoint( 0 ),
        endpoint( duration_ns,
                  rotationVectorQuaternion( signed_theta * axis ) ),
    };
    input.packets = {
        firstZero( 0 ),
        validPacket( 1U, 0, duration_ns, Eigen::Vector3d::Zero() ),
    };
    return input;
  }

  [[nodiscard]] GyroAlignmentInput candidateObservabilityFixture()
  {
    GyroAlignmentInput input;
    input.endpoints.push_back( endpoint( 0 ) );
    input.packets.push_back( firstZero( 0 ) );
    Eigen::Quaterniond q_wb         = Eigen::Quaterniond::Identity();
    std::int64_t       timestamp_ns = 0;
    for ( std::size_t i = 0; i < 100U; ++i )
    {
      const std::size_t half_index = i % 50U;
      double            residual_x = 0.0;
      if ( half_index == 0U )
      {
        residual_x = std::numbers::pi - 2.0e-5;
      }
      else if ( half_index == 1U )
      {
        residual_x = -( std::numbers::pi - 2.0e-5 );
      }
      else if ( half_index == 2U )
      {
        residual_x = -1.0e-3;
      }
      q_wb = q_wb * rotationVectorQuaternion(
                        Eigen::Vector3d{ residual_x, 0.0, 0.0 } );
      const auto next = timestamp_ns + kSecondNs;
      input.packets.push_back( validPacket(
          input.packets.size(), timestamp_ns, next, Eigen::Vector3d::Zero() ) );
      input.endpoints.push_back( endpoint( next, q_wb ) );
      timestamp_ns = next;
    }
    for ( std::size_t i = 0; i < 60U; ++i )
    {
      const auto next = timestamp_ns + kSecondNs;
      input.packets.push_back( validPacket(
          input.packets.size(), timestamp_ns, next, Eigen::Vector3d::Zero() ) );
      input.endpoints.push_back( endpoint( next, q_wb ) );
      timestamp_ns = next;
    }
    return input;
  }

  [[nodiscard]] GyroAlignmentInput noncommutingFixture()
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    GyroAlignmentInput    input;
    input.endpoints.push_back( endpoint( 0 ) );
    input.packets.push_back( firstZero( 0 ) );
    Eigen::Quaterniond     q_wb         = Eigen::Quaterniond::Identity();
    std::int64_t           timestamp_ns = 0;
    constexpr std::int64_t kDtNs        = 500'000'000;
    for ( std::size_t i = 0; i < 320U; ++i )
    {
      const Eigen::Vector3d true_rate =
          i % 2U == 0U ? Eigen::Vector3d{ 0.02, 0.0, 0.0 }
                       : Eigen::Vector3d{ 0.0, -0.03, 0.01 };
      q_wb            = q_wb * rotationVectorQuaternion( true_rate * 0.5 );
      const auto next = timestamp_ns + kDtNs;
      input.packets.push_back( validPacket(
          input.packets.size(), timestamp_ns, next, true_rate + bias ) );
      input.endpoints.push_back( endpoint( next, q_wb ) );
      timestamp_ns = next;
    }
    return input;
  }

  [[nodiscard]] GyroAlignmentInput conditionFixture( double two_pi_gap )
  {
    GyroAlignmentInput input;
    input.endpoints.push_back( endpoint( 0 ) );
    input.packets.push_back( firstZero( 0 ) );
    constexpr std::int64_t kDtNs = 20'000'000;
    const double           rate_x =
        ( 2.0 * std::numbers::pi - two_pi_gap ) / 0.02;
    std::int64_t timestamp_ns = 0;
    for ( std::size_t half = 0; half < 2U; ++half )
    {
      for ( std::size_t i = 0; i < 2'250U; ++i )
      {
        const auto next = timestamp_ns + kDtNs;
        input.packets.push_back( validPacket(
            input.packets.size(), timestamp_ns, next,
            Eigen::Vector3d{ rate_x, 0.0, 0.0 } ) );
        input.endpoints.push_back( endpoint( next ) );
        timestamp_ns = next;
      }
      const auto boundary =
          static_cast<std::int64_t>( half + 1U ) * 50 * kSecondNs;
      input.packets.push_back(
          gapPacket( input.packets.size(), timestamp_ns, boundary ) );
      input.endpoints.push_back( endpoint( boundary ) );
      timestamp_ns = boundary;
    }
    return input;
  }

  [[nodiscard]] Eigen::Vector3d principalLogOracle(
      const Eigen::Matrix3d& rotation )
  {
    const Eigen::Vector3d v{
        0.5 * ( rotation( 2, 1 ) - rotation( 1, 2 ) ),
        0.5 * ( rotation( 0, 2 ) - rotation( 2, 0 ) ),
        0.5 * ( rotation( 1, 0 ) - rotation( 0, 1 ) ),
    };
    const double s = v.norm();
    const double c =
        std::clamp( 0.5 * ( rotation.trace() - 1.0 ), -1.0, 1.0 );
    const double theta = std::atan2( s, c );
    if ( theta <= kEpsPi )
    {
      return v;
    }
    return ( theta / s ) * v;
  }

  [[nodiscard]] Eigen::Matrix3d expOracle(
      const Eigen::Vector3d& rotation_vector )
  {
    const double theta = rotation_vector.norm();
    if ( theta == 0.0 )
    {
      return Eigen::Matrix3d::Identity();
    }
    const Eigen::Vector3d axis = rotation_vector / theta;
    const Eigen::Matrix3d axis_hat =
        ( Eigen::Matrix3d{} << 0.0, -axis.z(), axis.y(), axis.z(), 0.0,
          -axis.x(), -axis.y(), axis.x(), 0.0 )
            .finished();
    return Eigen::Matrix3d::Identity() + std::sin( theta ) * axis_hat +
           ( 1.0 - std::cos( theta ) ) * axis_hat * axis_hat;
  }

  [[nodiscard]] Eigen::Vector3d nonlinearResidualOracle(
      const NonlinearRow& row, const Eigen::Matrix3d& r_vis,
      const Eigen::Vector3d& bias )
  {
    const Eigen::Matrix3d r_imu =
        expOracle( ( row.gyro_radps - bias ) * row.duration_s );
    return principalLogOracle( r_imu.transpose() * r_vis );
  }

  [[nodiscard]] double orderedLossOracle(
      const std::vector<Eigen::Vector3d>& residuals )
  {
    double loss = 0.0;
    for ( const Eigen::Vector3d& residual : residuals )
    {
      const double packet_loss =
          ( residual.x() * residual.x() +
            residual.y() * residual.y() ) +
          residual.z() * residual.z();
      loss += packet_loss;
    }
    return loss;
  }

  struct NonlinearFitOracle
  {
    Eigen::Vector3d bias;
    Eigen::Vector3d singular_values;
    double          condition;
    double          loss_zero;
    double          loss_fit;
    double          candidate_gap;
  };

  [[nodiscard]] NonlinearFitOracle fitNonlinearRowsOracle(
      const std::vector<NonlinearRow>& rows )
  {
    constexpr double             kH = 1.0e-5;
    std::vector<Eigen::Matrix3d> vis_rotations;
    vis_rotations.reserve( rows.size() );
    Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
    for ( const NonlinearRow& row : rows )
    {
      const Eigen::Quaterniond q_i = q_wb.normalized();
      q_wb                         = q_wb * rotationVectorQuaternion( row.vis_rotation_vector );
      const Eigen::Quaterniond q_j = q_wb.normalized();
      vis_rotations.push_back( q_i.toRotationMatrix().transpose() *
                               q_j.toRotationMatrix() );
    }

    Eigen::VectorXd residual_zero(
        static_cast<Eigen::Index>( 3U * rows.size() ) );
    Eigen::MatrixXd jacobian(
        static_cast<Eigen::Index>( 3U * rows.size() ), 3 );
    std::vector<Eigen::Vector3d> zero_residuals;
    zero_residuals.reserve( rows.size() );
    for ( std::size_t row_index = 0; row_index < rows.size(); ++row_index )
    {
      const Eigen::Vector3d residual =
          nonlinearResidualOracle( rows[ row_index ],
                                   vis_rotations[ row_index ],
                                   Eigen::Vector3d::Zero() );
      zero_residuals.push_back( residual );
      residual_zero.segment<3>(
          static_cast<Eigen::Index>( 3U * row_index ) ) = residual;
      for ( Eigen::Index axis = 0; axis < 3; ++axis )
      {
        Eigen::Vector3d plus  = Eigen::Vector3d::Zero();
        Eigen::Vector3d minus = Eigen::Vector3d::Zero();
        plus( axis )          = kH;
        minus( axis )         = -kH;
        jacobian.block<3, 1>(
            static_cast<Eigen::Index>( 3U * row_index ), axis ) =
            ( nonlinearResidualOracle( rows[ row_index ],
                                       vis_rotations[ row_index ], plus ) -
              nonlinearResidualOracle( rows[ row_index ],
                                       vis_rotations[ row_index ], minus ) ) /
            ( 2.0 * kH );
      }
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd{
        jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV };
    const Eigen::Vector3d        singular_values = svd.singularValues();
    const Eigen::Vector3d        bias            = svd.solve( -residual_zero );
    std::vector<Eigen::Vector3d> fit_residuals;
    fit_residuals.reserve( rows.size() );
    double candidate_gap = std::numeric_limits<double>::infinity();
    for ( std::size_t row_index = 0; row_index < rows.size(); ++row_index )
    {
      const Eigen::Vector3d residual = nonlinearResidualOracle(
          rows[ row_index ], vis_rotations[ row_index ], bias );
      fit_residuals.push_back( residual );
      candidate_gap = std::min(
          candidate_gap, std::numbers::pi - residual.norm() );
    }
    return { bias,
             singular_values,
             singular_values.x() / singular_values.z(),
             orderedLossOracle( zero_residuals ),
             orderedLossOracle( fit_residuals ),
             candidate_gap };
  }

  enum class FdGuardOracle
  {
    kFrozen,
    kMissingMargin,
    kNanosecondsAsSeconds,
    kDoubleMargin,
  };

  [[nodiscard]] bool fdGuardExcludes( double gap_to_pi, double duration_s,
                                      FdGuardOracle oracle )
  {
    constexpr double kH     = 1.0e-5;
    double           margin = 0.0;
    switch ( oracle )
    {
      case FdGuardOracle::kFrozen:
        margin = std::nextafter( kH * duration_s,
                                 std::numeric_limits<double>::infinity() );
        break;
      case FdGuardOracle::kMissingMargin:
        margin = 0.0;
        break;
      case FdGuardOracle::kNanosecondsAsSeconds:
        margin = kH * duration_s * 1.0e9;
        break;
      case FdGuardOracle::kDoubleMargin:
        margin = 2.0 * kH * duration_s;
        break;
    }
    return gap_to_pi <= kEpsPi + margin;
  }

  [[nodiscard]] std::uint64_t positiveBits( double value )
  {
    EXPECT_GE( value, 0.0 );
    return std::bit_cast<std::uint64_t>( value );
  }

  [[nodiscard]] std::uint64_t ulpDistance( double lhs, double rhs )
  {
    const auto lhs_bits = positiveBits( lhs );
    const auto rhs_bits = positiveBits( rhs );
    return lhs_bits >= rhs_bits ? lhs_bits - rhs_bits
                                : rhs_bits - lhs_bits;
  }

  enum class FitTraceStatus
  {
    kSolvedNonincrease,
    kSolvedIncrease,
    kInconclusive,
    kHard,
  };

  struct VerdictTraceOracle
  {
    GyroAlignmentStatus      status;
    std::vector<std::string> evaluated;
    bool                     analysis_available;
  };

  [[nodiscard]] VerdictTraceOracle evaluateSiblingTraceOracle(
      const std::array<FitTraceStatus, 3>& sibling_status )
  {
    VerdictTraceOracle                        result{ GyroAlignmentStatus::kPass, {}, true };
    constexpr std::array<std::string_view, 3> kNames{ "full", "early",
                                                      "late" };
    bool                                      has_inconclusive       = false;
    bool                                      has_hypothesis_failure = false;
    for ( std::size_t i = 0; i < sibling_status.size(); ++i )
    {
      result.evaluated.emplace_back( kNames[ i ] );
      if ( sibling_status[ i ] == FitTraceStatus::kHard )
      {
        result.status             = GyroAlignmentStatus::kHardError;
        result.analysis_available = false;
        return result;
      }
      has_inconclusive = has_inconclusive ||
                         sibling_status[ i ] == FitTraceStatus::kInconclusive;
      has_hypothesis_failure =
          has_hypothesis_failure ||
          sibling_status[ i ] == FitTraceStatus::kSolvedIncrease;
    }
    if ( has_inconclusive )
    {
      result.status = GyroAlignmentStatus::kInconclusive;
    }
    else if ( has_hypothesis_failure )
    {
      result.status = GyroAlignmentStatus::kHypothesisFail;
    }
    return result;
  }

  struct SevenPointTrace
  {
    std::vector<Eigen::Vector3d> bias_points;
    Eigen::Matrix3d              jacobian;
  };

  [[nodiscard]] SevenPointTrace sevenPointTraceOracle()
  {
    constexpr double      kH = 1.0e-5;
    const Eigen::Matrix3d residual_jacobian =
        ( Eigen::Matrix3d{} << 1.0, 0.2, -0.1, -0.3, 0.9, 0.4, 0.25, -0.5,
          1.1 )
            .finished();
    SevenPointTrace trace{
        { Eigen::Vector3d::Zero(), Eigen::Vector3d{ kH, 0.0, 0.0 },
          Eigen::Vector3d{ -kH, 0.0, 0.0 },
          Eigen::Vector3d{ 0.0, kH, 0.0 },
          Eigen::Vector3d{ 0.0, -kH, 0.0 },
          Eigen::Vector3d{ 0.0, 0.0, kH },
          Eigen::Vector3d{ 0.0, 0.0, -kH } },
        Eigen::Matrix3d::Zero() };
    for ( Eigen::Index axis = 0; axis < 3; ++axis )
    {
      const auto& plus = trace.bias_points.at(
          1U + 2U * static_cast<std::size_t>( axis ) );
      const auto& minus = trace.bias_points.at(
          2U + 2U * static_cast<std::size_t>( axis ) );
      trace.jacobian.col( axis ) =
          ( residual_jacobian * plus - residual_jacobian * minus ) /
          ( 2.0 * kH );
    }
    EXPECT_TRUE( trace.jacobian.isApprox( residual_jacobian, 1.0e-15 ) );
    return trace;
  }

  struct PhysicalStencilOracle
  {
    bool            shape_valid;
    Eigen::Matrix3d jacobian;
    Eigen::Vector3d candidate_bias;
    double          candidate_loss;
  };

  [[nodiscard]] std::vector<Eigen::Vector3d> canonicalBiasPoints()
  {
    constexpr double kH = 1.0e-5;
    return { Eigen::Vector3d::Zero(), Eigen::Vector3d{ kH, 0.0, 0.0 },
             Eigen::Vector3d{ -kH, 0.0, 0.0 },
             Eigen::Vector3d{ 0.0, kH, 0.0 },
             Eigen::Vector3d{ 0.0, -kH, 0.0 },
             Eigen::Vector3d{ 0.0, 0.0, kH },
             Eigen::Vector3d{ 0.0, 0.0, -kH } };
  }

  [[nodiscard]] PhysicalStencilOracle physicalStencilOracle(
      const std::vector<Eigen::Vector3d>& bias_points )
  {
    constexpr double      kH  = 1.0e-5;
    constexpr double      kDt = 0.5;
    const Eigen::Vector3d known_bias{ 0.002, -0.003, 0.001 };
    const Eigen::Vector3d true_rate{ 0.02, 0.0, 0.0 };
    const Eigen::Vector3d measured_rate = true_rate + known_bias;
    const Eigen::Matrix3d r_vis =
        rotationVectorQuaternion( true_rate * kDt ).toRotationMatrix();
    const auto residual = [ &measured_rate, &r_vis, kDt ](
                              const Eigen::Vector3d& bias ) {
      const Eigen::Matrix3d r_imu =
          rotationVectorQuaternion( ( measured_rate - bias ) * kDt )
              .toRotationMatrix();
      return principalLogOracle( r_imu.transpose() * r_vis );
    };

    const auto canonical   = canonicalBiasPoints();
    bool       shape_valid = bias_points.size() == canonical.size();
    if ( shape_valid )
    {
      for ( std::size_t i = 0; i < canonical.size(); ++i )
      {
        shape_valid =
            shape_valid &&
            ( bias_points[ i ].array() == canonical[ i ].array() ).all();
      }
    }
    if ( bias_points.size() < canonical.size() )
    {
      return { false, Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(),
               std::numeric_limits<double>::infinity() };
    }

    Eigen::Matrix3d jacobian;
    for ( Eigen::Index axis = 0; axis < 3; ++axis )
    {
      const auto plus_index =
          1U + 2U * static_cast<std::size_t>( axis );
      const auto minus_index =
          2U + 2U * static_cast<std::size_t>( axis );
      jacobian.col( axis ) =
          ( residual( bias_points.at( plus_index ) ) -
            residual( bias_points.at( minus_index ) ) ) /
          ( 2.0 * kH );
    }
    const Eigen::Vector3d r0 = residual( bias_points.front() );
    const Eigen::Vector3d candidate =
        Eigen::JacobiSVD<Eigen::Matrix3d>{
            jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV }
            .solve( -r0 );
    const Eigen::Vector3d candidate_residual = residual( candidate );
    const double          loss =
        ( candidate_residual.x() * candidate_residual.x() +
          candidate_residual.y() * candidate_residual.y() ) +
        candidate_residual.z() * candidate_residual.z();
    return { shape_valid, jacobian, candidate, loss };
  }

  [[nodiscard]] double conditionMetricOracle( double two_pi_gap )
  {
    constexpr double      kH  = 1.0e-5;
    constexpr double      kDt = 0.02;
    const Eigen::Vector3d gyro{
        ( 2.0 * std::numbers::pi - two_pi_gap ) / kDt, 0.0, 0.0 };
    const auto residual = [ &gyro, kDt ]( const Eigen::Vector3d& bias ) {
      const Eigen::Matrix3d r_imu =
          rotationVectorQuaternion( ( gyro - bias ) * kDt )
              .toRotationMatrix();
      return principalLogOracle( r_imu.transpose() );
    };
    Eigen::Matrix3d jacobian;
    for ( Eigen::Index axis = 0; axis < 3; ++axis )
    {
      Eigen::Vector3d plus  = Eigen::Vector3d::Zero();
      Eigen::Vector3d minus = Eigen::Vector3d::Zero();
      plus( axis )          = kH;
      minus( axis )         = -kH;
      jacobian.col( axis ) =
          ( residual( plus ) - residual( minus ) ) / ( 2.0 * kH );
    }
    const Eigen::Vector3d singular_values =
        Eigen::JacobiSVD<Eigen::Matrix3d>{ jacobian }.singularValues();
    return singular_values.x() / singular_values.z();
  }

  [[nodiscard]] const GyroAlignmentAnalysis& requireAnalysis(
      const GyroAlignmentResult& result )
  {
    EXPECT_TRUE( result.analysis.has_value() );
    return *result.analysis;
  }

  void expectHardError( const GyroAlignmentResult& result,
                        GyroAlignmentErrorCode     code )
  {
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kHardError );
    EXPECT_EQ( result.verdict.exit_code, 1 );
    EXPECT_EQ( result.error, code );
    EXPECT_FALSE( result.analysis.has_value() );
    EXPECT_FALSE( result.verdict.detail.empty() );
  }

  void expectFitNullability( const GyroAlignmentFit& fit,
                             GyroAlignmentFitStatus  status )
  {
    EXPECT_EQ( fit.status, status );
    if ( status == GyroAlignmentFitStatus::kSolved )
    {
      EXPECT_TRUE( fit.bias_radps.has_value() );
      EXPECT_TRUE( fit.singular_values.has_value() );
      EXPECT_EQ( fit.rank, 3U );
      EXPECT_TRUE( fit.condition.has_value() );
      EXPECT_TRUE( fit.loss_zero_rad2.has_value() );
      EXPECT_TRUE( fit.loss_fit_rad2.has_value() );
      EXPECT_TRUE( fit.nonlinear_nonincrease.has_value() );
      return;
    }

    EXPECT_FALSE( fit.bias_radps.has_value() );
    EXPECT_FALSE( fit.loss_zero_rad2.has_value() );
    EXPECT_FALSE( fit.loss_fit_rad2.has_value() );
    EXPECT_FALSE( fit.nonlinear_nonincrease.has_value() );
    if ( status == GyroAlignmentFitStatus::kSupportInsufficient )
    {
      EXPECT_FALSE( fit.singular_values.has_value() );
      EXPECT_FALSE( fit.rank.has_value() );
      EXPECT_FALSE( fit.condition.has_value() );
    }
  }

  TEST( GyroAlignmentTest, PublicErrorTaxonomyHasExactlyTwentyFourCodes )
  {
    constexpr std::array kCodes{
        GyroAlignmentErrorCode::kInvalidProtocolDescriptor,
        GyroAlignmentErrorCode::kInputNotFound,
        GyroAlignmentErrorCode::kInputNotRegularFile,
        GyroAlignmentErrorCode::kInputHashMismatch,
        GyroAlignmentErrorCode::kInputIo,
        GyroAlignmentErrorCode::kInputGrammar,
        GyroAlignmentErrorCode::kInputSchema,
        GyroAlignmentErrorCode::kUnknownEnum,
        GyroAlignmentErrorCode::kDiagFailed,
        GyroAlignmentErrorCode::kIndexOrder,
        GyroAlignmentErrorCode::kCountMismatch,
        GyroAlignmentErrorCode::kDurationMismatch,
        GyroAlignmentErrorCode::kEndpointMismatch,
        GyroAlignmentErrorCode::kJoinMismatch,
        GyroAlignmentErrorCode::kTimestampOverflow,
        GyroAlignmentErrorCode::kNonfiniteInput,
        GyroAlignmentErrorCode::kInvalidQuaternion,
        GyroAlignmentErrorCode::kDiagPoseContradiction,
        GyroAlignmentErrorCode::kQ2IntegrationError,
        GyroAlignmentErrorCode::kNonfiniteComputation,
        GyroAlignmentErrorCode::kOutputExists,
        GyroAlignmentErrorCode::kOutputIo,
        GyroAlignmentErrorCode::kOutputPublish,
        GyroAlignmentErrorCode::kOutputRehashMismatch,
    };
    static_assert( kCodes.size() == 24U );
    EXPECT_EQ( kCodes.front(),
               GyroAlignmentErrorCode::kInvalidProtocolDescriptor );
    EXPECT_EQ( kCodes.back(),
               GyroAlignmentErrorCode::kOutputRehashMismatch );
  }

  TEST( GyroAlignmentTest, FirstZeroOnlyUsesSentinelRangesAndExactNullability )
  {
    GyroAlignmentInput input;
    input.endpoints = { endpoint( 17 ) };
    input.packets   = { firstZero( 17 ) };

    const auto  result   = analyzeGyroAlignment( input );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( result.verdict.exit_code, 3 );
    EXPECT_EQ( result.verdict.reason_codes,
               std::vector{ GyroAlignmentReason::kFitSupport } );
    EXPECT_EQ( analysis.ranges.t0_ns, 17 );
    EXPECT_EQ( analysis.ranges.t_mid_ns, 50'000'000'017 );
    EXPECT_EQ( analysis.ranges.t_split_ns, 100'000'000'017 );
    EXPECT_FALSE( analysis.ranges.validation_last_end_ns.has_value() );
    EXPECT_EQ( analysis.eligibility.packet_total_nonfirst, 0U );
    EXPECT_EQ( analysis.eligibility.packet_structurally_eligible, 0U );
    EXPECT_EQ( analysis.eligibility.packet_fit_eligible, 0U );
    EXPECT_EQ( analysis.eligibility.validation_slots_total, 0U );
    EXPECT_FALSE(
        analysis.eligibility.validation_blocks_eligible.has_value() );
    EXPECT_FALSE( analysis.support.validation_blocks.has_value() );
    EXPECT_FALSE( analysis.support.sufficient.has_value() );
    EXPECT_FALSE( analysis.gates.evaluated );
    EXPECT_FALSE( analysis.gates.blocks_total.has_value() );
    expectFitNullability( analysis.full,
                          GyroAlignmentFitStatus::kSupportInsufficient );
    expectFitNullability( analysis.early,
                          GyroAlignmentFitStatus::kSupportInsufficient );
    expectFitNullability( analysis.late,
                          GyroAlignmentFitStatus::kSupportInsufficient );
  }

  TEST( GyroAlignmentTest, KnownBiasPhysicalOraclePassesAllFourGates )
  {
    const Eigen::Vector3d known_bias{ 0.002, -0.003, 0.001 };
    const auto            result = analyzeGyroAlignment(
        stationaryBiasFixture( known_bias, known_bias ) );
    const auto& analysis = requireAnalysis( result );

    ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kPass );
    EXPECT_EQ( result.verdict.exit_code, 0 );
    EXPECT_TRUE( result.verdict.reason_codes.empty() );
    EXPECT_FALSE( result.error.has_value() );
    for ( const GyroAlignmentFit* fit :
          { &analysis.full, &analysis.early, &analysis.late } )
    {
      expectFitNullability( *fit, GyroAlignmentFitStatus::kSolved );
      ASSERT_TRUE( fit->bias_radps.has_value() );
      EXPECT_TRUE( fit->bias_radps->isApprox( known_bias, 2.0e-12 ) );
      EXPECT_LE( *fit->loss_fit_rad2, *fit->loss_zero_rad2 );
      EXPECT_EQ( fit->nonlinear_nonincrease, true );
      EXPECT_GT( ulpDistance( *fit->loss_fit_rad2,
                              *fit->loss_zero_rad2 ),
                 0U );
    }
    EXPECT_EQ( analysis.full.eligible_duration_ns, 100 * kSecondNs );
    EXPECT_EQ( analysis.early.eligible_duration_ns, 50 * kSecondNs );
    EXPECT_EQ( analysis.late.eligible_duration_ns, 50 * kSecondNs );
    EXPECT_EQ( analysis.eligibility.validation_slots_total, 60U );
    EXPECT_EQ( analysis.eligibility.validation_blocks_eligible, 60U );
    EXPECT_EQ( analysis.support.validation_blocks, 60U );
    EXPECT_EQ( analysis.support.sufficient, true );
    EXPECT_TRUE( analysis.gates.evaluated );
    EXPECT_EQ( analysis.gates.aggregate_reduction, true );
    EXPECT_EQ( analysis.gates.improved_fraction, true );
    EXPECT_EQ( analysis.gates.axis_nonworsening, true );
    EXPECT_EQ( analysis.gates.half_stability, true );
    EXPECT_EQ( analysis.gates.all_passed, true );
    ASSERT_EQ( analysis.blocks.size(), 60U );
    EXPECT_EQ( analysis.blocks.front().block_index, 0U );
    EXPECT_EQ( analysis.blocks.back().block_index, 59U );
  }

  TEST( GyroAlignmentTest, ZeroBiasIsSolvedButFailsPositiveReductionGate )
  {
    const auto  result   = analyzeGyroAlignment( stationaryBiasFixture(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() ) );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( result.verdict.status,
               GyroAlignmentStatus::kHypothesisFail );
    EXPECT_EQ( result.verdict.exit_code, 2 );
    EXPECT_EQ( analysis.full.status, GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( analysis.gates.aggregate_reduction, false );
    EXPECT_EQ( analysis.gates.all_passed, false );
    EXPECT_NE( std::find( result.verdict.reason_codes.begin(),
                          result.verdict.reason_codes.end(),
                          GyroAlignmentReason::kValidationAggregate ),
               result.verdict.reason_codes.end() );
  }

  TEST( GyroAlignmentTest, ExactNinetyAndFortyFiveSecondSupportPasses )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    const auto            result =
        analyzeGyroAlignment( splitSupportFixture( 900'000'000, bias ) );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( analysis.full.eligible_duration_ns, 90 * kSecondNs );
    EXPECT_EQ( analysis.early.eligible_duration_ns, 45 * kSecondNs );
    EXPECT_EQ( analysis.late.eligible_duration_ns, 45 * kSecondNs );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kPass );

    const auto below =
        analyzeGyroAlignment( splitSupportFixture( 899'999'999, bias ) );
    const auto& below_analysis = requireAnalysis( below );
    EXPECT_EQ( below.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( below_analysis.full.status,
               GyroAlignmentFitStatus::kSupportInsufficient );
    EXPECT_EQ( below_analysis.early.status,
               GyroAlignmentFitStatus::kSupportInsufficient );
    EXPECT_EQ( below_analysis.late.status,
               GyroAlignmentFitStatus::kSupportInsufficient );
  }

  TEST( GyroAlignmentTest, FortyOfSixtyImprovedBlocksPassesEquality )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    const auto            result =
        analyzeGyroAlignment( mixedValidationFixture( bias, 40U ) );
    const auto& gates = requireAnalysis( result ).gates;
    EXPECT_EQ( gates.blocks_improved, 40U );
    EXPECT_EQ( gates.blocks_total, 60U );
    EXPECT_EQ( gates.improved_fraction, true );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kPass );

    const auto below =
        analyzeGyroAlignment( mixedValidationFixture( bias, 39U ) );
    const auto& below_gates = requireAnalysis( below ).gates;
    EXPECT_EQ( below_gates.blocks_improved, 39U );
    EXPECT_EQ( below_gates.improved_fraction, false );
    EXPECT_EQ( below.verdict.status,
               GyroAlignmentStatus::kHypothesisFail );
  }

  TEST( GyroAlignmentTest, AggregateCompositeUsesFrozenPhysicalBracket )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    const auto            inside =
        analyzeGyroAlignment( mixedValidationFixture( bias, 34U ) );
    const auto outside =
        analyzeGyroAlignment( mixedValidationFixture( bias, 33U ) );
    const auto& inside_gates  = requireAnalysis( inside ).gates;
    const auto& outside_gates = requireAnalysis( outside ).gates;
    ASSERT_TRUE( inside_gates.validation_loss_zero_rad2.has_value() );
    ASSERT_TRUE( inside_gates.validation_loss_fit_rad2.has_value() );
    ASSERT_TRUE( outside_gates.validation_loss_zero_rad2.has_value() );
    ASSERT_TRUE( outside_gates.validation_loss_fit_rad2.has_value() );
    const double inside_margin =
        0.8 * *inside_gates.validation_loss_zero_rad2 -
        *inside_gates.validation_loss_fit_rad2;
    const double outside_margin =
        *outside_gates.validation_loss_fit_rad2 -
        0.8 * *outside_gates.validation_loss_zero_rad2;
    const double packet_loss =
        ( bias.x() * bias.x() + bias.y() * bias.y() ) +
        bias.z() * bias.z();
    double expected_inside_zero  = 0.0;
    double expected_inside_fit   = 0.0;
    double expected_outside_zero = 0.0;
    double expected_outside_fit  = 0.0;
    for ( std::size_t i = 0; i < 60U; ++i )
    {
      expected_inside_zero += i < 34U ? packet_loss : 0.0;
      expected_inside_fit += i < 34U ? 0.0 : packet_loss;
      expected_outside_zero += i < 33U ? packet_loss : 0.0;
      expected_outside_fit += i < 33U ? 0.0 : packet_loss;
    }
    EXPECT_EQ( positiveBits( packet_loss ), 0x3eed5c31593e5fb8ULL );
    EXPECT_EQ( positiveBits( expected_inside_zero ),
               0x3f3f31f46ed245baULL );
    EXPECT_EQ( positiveBits( expected_inside_fit ),
               0x3f37dae81882adcaULL );
    EXPECT_EQ( positiveBits( expected_outside_zero ),
               0x3f3e4712e40852bcULL );
    EXPECT_EQ( positiveBits( expected_outside_fit ),
               0x3f38c5c9a34ca0c8ULL );
    EXPECT_NEAR( *inside_gates.validation_loss_zero_rad2,
                 expected_inside_zero, 1.0e-12 );
    EXPECT_NEAR( *inside_gates.validation_loss_fit_rad2,
                 expected_inside_fit, 1.0e-12 );
    EXPECT_NEAR( *outside_gates.validation_loss_zero_rad2,
                 expected_outside_zero, 1.0e-12 );
    EXPECT_NEAR( *outside_gates.validation_loss_fit_rad2,
                 expected_outside_fit, 1.0e-12 );
    EXPECT_GT( inside_margin, 0.0 );
    EXPECT_GT( outside_margin, 0.0 );
    EXPECT_GT( ulpDistance(
                   *inside_gates.validation_loss_fit_rad2,
                   0.8 * *inside_gates.validation_loss_zero_rad2 ),
               0U );
    EXPECT_GT( ulpDistance(
                   *outside_gates.validation_loss_fit_rad2,
                   0.8 * *outside_gates.validation_loss_zero_rad2 ),
               0U );
    EXPECT_EQ( inside_gates.aggregate_reduction, true );
    EXPECT_EQ( outside_gates.aggregate_reduction, false );
  }

  TEST( GyroAlignmentTest, PerAxisCompositeUsesFrozenPhysicalBracket )
  {
    const auto inside =
        analyzeGyroAlignment( axisValidationFixture( 29U ) );
    const auto outside =
        analyzeGyroAlignment( axisValidationFixture( 31U ) );
    const auto& inside_gates  = requireAnalysis( inside ).gates;
    const auto& outside_gates = requireAnalysis( outside ).gates;
    ASSERT_TRUE( inside_gates.axis_loss_zero_rad2.has_value() );
    ASSERT_TRUE( inside_gates.axis_loss_fit_rad2.has_value() );
    ASSERT_TRUE( outside_gates.axis_loss_zero_rad2.has_value() );
    ASSERT_TRUE( outside_gates.axis_loss_fit_rad2.has_value() );
    constexpr double kXLoss = 0.002 * 0.002;
    EXPECT_EQ( positiveBits( 29.0 * kXLoss ), 0x3f1e68a0d349be90ULL );
    EXPECT_EQ( positiveBits( 31.0 * kXLoss ), 0x3f2040bfe3b03e21ULL );
    EXPECT_NEAR( inside_gates.axis_loss_fit_rad2->x(), 29.0 * kXLoss,
                 1.0e-12 );
    EXPECT_NEAR( inside_gates.axis_loss_zero_rad2->x(), 31.0 * kXLoss,
                 1.0e-12 );
    EXPECT_NEAR( outside_gates.axis_loss_fit_rad2->x(), 31.0 * kXLoss,
                 1.0e-12 );
    EXPECT_NEAR( outside_gates.axis_loss_zero_rad2->x(), 29.0 * kXLoss,
                 1.0e-12 );
    EXPECT_LT( inside_gates.axis_loss_fit_rad2->x(),
               inside_gates.axis_loss_zero_rad2->x() );
    EXPECT_GT( ulpDistance( inside_gates.axis_loss_fit_rad2->x(),
                            inside_gates.axis_loss_zero_rad2->x() ),
               0U );
    EXPECT_EQ( inside_gates.axis_nonworsening, true );
    EXPECT_GT( outside_gates.axis_loss_fit_rad2->x(),
               outside_gates.axis_loss_zero_rad2->x() );
    EXPECT_GT( ulpDistance( outside_gates.axis_loss_fit_rad2->x(),
                            outside_gates.axis_loss_zero_rad2->x() ),
               0U );
    EXPECT_EQ( outside_gates.axis_nonworsening, false );
    EXPECT_EQ( outside.verdict.status,
               GyroAlignmentStatus::kHypothesisFail );
  }

  TEST( GyroAlignmentTest, HalfStabilityCompositeUsesFrozenTwoSidedFixtures )
  {
    const auto inside = analyzeGyroAlignment( halfBiasFixture( 0.000999 ) );
    const auto outside =
        analyzeGyroAlignment( halfBiasFixture( 0.001001 ) );
    const auto& inside_gates  = requireAnalysis( inside ).gates;
    const auto& outside_gates = requireAnalysis( outside ).gates;
    ASSERT_TRUE( inside_gates.half_bias_max_abs_diff_radps.has_value() );
    ASSERT_TRUE( outside_gates.half_bias_max_abs_diff_radps.has_value() );
    EXPECT_NEAR( *inside_gates.half_bias_max_abs_diff_radps, 0.000999,
                 1.0e-12 );
    EXPECT_NEAR( *outside_gates.half_bias_max_abs_diff_radps, 0.001001,
                 1.0e-12 );
    EXPECT_EQ( positiveBits( 0.000999 ), 0x3f505e1c15097c81ULL );
    EXPECT_EQ( positiveBits( 0.001001 ), 0x3f50667f90d9d777ULL );
    EXPECT_LT( *inside_gates.half_bias_max_abs_diff_radps, 1.0e-3 );
    EXPECT_GT( *outside_gates.half_bias_max_abs_diff_radps, 1.0e-3 );
    EXPECT_GT( 1.0e-3 - *inside_gates.half_bias_max_abs_diff_radps,
               0.0 );
    EXPECT_GT( *outside_gates.half_bias_max_abs_diff_radps - 1.0e-3,
               0.0 );
    EXPECT_GT( ulpDistance( *inside_gates.half_bias_max_abs_diff_radps,
                            1.0e-3 ),
               0U );
    EXPECT_GT( ulpDistance( *outside_gates.half_bias_max_abs_diff_radps,
                            1.0e-3 ),
               0U );
    EXPECT_EQ( inside_gates.half_stability, true );
    EXPECT_EQ( outside_gates.half_stability, false );
  }

  TEST( GyroAlignmentTest,
        IndependentBinary64OracleFreezesAllCompositeBrackets )
  {
    struct FrozenBracket
    {
      std::uint64_t threshold_bits;
      std::uint64_t inside_bits;
      std::uint64_t outside_bits;
    };
    constexpr std::array<FrozenBracket, 4> kLessEqualBrackets{
        FrozenBracket{ 0x3fe999999999999aULL, 0x3fe9999999999999ULL,
                       0x3fe999999999999bULL },
        FrozenBracket{ 0x3f50624dd2f1a9fcULL, 0x3f50624dd2f1a9fbULL,
                       0x3f50624dd2f1a9fdULL },
        FrozenBracket{ 0x412e848000000000ULL, 0x412e847fffffffffULL,
                       0x412e848000000001ULL },
        FrozenBracket{ 0x3ff0000000000000ULL, 0x3fefffffffffffffULL,
                       0x3ff0000000000001ULL },
    };
    for ( const auto bracket : kLessEqualBrackets )
    {
      const double threshold = std::bit_cast<double>( bracket.threshold_bits );
      const double inside    = std::bit_cast<double>( bracket.inside_bits );
      const double outside   = std::bit_cast<double>( bracket.outside_bits );
      EXPECT_LE( inside, threshold );
      EXPECT_GT( outside, threshold );
      EXPECT_EQ( ulpDistance( inside, threshold ), 1U );
      EXPECT_EQ( ulpDistance( outside, threshold ), 1U );
      EXPECT_GT( threshold - inside, 0.0 );
      EXPECT_GT( outside - threshold, 0.0 );
    }

    constexpr std::uint64_t kNearPiThresholdBits = 0x3ee500b588e368f2ULL;
    const double            near_pi_threshold =
        std::bit_cast<double>( kNearPiThresholdBits );
    const double near_pi_inside =
        std::bit_cast<double>( kNearPiThresholdBits - 1U );
    const double near_pi_outside =
        std::bit_cast<double>( kNearPiThresholdBits + 1U );
    EXPECT_LE( near_pi_inside, near_pi_threshold );
    EXPECT_GT( near_pi_outside, near_pi_threshold );
    EXPECT_EQ( ulpDistance( near_pi_inside, near_pi_threshold ), 1U );
    EXPECT_EQ( ulpDistance( near_pi_outside, near_pi_threshold ), 1U );

    const double aggregate_threshold =
        std::bit_cast<double>( kLessEqualBrackets[ 0 ].threshold_bits );
    EXPECT_EQ( aggregate_threshold, 0.8 );
    const double half_threshold =
        std::bit_cast<double>( kLessEqualBrackets[ 1 ].threshold_bits );
    EXPECT_EQ( half_threshold, 1.0e-3 );
    const double condition_threshold =
        std::bit_cast<double>( kLessEqualBrackets[ 2 ].threshold_bits );
    EXPECT_EQ( condition_threshold, 1.0e6 );
    const double nonlinear_loss_zero =
        std::bit_cast<double>( kLessEqualBrackets[ 3 ].threshold_bits );
    const double nonlinear_inside =
        std::bit_cast<double>( kLessEqualBrackets[ 3 ].inside_bits );
    const double nonlinear_outside =
        std::bit_cast<double>( kLessEqualBrackets[ 3 ].outside_bits );
    EXPECT_LE( nonlinear_inside, nonlinear_loss_zero );
    EXPECT_GT( nonlinear_outside, nonlinear_loss_zero );
  }

  TEST( GyroAlignmentTest, FdGuardCoversEverySignedAxisCrossing )
  {
    const std::array axes{ Eigen::Vector3d::UnitX(),
                           Eigen::Vector3d::UnitY(),
                           Eigen::Vector3d::UnitZ() };
    for ( const auto& axis : axes )
    {
      for ( const double sign : { -1.0, 1.0 } )
      {
        const Eigen::Matrix3d r_vis = rotationVectorQuaternion(
                                          sign *
                                          ( std::numbers::pi - 5.0e-6 ) * axis )
                                          .toRotationMatrix();
        const double oracle_gap =
            std::numbers::pi - principalLogOracle( r_vis ).norm();
        const double frozen_theta = std::numbers::pi - 5.0e-6;
        const double frozen_gap   = std::numbers::pi - frozen_theta;
        EXPECT_EQ( positiveBits( frozen_theta ), 0x400921f8b52d7bfcULL );
        EXPECT_EQ( positiveBits( frozen_gap ), 0x3ed4f8b588e00000ULL );
        EXPECT_NEAR( oracle_gap, frozen_gap, 2.0e-15 );
        const double oracle_threshold =
            kEpsPi + std::nextafter(
                         1.0e-5,
                         std::numeric_limits<double>::infinity() );
        EXPECT_LE( oracle_gap, oracle_threshold );
        EXPECT_GT( oracle_threshold - oracle_gap, 0.0 );
        EXPECT_GT( ulpDistance( oracle_gap, oracle_threshold ), 0U );
        const auto  result   = analyzeGyroAlignment( nearPiPacketFixture(
            1.0, axis, sign * ( std::numbers::pi - 5.0e-6 ) ) );
        const auto& analysis = requireAnalysis( result );
        EXPECT_EQ( analysis.eligibility.packet_total_nonfirst, 1U );
        EXPECT_EQ( analysis.eligibility.packet_structurally_eligible, 1U );
        EXPECT_EQ( analysis.eligibility.packet_fit_eligible, 0U );
        EXPECT_EQ( analysis.eligibility.interval_excluded_counts.near_pi,
                   1U );
        EXPECT_EQ( result.verdict.reason_codes,
                   std::vector{ GyroAlignmentReason::kFitSupport } );
      }
    }
  }

  TEST( GyroAlignmentTest, FdMarginScalesWithIntervalSecondsNotNanoseconds )
  {
    const double theta        = std::numbers::pi - 5.0e-6;
    const auto   short_result = analyzeGyroAlignment(
        nearPiPacketFixture( 0.1, Eigen::Vector3d::UnitX(), theta ) );
    const auto long_result = analyzeGyroAlignment(
        nearPiPacketFixture( 1.0, Eigen::Vector3d::UnitX(), theta ) );
    const auto& short_e = requireAnalysis( short_result ).eligibility;
    const auto& long_e  = requireAnalysis( long_result ).eligibility;
    EXPECT_EQ( short_e.packet_fit_eligible, 1U );
    EXPECT_EQ( short_e.interval_excluded_counts.near_pi, 0U );
    EXPECT_EQ( long_e.packet_fit_eligible, 0U );
    EXPECT_EQ( long_e.interval_excluded_counts.near_pi, 1U );
  }

  TEST( GyroAlignmentTest, IndependentOracleRejectsFdStencilMutants )
  {
    constexpr double kCrossingGap = 5.0e-6;
    EXPECT_TRUE( fdGuardExcludes( kCrossingGap, 1.0,
                                  FdGuardOracle::kFrozen ) );
    EXPECT_FALSE( fdGuardExcludes( kCrossingGap, 1.0,
                                   FdGuardOracle::kMissingMargin ) );
    EXPECT_TRUE( fdGuardExcludes( kCrossingGap, 0.1,
                                  FdGuardOracle::kNanosecondsAsSeconds ) );
    EXPECT_FALSE( fdGuardExcludes( kCrossingGap, 0.1,
                                   FdGuardOracle::kFrozen ) );
    constexpr double kOverExclusionGap = 1.5e-5;
    EXPECT_FALSE( fdGuardExcludes( kOverExclusionGap, 1.0,
                                   FdGuardOracle::kFrozen ) );
    EXPECT_TRUE( fdGuardExcludes( kOverExclusionGap, 1.0,
                                  FdGuardOracle::kDoubleMargin ) );

    const SevenPointTrace trace = sevenPointTraceOracle();
    ASSERT_EQ( trace.bias_points.size(), 7U );
    for ( std::size_t i = 1U; i < trace.bias_points.size(); ++i )
    {
      EXPECT_EQ( ( trace.bias_points[ i ].array() != 0.0 ).count(), 1 );
    }
    constexpr double      kH = 1.0e-5;
    const Eigen::Vector3d compound_mutant{ kH, kH, 0.0 };
    EXPECT_EQ( ( compound_mutant.array() != 0.0 ).count(), 2 );
    bool compound_found = false;
    for ( const auto& point : trace.bias_points )
    {
      compound_found = compound_found ||
                       ( point.array() == compound_mutant.array() ).all();
    }
    EXPECT_FALSE( compound_found );

    for ( std::size_t omitted = 1U; omitted < trace.bias_points.size();
          ++omitted )
    {
      auto omission_mutant = trace.bias_points;
      omission_mutant.erase( omission_mutant.begin() +
                             static_cast<std::ptrdiff_t>( omitted ) );
      EXPECT_EQ( omission_mutant.size(), 6U );
      const std::size_t partner_index =
          omitted % 2U == 0U ? omitted - 1U : omitted + 1U;
      const Eigen::Vector3d signed_partner =
          trace.bias_points.at( partner_index );
      EXPECT_TRUE( std::any_of(
          omission_mutant.begin(), omission_mutant.end(),
          [ &signed_partner ]( const auto& point ) {
            return ( point.array() == signed_partner.array() ).all();
          } ) );
    }

    auto add_compound_mutant = trace.bias_points;
    add_compound_mutant.push_back( compound_mutant );
    EXPECT_EQ( add_compound_mutant.size(), 8U );
    auto replace_mutant = trace.bias_points;
    replace_mutant[ 1 ] = compound_mutant;
    EXPECT_EQ( replace_mutant.size(), 7U );
    EXPECT_EQ( ( replace_mutant[ 1 ].array() != 0.0 ).count(), 2 );
    auto wrong_order_mutant = trace.bias_points;
    std::swap( wrong_order_mutant[ 1 ], wrong_order_mutant[ 3 ] );
    EXPECT_FALSE( std::equal(
        wrong_order_mutant.begin(), wrong_order_mutant.end(),
        trace.bias_points.begin(), []( const auto& lhs, const auto& rhs ) {
          return ( lhs.array() == rhs.array() ).all();
        } ) );

    const auto canonical_physical = canonicalBiasPoints();
    const auto physical_correct =
        physicalStencilOracle( canonical_physical );
    ASSERT_TRUE( physical_correct.shape_valid );
    EXPECT_LE( ( physical_correct.candidate_bias -
                 Eigen::Vector3d{ 0.002, -0.003, 0.001 } )
                   .norm(),
               2.0e-8 );
    EXPECT_TRUE( std::isfinite( physical_correct.candidate_loss ) );
    for ( std::size_t omitted = 1U; omitted < canonical_physical.size();
          ++omitted )
    {
      auto physical_omission = canonical_physical;
      physical_omission.erase(
          physical_omission.begin() +
          static_cast<std::ptrdiff_t>( omitted ) );
      const auto omission_oracle =
          physicalStencilOracle( physical_omission );
      EXPECT_FALSE( omission_oracle.shape_valid );
      EXPECT_TRUE( std::isinf( omission_oracle.candidate_loss ) );
    }
    auto physical_add = canonical_physical;
    physical_add.push_back( compound_mutant );
    const auto add_oracle = physicalStencilOracle( physical_add );
    EXPECT_FALSE( add_oracle.shape_valid );
    auto physical_replace     = canonical_physical;
    physical_replace[ 1 ]     = compound_mutant;
    const auto replace_oracle = physicalStencilOracle( physical_replace );
    EXPECT_FALSE( replace_oracle.shape_valid );
    EXPECT_FALSE( replace_oracle.candidate_bias.isApprox(
        physical_correct.candidate_bias, 1.0e-12 ) );
    auto physical_wrong_order = canonical_physical;
    std::swap( physical_wrong_order[ 1 ], physical_wrong_order[ 3 ] );
    const auto wrong_order_oracle =
        physicalStencilOracle( physical_wrong_order );
    EXPECT_FALSE( wrong_order_oracle.shape_valid );
    EXPECT_FALSE( wrong_order_oracle.candidate_bias.isApprox(
        physical_correct.candidate_bias, 1.0e-12 ) );

    const auto noncommuting_public =
        analyzeGyroAlignment( noncommutingFixture() );
    const auto& public_full = requireAnalysis( noncommuting_public ).full;
    ASSERT_TRUE( public_full.bias_radps.has_value() );
    EXPECT_LE( ( *public_full.bias_radps - physical_correct.candidate_bias )
                   .norm(),
               2.0e-7 );

    const auto  public_result = analyzeGyroAlignment( nearPiPacketFixture(
        1.0, Eigen::Vector3d::UnitX(),
        std::numbers::pi - kCrossingGap ) );
    const auto& public_eligibility =
        requireAnalysis( public_result ).eligibility;
    EXPECT_EQ( public_eligibility.packet_fit_eligible, 0U );
    EXPECT_EQ( public_eligibility.interval_excluded_counts.near_pi, 1U );
  }

  TEST( GyroAlignmentTest, PrincipalLogOracleCoversNearZeroAndGeneralBranch )
  {
    const Eigen::Vector3d tiny{ 1.0e-10, -2.0e-10, 3.0e-10 };
    const Eigen::Vector3d general{ 0.2, -0.1, 0.3 };
    const auto            tiny_q = rotationVectorQuaternion( tiny ).toRotationMatrix();
    const auto            general_q =
        rotationVectorQuaternion( general ).toRotationMatrix();
    EXPECT_TRUE( principalLogOracle( tiny_q ).isApprox(
        Eigen::Vector3d{ 1.0e-10, -2.0e-10, 3.0e-10 }, 1.0e-18 ) );
    EXPECT_TRUE(
        principalLogOracle( general_q ).isApprox( general, 1.0e-14 ) );

    const auto result = analyzeGyroAlignment( stationaryBiasFixture(
        Eigen::Vector3d{ 0.2, -0.1, 0.3 },
        Eigen::Vector3d{ 0.2, -0.1, 0.3 } ) );
    ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kPass );
    const auto& block = requireAnalysis( result ).blocks.front();
    EXPECT_LE( block.r_fit_rad.norm(), 1.0e-12 );
  }

  TEST( GyroAlignmentTest, RankDeficientPhysicalFixtureHasExactSupport )
  {
    GyroAlignmentInput input;
    input.endpoints.push_back( endpoint( 0 ) );
    input.packets.push_back( firstZero( 0 ) );
    constexpr std::int64_t kDtNs        = 20'000'000;
    constexpr double       kRate        = 2.0 * std::numbers::pi / 0.02;
    std::int64_t           timestamp_ns = 0;
    for ( std::size_t half = 0; half < 2U; ++half )
    {
      for ( std::size_t i = 0; i < 2'250U; ++i )
      {
        const auto next = timestamp_ns + kDtNs;
        input.packets.push_back( validPacket(
            input.packets.size(), timestamp_ns, next,
            Eigen::Vector3d{ kRate, 0.0, 0.0 } ) );
        input.endpoints.push_back( endpoint( next ) );
        timestamp_ns = next;
      }
      const auto boundary =
          static_cast<std::int64_t>( half + 1U ) * 50 * kSecondNs;
      input.packets.push_back(
          gapPacket( input.packets.size(), timestamp_ns, boundary ) );
      input.endpoints.push_back( endpoint( boundary ) );
      timestamp_ns = boundary;
    }

    const auto  result   = analyzeGyroAlignment( input );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( analysis.full.eligible_duration_ns, 90 * kSecondNs );
    EXPECT_EQ( analysis.early.eligible_duration_ns, 45 * kSecondNs );
    EXPECT_EQ( analysis.late.eligible_duration_ns, 45 * kSecondNs );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    for ( const GyroAlignmentFit* fit :
          { &analysis.full, &analysis.early, &analysis.late } )
    {
      EXPECT_EQ( fit->status, GyroAlignmentFitStatus::kRankDeficient );
      EXPECT_TRUE( fit->singular_values.has_value() );
      EXPECT_TRUE( fit->rank.has_value() );
      EXPECT_LT( *fit->rank, 3U );
      EXPECT_FALSE( fit->condition.has_value() );
      EXPECT_FALSE( fit->bias_radps.has_value() );
    }
  }

  TEST( GyroAlignmentTest,
        FrozenPhysicalNonlinearLossBracketHasIndependentBinary64Oracle )
  {
    // Frozen offline from an independent NumPy Rodrigues/Log/SVD model, then
    // calibrated for the typed endpoint roundoff with a direct GTSAM Q2
    // harness. Neither path calls the Q3 analyzer.
    constexpr double        kFailLossZero  = 0x1.4a5d96e2a692ap+7;
    constexpr double        kFailLossFit   = 0x1.8484a50793e01p+7;
    constexpr double        kFailMargin    = 0x1.d13871276a6b8p+4;
    constexpr std::uint64_t kFailUlpMargin = 0x3a270e24ed4d7ULL;
    constexpr double        kPassLossZero  = 0x1.294573a79788fp-11;
    constexpr double        kPassLossFit   = 0x1.61cb49fffffffp-99;
    constexpr double        kPassMargin    = 0x1.294573a79788fp-11;
    constexpr std::uint64_t kPassUlpMargin = 0x57c77a29a797890ULL;
    const Eigen::Vector3d   kFailBias{
        -0x1.178c3f1a9aba1p+1,
        0x1.718a102fad2bap-2,
        -0x1.e83a506eafc14p+1,
    };
    const Eigen::Vector3d kPassBias{
        0x1.0624dd2f1a894p-9,
        -0x1.89374bc6a7d99p-9,
        0x1.0624dd2f1a896p-10,
    };

    const auto fail_rows   = frozenNonlinearRows();
    const auto fail_oracle = fitNonlinearRowsOracle( fail_rows );
    ASSERT_EQ( fail_rows.size(), 30U );
    double fail_duration_s = 0.0;
    for ( const NonlinearRow& row : fail_rows )
    {
      fail_duration_s += row.duration_s;
    }
    EXPECT_EQ( positiveBits( fail_duration_s ), 0x4047400000000000ULL );
    EXPECT_TRUE( fail_oracle.bias.isApprox( kFailBias, 2.0e-8 ) );
    EXPECT_NEAR( fail_oracle.condition, 0x1.4921ee75ca655p+2,
                 2.0e-8 );
    EXPECT_NEAR( fail_oracle.loss_zero, kFailLossZero, 2.0e-11 );
    EXPECT_NEAR( fail_oracle.loss_fit, kFailLossFit, 2.0e-7 );
    EXPECT_NEAR( fail_oracle.candidate_gap, 0x1.84531f28bb770p-2,
                 2.0e-8 );
    EXPECT_GT( fail_oracle.loss_fit, fail_oracle.loss_zero );

    const NonlinearRow kPassRow{
        0x1.ccccccccccccdp-1,
        Eigen::Vector3d{ 0x1.0624dd2f1a9fcp-9,
                         -0x1.89374bc6a7efap-9,
                         0x1.0624dd2f1a9fcp-10 },
        Eigen::Vector3d::Zero(),
    };
    const std::vector<NonlinearRow> pass_rows( 50U, kPassRow );
    const auto                      pass_oracle = fitNonlinearRowsOracle( pass_rows );
    EXPECT_TRUE( pass_oracle.bias.isApprox( kPassBias, 2.0e-12 ) );
    EXPECT_NEAR( pass_oracle.loss_zero, kPassLossZero, 2.0e-15 );
    EXPECT_NEAR( pass_oracle.loss_fit, kPassLossFit, 1.0e-28 );
    EXPECT_LT( pass_oracle.loss_fit, pass_oracle.loss_zero );

    EXPECT_EQ( positiveBits( kFailLossZero ), 0x4064a5d96e2a692aULL );
    EXPECT_EQ( positiveBits( kFailLossFit ), 0x4068484a50793e01ULL );
    EXPECT_EQ( positiveBits( kFailLossFit - kFailLossZero ),
               0x403d13871276a6b8ULL );
    EXPECT_EQ( kFailLossFit - kFailLossZero, kFailMargin );
    EXPECT_EQ( ulpDistance( kFailLossFit, kFailLossZero ),
               kFailUlpMargin );
    EXPECT_EQ( positiveBits( kPassLossZero ), 0x3f4294573a79788fULL );
    EXPECT_EQ( positiveBits( kPassLossFit ), 0x39c61cb49fffffffULL );
    EXPECT_EQ( kPassLossZero - kPassLossFit, kPassMargin );
    EXPECT_EQ( ulpDistance( kPassLossZero, kPassLossFit ),
               kPassUlpMargin );

    const auto  pass_result = analyzeGyroAlignment( splitSupportFixture(
        900'000'000, kPassRow.gyro_radps ) );
    const auto& pass_fit    = requireAnalysis( pass_result ).early;
    ASSERT_EQ( pass_fit.status, GyroAlignmentFitStatus::kSolved );
    ASSERT_TRUE( pass_fit.loss_zero_rad2.has_value() );
    ASSERT_TRUE( pass_fit.loss_fit_rad2.has_value() );
    ASSERT_TRUE( pass_fit.bias_radps.has_value() );
    EXPECT_EQ( pass_fit.nonlinear_nonincrease, true );
    EXPECT_TRUE( pass_fit.bias_radps->isApprox( kPassBias, 2.0e-12 ) );
    EXPECT_NEAR( *pass_fit.loss_zero_rad2, kPassLossZero, 2.0e-15 );
    EXPECT_NEAR( *pass_fit.loss_fit_rad2, kPassLossFit, 1.0e-28 );
    EXPECT_GT( *pass_fit.loss_zero_rad2 - *pass_fit.loss_fit_rad2,
               5.0e-4 );
    EXPECT_GE( ulpDistance( *pass_fit.loss_zero_rad2,
                            *pass_fit.loss_fit_rad2 ),
               kPassUlpMargin / 2U );
  }

  TEST( GyroAlignmentTest,
        PhysicalNonlinearAndRankSiblingsPreserveOrderAndPrecedence )
  {
    constexpr double                       kFailLossZero  = 0x1.4a5d96e2a692ap+7;
    constexpr double                       kFailLossFit   = 0x1.8484a50793e01p+7;
    constexpr std::uint64_t                kFailUlpMargin = 0x3a270e24ed4d7ULL;
    const std::vector<GyroAlignmentReason> kExpectedReasons{
        GyroAlignmentReason::kFitRank,
        GyroAlignmentReason::kPrefixNonlinearLoss,
    };

    const auto nonlinear_early =
        analyzeGyroAlignment( nonlinearRankSiblingFixture( true ) );
    const auto nonlinear_late =
        analyzeGyroAlignment( nonlinearRankSiblingFixture( false ) );
    const auto& early_analysis = requireAnalysis( nonlinear_early );
    const auto& late_analysis  = requireAnalysis( nonlinear_late );

    EXPECT_EQ( nonlinear_early.verdict.status,
               GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( nonlinear_late.verdict.status,
               GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( nonlinear_early.verdict.exit_code, 3 );
    EXPECT_EQ( nonlinear_late.verdict.exit_code, 3 );
    EXPECT_EQ( nonlinear_early.verdict.reason_codes, kExpectedReasons );
    EXPECT_EQ( nonlinear_late.verdict.reason_codes, kExpectedReasons );

    EXPECT_EQ( early_analysis.full.status,
               GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( early_analysis.early.status,
               GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( early_analysis.late.status,
               GyroAlignmentFitStatus::kRankDeficient );
    EXPECT_EQ( late_analysis.full.status,
               GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( late_analysis.early.status,
               GyroAlignmentFitStatus::kRankDeficient );
    EXPECT_EQ( late_analysis.late.status,
               GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( early_analysis.full.nonlinear_nonincrease, true );
    EXPECT_EQ( late_analysis.full.nonlinear_nonincrease, true );

    const GyroAlignmentFit& early_fail = early_analysis.early;
    const GyroAlignmentFit& late_fail  = late_analysis.late;
    for ( const GyroAlignmentFit* fit : { &early_fail, &late_fail } )
    {
      ASSERT_TRUE( fit->loss_zero_rad2.has_value() );
      ASSERT_TRUE( fit->loss_fit_rad2.has_value() );
      EXPECT_EQ( fit->nonlinear_nonincrease, false );
      EXPECT_NEAR( *fit->loss_zero_rad2, kFailLossZero, 2.0e-11 );
      EXPECT_NEAR( *fit->loss_fit_rad2, kFailLossFit, 2.0e-7 );
      EXPECT_GT( *fit->loss_fit_rad2 - *fit->loss_zero_rad2, 29.0 );
      EXPECT_GE( ulpDistance( *fit->loss_fit_rad2,
                              *fit->loss_zero_rad2 ),
                 kFailUlpMargin / 2U );
    }
    EXPECT_EQ( early_analysis.full.eligible_duration_ns,
               95'500'000'000 );
    EXPECT_EQ( early_analysis.early.eligible_duration_ns,
               46'500'000'000 );
    EXPECT_EQ( early_analysis.late.eligible_duration_ns,
               45'000'000'000 );
    EXPECT_EQ( late_analysis.full.eligible_duration_ns,
               95'500'000'000 );
    EXPECT_EQ( late_analysis.early.eligible_duration_ns,
               45'000'000'000 );
    EXPECT_EQ( late_analysis.late.eligible_duration_ns,
               46'500'000'000 );
    EXPECT_FALSE( early_analysis.gates.evaluated );
    EXPECT_FALSE( late_analysis.gates.evaluated );
  }

  TEST( GyroAlignmentTest,
        IndependentTraceFreezesSiblingOrderAndVerdictPrecedence )
  {
    const auto inconclusive_first = evaluateSiblingTraceOracle(
        { FitTraceStatus::kInconclusive, FitTraceStatus::kSolvedIncrease,
          FitTraceStatus::kSolvedNonincrease } );
    const auto hypothesis_first = evaluateSiblingTraceOracle(
        { FitTraceStatus::kSolvedIncrease, FitTraceStatus::kInconclusive,
          FitTraceStatus::kSolvedNonincrease } );
    const std::vector<std::string> frozen_order{ "full", "early", "late" };
    EXPECT_EQ( inconclusive_first.evaluated, frozen_order );
    EXPECT_EQ( hypothesis_first.evaluated, frozen_order );
    EXPECT_EQ( inconclusive_first.status,
               GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( hypothesis_first.status,
               GyroAlignmentStatus::kInconclusive );
    EXPECT_TRUE( inconclusive_first.analysis_available );
    EXPECT_TRUE( hypothesis_first.analysis_available );

    const auto later_hard = evaluateSiblingTraceOracle(
        { FitTraceStatus::kSolvedIncrease, FitTraceStatus::kHard,
          FitTraceStatus::kSolvedNonincrease } );
    EXPECT_EQ( later_hard.status, GyroAlignmentStatus::kHardError );
    EXPECT_EQ( later_hard.evaluated,
               ( std::vector<std::string>{ "full", "early" } ) );
    EXPECT_FALSE( later_hard.analysis_available );

    GyroAlignmentInput public_hard_input;
    public_hard_input.endpoints = { endpoint( 0 ), endpoint( kSecondNs ) };
    public_hard_input.packets   = {
        firstZero( 0 ),
        validPacket( 1U, 0, kSecondNs, Eigen::Vector3d::Zero() ),
    };
    public_hard_input.endpoints[ 1 ].q_WB =
        Eigen::Quaterniond{ 2.0, 0.0, 0.0, 0.0 };
    const auto public_hard = analyzeGyroAlignment( public_hard_input );
    EXPECT_EQ( public_hard.verdict.status, GyroAlignmentStatus::kHardError );
    EXPECT_EQ( public_hard.verdict.exit_code, 1 );
    EXPECT_FALSE( public_hard.analysis.has_value() );
  }

  TEST( GyroAlignmentTest, ConditionBoundaryUsesFrozenPhysicalBracket )
  {
    const auto  inside      = analyzeGyroAlignment( conditionFixture( 1.0e-5 ) );
    const auto  outside     = analyzeGyroAlignment( conditionFixture( 5.0e-6 ) );
    const auto& inside_fit  = requireAnalysis( inside ).full;
    const auto& outside_fit = requireAnalysis( outside ).full;
    ASSERT_TRUE( inside_fit.condition.has_value() );
    ASSERT_TRUE( outside_fit.condition.has_value() );
    const double inside_oracle  = conditionMetricOracle( 1.0e-5 );
    const double outside_oracle = conditionMetricOracle( 5.0e-6 );
    const double inside_formula =
        ( 2.0 * std::numbers::pi - 1.0e-5 ) / 1.0e-5;
    const double outside_formula =
        ( 2.0 * std::numbers::pi - 5.0e-6 ) / 5.0e-6;
    EXPECT_EQ( positiveBits( inside_formula ), 0x41232cbb0fba43a7ULL );
    EXPECT_EQ( positiveBits( outside_formula ), 0x41332cbc0fba43a7ULL );
    EXPECT_NEAR( inside_oracle, inside_formula, 2.0 );
    EXPECT_NEAR( outside_oracle, outside_formula, 4.0 );
    EXPECT_NEAR( *inside_fit.condition, inside_oracle,
                 1.0e-7 * inside_oracle );
    EXPECT_NEAR( *outside_fit.condition, outside_oracle,
                 1.0e-7 * outside_oracle );
    EXPECT_LT( *inside_fit.condition, 1.0e6 );
    EXPECT_GT( *outside_fit.condition, 1.0e6 );
    EXPECT_GT( 1.0e6 - *inside_fit.condition, 0.0 );
    EXPECT_GT( *outside_fit.condition - 1.0e6, 0.0 );
    EXPECT_GT( ulpDistance( *inside_fit.condition, 1.0e6 ), 0U );
    EXPECT_GT( ulpDistance( *outside_fit.condition, 1.0e6 ), 0U );
    EXPECT_EQ( inside_fit.status, GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( outside_fit.status,
               GyroAlignmentFitStatus::kIllConditioned );
    EXPECT_FALSE( outside_fit.bias_radps.has_value() );
    EXPECT_FALSE( outside_fit.loss_fit_rad2.has_value() );
  }

  TEST( GyroAlignmentTest,
        CandidateNearPiIsObservabilityFailureWithoutPublishingBias )
  {
    const auto result =
        analyzeGyroAlignment( candidateObservabilityFixture() );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_NE( std::find( result.verdict.reason_codes.begin(),
                          result.verdict.reason_codes.end(),
                          GyroAlignmentReason::kFitObservability ),
               result.verdict.reason_codes.end() );
    for ( const GyroAlignmentFit* fit :
          { &analysis.full, &analysis.early, &analysis.late } )
    {
      EXPECT_EQ( fit->status,
                 GyroAlignmentFitStatus::kObservabilityFailed );
      EXPECT_FALSE( fit->bias_radps.has_value() );
      EXPECT_TRUE( fit->singular_values.has_value() );
      EXPECT_EQ( fit->rank, 3U );
      EXPECT_TRUE( fit->condition.has_value() );
      EXPECT_FALSE( fit->loss_zero_rad2.has_value() );
      EXPECT_FALSE( fit->loss_fit_rad2.has_value() );
      EXPECT_FALSE( fit->nonlinear_nonincrease.has_value() );
    }
    EXPECT_FALSE(
        analysis.eligibility.validation_blocks_eligible.has_value() );
    EXPECT_FALSE( analysis.gates.evaluated );
  }

  TEST( GyroAlignmentTest, ValidationNearPiExcludesWholeCalendarBlock )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    auto                  input   = stationaryBiasFixture( bias, bias );
    const auto            near_pi = rotationVectorQuaternion(
        Eigen::Vector3d{ std::numbers::pi - 1.0e-9, 0.0, 0.0 } );
    for ( std::size_t i = 101U; i < input.endpoints.size(); ++i )
    {
      input.endpoints[ i ].q_WB = near_pi;
    }

    const auto  result   = analyzeGyroAlignment( input );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( analysis.full.status, GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( analysis.eligibility.validation_slots_total, 60U );
    EXPECT_EQ( analysis.eligibility.validation_blocks_eligible, 59U );
    EXPECT_EQ( analysis.eligibility.block_excluded_counts.near_pi, 1U );
    EXPECT_EQ( analysis.support.validation_blocks, 59U );
    EXPECT_EQ( analysis.support.sufficient, false );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_NE( std::find( result.verdict.reason_codes.begin(),
                          result.verdict.reason_codes.end(),
                          GyroAlignmentReason::kValidationSupport ),
               result.verdict.reason_codes.end() );
    EXPECT_EQ( analysis.blocks.size(), 59U );
    EXPECT_EQ( analysis.blocks.front().block_index, 1U );
  }

  TEST( GyroAlignmentTest, RejectedMiddleEndpointExcludesBothAdjacentRows )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    GyroAlignmentInput    control =
        stationaryBiasFixture( bias, bias, kSecondNs, 102U, 60U );
    GyroAlignmentInput rejected = control;
    rejected.endpoints.at( 110U ).status =
        GyroAlignmentEndpointStatus::kRejected;
    rejected.endpoints.at( 110U ).q_WB.reset();

    const auto  control_result  = analyzeGyroAlignment( control );
    const auto  rejected_result = analyzeGyroAlignment( rejected );
    const auto& a               = requireAnalysis( control_result );
    const auto& b               = requireAnalysis( rejected_result );
    EXPECT_EQ( b.eligibility.packet_total_nonfirst,
               a.eligibility.packet_total_nonfirst );
    EXPECT_EQ( b.eligibility.interval_excluded_counts.diag_rejected_no_pose,
               a.eligibility.interval_excluded_counts.diag_rejected_no_pose +
                   2U );
    EXPECT_EQ( b.eligibility.packet_structurally_eligible + 2U,
               a.eligibility.packet_structurally_eligible );
    EXPECT_EQ( b.eligibility.packet_fit_eligible + 2U,
               a.eligibility.packet_fit_eligible );
    EXPECT_EQ( b.eligibility.interval_excluded_counts.gap,
               a.eligibility.interval_excluded_counts.gap );
    EXPECT_EQ( b.eligibility.interval_excluded_counts.empty_nonfirst,
               a.eligibility.interval_excluded_counts.empty_nonfirst );
    EXPECT_EQ( b.eligibility.interval_excluded_counts.segment_boundary,
               a.eligibility.interval_excluded_counts.segment_boundary );
    EXPECT_EQ( b.eligibility.interval_excluded_counts.near_pi,
               a.eligibility.interval_excluded_counts.near_pi );
    EXPECT_EQ( b.eligibility.packet_total_nonfirst,
               b.eligibility.interval_excluded_counts.gap +
                   b.eligibility.interval_excluded_counts.empty_nonfirst +
                   b.eligibility.interval_excluded_counts
                       .diag_rejected_no_pose +
                   b.eligibility.interval_excluded_counts.segment_boundary +
                   b.eligibility.packet_structurally_eligible );
    EXPECT_EQ( b.eligibility.packet_structurally_eligible,
               b.eligibility.packet_fit_eligible +
                   b.eligibility.interval_excluded_counts.near_pi );
    EXPECT_EQ( control_result.verdict.status, GyroAlignmentStatus::kPass );
    EXPECT_EQ( rejected_result.verdict.status, GyroAlignmentStatus::kPass );
    for ( const GyroAlignmentBlock& block : b.blocks )
    {
      EXPECT_FALSE( block.t_start_ns <= 110 * kSecondNs &&
                    110 * kSecondNs <= block.t_end_ns );
    }
  }

  TEST( GyroAlignmentTest, IntervalFirstMatchPreventsDoubleCounting )
  {
    GyroAlignmentInput input;
    input.endpoints    = { endpoint( 0 ), endpoint( kSecondNs ),
                           endpoint( 2 * kSecondNs ),
                           endpoint( 3 * kSecondNs,
                                     Eigen::Quaterniond::Identity(),
                                     GyroAlignmentEndpointStatus::kRejected ),
                           endpoint( 4 * kSecondNs,
                                     Eigen::Quaterniond::Identity(),
                                     GyroAlignmentEndpointStatus::kOk, 1U ) };
    input.packets      = { firstZero( 0 ),
                           gapPacket( 1U, 0, kSecondNs ),
                           GyroAlignmentPacket{
                               .packet_index = 2U,
                               .t_prev_ns    = kSecondNs,
                               .t_cur_ns     = 2 * kSecondNs,
                               .segment_id   = 0U,
                               .imu_gap      = false,
                               .status       = GyroAlignmentPacketStatus::kEmptyNonfirst,
                               .samples      = {},
                               .sum_dt_ns    = 0,
                               .interval_ns  = kSecondNs,
                      },
                           validPacket( 3U, 2 * kSecondNs, 3 * kSecondNs,
                                        Eigen::Vector3d::Zero() ),
                           validPacket( 4U, 3 * kSecondNs, 4 * kSecondNs,
                                        Eigen::Vector3d::Zero(), 1U ) };
    const auto  result = analyzeGyroAlignment( input );
    const auto& e      = requireAnalysis( result ).eligibility;
    EXPECT_EQ( e.interval_excluded_counts.gap, 1U );
    EXPECT_EQ( e.interval_excluded_counts.empty_nonfirst, 1U );
    EXPECT_EQ( e.interval_excluded_counts.diag_rejected_no_pose, 2U );
    EXPECT_EQ( e.interval_excluded_counts.segment_boundary, 0U );
    EXPECT_EQ( e.packet_total_nonfirst, 4U );
  }

  TEST( GyroAlignmentTest,
        AllContractValidLocalExclusionsProduceEmptyFitUniverse )
  {
    GyroAlignmentInput input;
    input.endpoints = {
        endpoint( 0 ),
        endpoint( kSecondNs ),
        endpoint( 2 * kSecondNs ),
        endpoint( 3 * kSecondNs, Eigen::Quaterniond::Identity(),
                  GyroAlignmentEndpointStatus::kRejected ),
        endpoint( 4 * kSecondNs ),
        endpoint( 5 * kSecondNs, Eigen::Quaterniond::Identity(),
                  GyroAlignmentEndpointStatus::kOk, 1U ),
        endpoint( 6 * kSecondNs,
                  rotationVectorQuaternion( Eigen::Vector3d{
                      std::numbers::pi - 5.0e-6, 0.0, 0.0 } ),
                  GyroAlignmentEndpointStatus::kOk, 1U ),
    };
    input.packets = {
        firstZero( 0 ),
        gapPacket( 1U, 0, kSecondNs ),
        GyroAlignmentPacket{
            .packet_index = 2U,
            .t_prev_ns    = kSecondNs,
            .t_cur_ns     = 2 * kSecondNs,
            .segment_id   = 0U,
            .imu_gap      = false,
            .status       = GyroAlignmentPacketStatus::kEmptyNonfirst,
            .samples      = {},
            .sum_dt_ns    = 0,
            .interval_ns  = kSecondNs,
        },
        validPacket( 3U, 2 * kSecondNs, 3 * kSecondNs,
                     Eigen::Vector3d::Zero() ),
        validPacket( 4U, 3 * kSecondNs, 4 * kSecondNs,
                     Eigen::Vector3d::Zero() ),
        validPacket( 5U, 4 * kSecondNs, 5 * kSecondNs,
                     Eigen::Vector3d::Zero(), 1U ),
        validPacket( 6U, 5 * kSecondNs, 6 * kSecondNs,
                     Eigen::Vector3d::Zero(), 1U ),
    };

    const auto  result   = analyzeGyroAlignment( input );
    const auto& analysis = requireAnalysis( result );
    const auto& excluded = analysis.eligibility.interval_excluded_counts;
    EXPECT_EQ( excluded.gap, 1U );
    EXPECT_EQ( excluded.empty_nonfirst, 1U );
    EXPECT_EQ( excluded.diag_rejected_no_pose, 2U );
    EXPECT_EQ( excluded.segment_boundary, 1U );
    EXPECT_EQ( excluded.near_pi, 1U );
    EXPECT_EQ( analysis.eligibility.packet_total_nonfirst, 6U );
    EXPECT_EQ( analysis.eligibility.packet_structurally_eligible, 1U );
    EXPECT_EQ( analysis.eligibility.packet_fit_eligible, 0U );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( result.verdict.reason_codes,
               std::vector{ GyroAlignmentReason::kFitSupport } );
    for ( const GyroAlignmentFit* fit :
          { &analysis.full, &analysis.early, &analysis.late } )
    {
      expectFitNullability( *fit,
                            GyroAlignmentFitStatus::kSupportInsufficient );
    }
  }

  TEST( GyroAlignmentTest, MidpointStraddleIsFullOnlyAndNeverSplit )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    GyroAlignmentInput    input;
    input.endpoints = { endpoint( 0 ), endpoint( 49 * kSecondNs ),
                        endpoint( 51 * kSecondNs ),
                        endpoint( 100 * kSecondNs ) };
    input.packets   = {
        firstZero( 0 ),
        validPacket( 1U, 0, 49 * kSecondNs, bias ),
        validPacket( 2U, 49 * kSecondNs, 51 * kSecondNs, bias ),
        validPacket( 3U, 51 * kSecondNs, 100 * kSecondNs, bias ),
    };
    const auto  result   = analyzeGyroAlignment( input );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( analysis.full.packet_count, 3U );
    EXPECT_EQ( analysis.early.packet_count, 1U );
    EXPECT_EQ( analysis.late.packet_count, 1U );
    EXPECT_EQ( analysis.full.eligible_duration_ns, 100 * kSecondNs );
    EXPECT_EQ( analysis.early.eligible_duration_ns, 49 * kSecondNs );
    EXPECT_EQ( analysis.late.eligible_duration_ns, 49 * kSecondNs );
    EXPECT_EQ( analysis.full.status, GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( analysis.early.status, GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( analysis.late.status, GyroAlignmentFitStatus::kSolved );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_NE( std::find( result.verdict.reason_codes.begin(),
                          result.verdict.reason_codes.end(),
                          GyroAlignmentReason::kValidationSupport ),
               result.verdict.reason_codes.end() );
  }

  TEST( GyroAlignmentTest, CalendarGapKeepsSlotAndMarksItIncomplete )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    auto                  input = stationaryBiasFixture( bias, bias );
    auto&                 gap   = input.packets.at( 101U );
    gap.status                  = GyroAlignmentPacketStatus::kGap;
    gap.imu_gap                 = true;

    const auto  result   = analyzeGyroAlignment( input );
    const auto& analysis = requireAnalysis( result );
    EXPECT_EQ( analysis.eligibility.validation_slots_total, 60U );
    EXPECT_EQ( analysis.eligibility.interval_excluded_counts.gap, 1U );
    EXPECT_EQ( analysis.eligibility.block_excluded_counts
                   .incomplete_calendar_block,
               1U );
    EXPECT_EQ( analysis.eligibility.validation_blocks_eligible, 59U );
    EXPECT_EQ( analysis.support.validation_blocks, 59U );
    EXPECT_EQ( analysis.support.sufficient, false );
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( analysis.blocks.front().block_index, 1U );
  }

  TEST( GyroAlignmentTest, HardInputErrorsNeverExposePartialAnalysis )
  {
    GyroAlignmentInput input;
    input.endpoints = { endpoint( 0 ), endpoint( kSecondNs ) };
    input.packets   = { firstZero( 0 ),
                        validPacket( 1U, 0, kSecondNs,
                                     Eigen::Vector3d::Zero() ) };

    auto invalid_quaternion = input;
    invalid_quaternion.endpoints.at( 1U ).q_WB =
        Eigen::Quaterniond{ 2.0, 0.0, 0.0, 0.0 };
    expectHardError( analyzeGyroAlignment( invalid_quaternion ),
                     GyroAlignmentErrorCode::kInvalidQuaternion );

    auto nonfinite = input;
    nonfinite.packets.at( 1U ).samples.at( 0U ).gyro_radps[ 2 ] =
        std::numeric_limits<double>::infinity();
    expectHardError( analyzeGyroAlignment( nonfinite ),
                     GyroAlignmentErrorCode::kNonfiniteInput );

    auto failed_diag = input;
    failed_diag.endpoints.at( 1U ).status =
        GyroAlignmentEndpointStatus::kFailed;
    expectHardError( analyzeGyroAlignment( failed_diag ),
                     GyroAlignmentErrorCode::kDiagFailed );

    auto contradiction = input;
    contradiction.endpoints.at( 1U ).status =
        GyroAlignmentEndpointStatus::kRejected;
    expectHardError( analyzeGyroAlignment( contradiction ),
                     GyroAlignmentErrorCode::kDiagPoseContradiction );
  }

  TEST( GyroAlignmentTest, TimestampAndPacketShapeFailuresAreTyped )
  {
    GyroAlignmentInput input;
    input.endpoints = { endpoint( 0 ), endpoint( kSecondNs ) };
    input.packets   = { firstZero( 0 ),
                        validPacket( 1U, 0, kSecondNs,
                                     Eigen::Vector3d::Zero() ) };

    auto order                          = input;
    order.packets.at( 1U ).packet_index = 2U;
    expectHardError( analyzeGyroAlignment( order ),
                     GyroAlignmentErrorCode::kIndexOrder );

    auto count = input;
    count.packets.at( 1U ).samples.pop_back();
    expectHardError( analyzeGyroAlignment( count ),
                     GyroAlignmentErrorCode::kCountMismatch );

    auto duration = input;
    duration.packets.at( 1U ).sum_dt_ns -= 1;
    expectHardError( analyzeGyroAlignment( duration ),
                     GyroAlignmentErrorCode::kDurationMismatch );

    auto endpoint_mismatch = input;
    endpoint_mismatch.packets.at( 1U ).samples.back().timestamp =
        phad::common::Timestamp{ kSecondNs - 1 };
    expectHardError( analyzeGyroAlignment( endpoint_mismatch ),
                     GyroAlignmentErrorCode::kEndpointMismatch );

    GyroAlignmentInput overflow;
    const auto         near_max = std::numeric_limits<std::int64_t>::max() - 10;
    overflow.endpoints          = { endpoint( near_max ) };
    overflow.packets            = { firstZero( near_max ) };
    expectHardError( analyzeGyroAlignment( overflow ),
                     GyroAlignmentErrorCode::kTimestampOverflow );
  }

  TEST( GyroAlignmentTest, SignFrameAxisAndUnitMutantsDisagreeWithOracle )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    const auto            result =
        analyzeGyroAlignment( stationaryBiasFixture( bias, bias ) );
    const auto& fit = requireAnalysis( result ).full;
    ASSERT_TRUE( fit.bias_radps.has_value() );
    EXPECT_TRUE( fit.bias_radps->isApprox( bias, 2.0e-12 ) );
    EXPECT_FALSE( fit.bias_radps->isApprox( -bias, 1.0e-6 ) );
    EXPECT_FALSE( fit.bias_radps->isApprox(
        Eigen::Vector3d{ bias.z(), bias.y(), bias.x() }, 1.0e-6 ) );
    EXPECT_FALSE( fit.bias_radps->isApprox(
        bias * ( 180.0 / std::numbers::pi ), 1.0e-6 ) );
    EXPECT_FALSE( fit.bias_radps->isApprox( 0.5 * bias, 1.0e-6 ) );
    EXPECT_FALSE( fit.bias_radps->isApprox( 2.0 * bias, 1.0e-6 ) );

    const Eigen::Matrix3d r_i =
        rotationVectorQuaternion( Eigen::Vector3d{ 0.2, 0.1, -0.3 } )
            .toRotationMatrix();
    const Eigen::Matrix3d r_j =
        rotationVectorQuaternion( Eigen::Vector3d{ -0.1, 0.3, 0.2 } )
            .toRotationMatrix();
    const Eigen::Matrix3d expected         = r_i.transpose() * r_j;
    const Eigen::Matrix3d transpose_mutant = r_j.transpose() * r_i;
    EXPECT_FALSE( principalLogOracle( expected ).isApprox( principalLogOracle( transpose_mutant ), 1.0e-12 ) );
  }

  TEST( GyroAlignmentTest,
        NoncommutingBodyFixtureDetectsComposeFrameAndTransposeMutants )
  {
    const Eigen::Vector3d bias{ 0.002, -0.003, 0.001 };
    const auto            result   = analyzeGyroAlignment( noncommutingFixture() );
    const auto&           analysis = requireAnalysis( result );
    ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kPass );
    ASSERT_TRUE( analysis.full.bias_radps.has_value() );
    EXPECT_TRUE( analysis.full.bias_radps->isApprox( bias, 2.0e-7 ) );
    ASSERT_EQ( analysis.blocks.size(), 60U );
    EXPECT_LE( analysis.blocks.front().r_fit_rad.norm(), 2.0e-8 );

    const Eigen::Matrix3d r_a = rotationVectorQuaternion(
                                    Eigen::Vector3d{ 0.01, 0.0, 0.0 } )
                                    .toRotationMatrix();
    const Eigen::Matrix3d r_b = rotationVectorQuaternion(
                                    Eigen::Vector3d{ 0.0, -0.015, 0.005 } )
                                    .toRotationMatrix();
    const Eigen::Matrix3d expected               = r_a * r_b;
    const Eigen::Matrix3d reverse_compose_mutant = r_b * r_a;
    const Eigen::Matrix3d transpose_mutant       = expected.transpose();
    const Eigen::Matrix3d T_B_left =
        rotationVectorQuaternion(
            Eigen::Vector3d{ 0.0, 0.0, std::numbers::pi / 2.0 } )
            .toRotationMatrix();
    const Eigen::Matrix3d camera_frame_mutant =
        T_B_left.transpose() * expected * T_B_left;
    EXPECT_GT( ( expected - reverse_compose_mutant ).norm(), 1.0e-6 );
    EXPECT_GT( ( expected - transpose_mutant ).norm(), 1.0e-3 );
    EXPECT_GT( ( expected - camera_frame_mutant ).norm(), 1.0e-3 );
  }

  TEST( GyroAlignmentTest, RunnerRoundTripHandlesFirstZeroOnlyWithoutRealData )
  {
    constexpr std::string_view kTum = "0.000000000 0 0 0 0 0 0 1\n";
    constexpr std::string_view kDiag =
        "timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,"
        "window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,"
        "num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,"
        "pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,"
        "is_keyframe,num_disparity\n"
        "0,ok,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0\n";
    constexpr std::string_view kPackets =
        "packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,"
        "sum_dt_ns,interval_ns,status\n"
        "0,0,0,0,0,0,0,0,first_zero\n";
    constexpr std::string_view kSamples =
        "packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,"
        "gyr_z_radps\n";
    constexpr std::array<std::string_view, 4> kNames{
        "est.tum", "diag.csv", "gyro_packets.csv", "gyro_samples.csv" };
    constexpr std::array<std::string_view, 4> kBytes{ kTum, kDiag, kPackets,
                                                      kSamples };
    constexpr std::array<std::string_view, 4> kHashes{
        "12d730a2c28778c6239e858e8faa569c06e0f42d399e39c1c5646c874ca6fc3f",
        "083fac02eeee21b787e02518e43e8b409859317443cbef0abb964dd85a7921ae",
        "749d6e057abeedf4ca4369729b54e9bbd5a4364ad0d2aec6eb54a8c1d6e70133",
        "a89b57f602488d7d6b048d7ac2fe9c22a5278d2cb7bfbeaa8da5fde63423ae89",
    };

    ScopedDirectory root{ "analyzer_runner_roundtrip" };
    const auto      q1_dir  = root.path / "q1";
    const auto      out_dir = root.path / "out";
    std::filesystem::create_directory( q1_dir );
    for ( std::size_t i = 0; i < kNames.size(); ++i )
    {
      std::ofstream out( q1_dir / kNames[ i ], std::ios::binary );
      out << kBytes[ i ];
    }

    GyroAlignmentProtocolDescriptor descriptor{
        .protocol_id       = "PHAD-M4-Q3-GYRO-ALIGN-SYNTHETIC-TEST-V1",
        .inputs            = { GyroAlignmentInputIdentity{ std::string{ kNames[ 0 ] },
                                                std::string{ kHashes[ 0 ] } },
                               GyroAlignmentInputIdentity{ std::string{ kNames[ 1 ] },
                                                std::string{ kHashes[ 1 ] } },
                               GyroAlignmentInputIdentity{ std::string{ kNames[ 2 ] },
                                                std::string{ kHashes[ 2 ] } },
                               GyroAlignmentInputIdentity{ std::string{ kNames[ 3 ] },
                                                std::string{ kHashes[ 3 ] } } },
        .protocol_identity = {
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "cccccccccccccccccccccccccccccccccccccccc",
            "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd" },
        .analyzer_identity = { "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee", "ffffffffffffffffffffffffffffffffffffffff", "Synthetic", "tester", "1.0" },
        .provenance        = { "synthetic-q1-run", "1111111111111111111111111111111111111111", "2222222222222222222222222222222222222222", "3333333333333333333333333333333333333333333333333333333333333333", "4444444444444444444444444444444444444444444444444444444444444444", "deadbeef", "5555555555555555555555555555555555555555555555555555555555555555", "6666666666666666666666666666666666666666", "7777777777777777777777777777777777777777" },
    };

    const auto result =
        runGyroAlignment( std::move( descriptor ), q1_dir, out_dir );
    ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    EXPECT_EQ( result.verdict.exit_code, 3 );
    EXPECT_TRUE( result.analysis.has_value() );
    EXPECT_TRUE( std::filesystem::is_regular_file(
        out_dir / "gyro_alignment.json" ) );
    EXPECT_TRUE( std::filesystem::is_regular_file(
        out_dir / "gyro_alignment_blocks.csv" ) );
    EXPECT_TRUE( std::filesystem::is_regular_file(
        out_dir / "gyro_alignment.manifest.json" ) );
  }

}  // namespace
