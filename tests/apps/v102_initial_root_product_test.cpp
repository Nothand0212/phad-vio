#include <gtest/gtest.h>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "apps/offline_vo_session.hpp"
#include "phad/eval/ate.hpp"
#include "phad/eval/rpe.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"

namespace
{

  constexpr std::string_view kExpectedManifestDigest =
      "a80c02d6c9bd9e5cd76412f30b6859fd8acdabf0885de1cba494725a183ced8f";
  constexpr std::uint64_t kProcessedPackets = 118U;

  class Sha256 final
  {
  public:
    Sha256()
        : m_context( EVP_MD_CTX_new(), &EVP_MD_CTX_free )
    {
      if ( !m_context ||
           EVP_DigestInit_ex( m_context.get(), EVP_sha256(), nullptr ) != 1 )
      {
        throw std::runtime_error( "failed to initialize SHA-256" );
      }
    }

    Sha256( const Sha256& )            = delete;
    Sha256& operator=( const Sha256& ) = delete;
    Sha256( Sha256&& )                 = delete;
    Sha256& operator=( Sha256&& )      = delete;

    void update( const void* data, const std::size_t size )
    {
      if ( m_finished || EVP_DigestUpdate( m_context.get(), data, size ) != 1 )
      {
        throw std::runtime_error( "failed to update SHA-256" );
      }
    }

    void update( const std::string_view bytes )
    {
      update( bytes.data(), bytes.size() );
    }

    [[nodiscard]] std::string finish()
    {
      if ( m_finished )
      {
        throw std::runtime_error( "SHA-256 was already finalized" );
      }
      std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
      unsigned int                               size = 0U;
      if ( EVP_DigestFinal_ex( m_context.get(), digest.data(), &size ) != 1 ||
           size != 32U )
      {
        throw std::runtime_error( "failed to finalize SHA-256" );
      }
      m_finished = true;

      std::ostringstream stream;
      stream << std::hex << std::setfill( '0' );
      for ( std::size_t index = 0U; index < size; ++index )
      {
        stream << std::setw( 2 )
               << static_cast<unsigned int>( digest[ index ] );
      }
      return stream.str();
    }

  private:
    using Context =
        std::unique_ptr<EVP_MD_CTX, decltype( &EVP_MD_CTX_free )>;

    Context m_context;
    bool    m_finished = false;
  };

  [[nodiscard]] std::string sha256File(
      const std::filesystem::path& path )
  {
    std::ifstream input( path, std::ios::binary );
    if ( !input )
    {
      throw std::runtime_error( "failed to open manifest input: " +
                                path.string() );
    }

    Sha256                     digest;
    std::array<char, 1U << 20> buffer{};
    while ( input )
    {
      input.read( buffer.data(),
                  static_cast<std::streamsize>( buffer.size() ) );
      const std::streamsize count = input.gcount();
      if ( count > 0 )
      {
        digest.update( buffer.data(), static_cast<std::size_t>( count ) );
      }
    }
    if ( !input.eof() )
    {
      throw std::runtime_error( "failed to read manifest input: " +
                                path.string() );
    }
    return digest.finish();
  }

  struct ManifestEntry
  {
    std::filesystem::path m_absolute;
    std::string           m_relative;
  };

  [[nodiscard]] std::string canonicalManifestDigest(
      const std::filesystem::path& sequence_root )
  {
    std::vector<ManifestEntry> entries;
    for ( const std::filesystem::directory_entry& entry :
          std::filesystem::recursive_directory_iterator( sequence_root ) )
    {
      if ( !std::filesystem::is_regular_file( entry.symlink_status() ) )
      {
        continue;
      }
      const std::string relative =
          std::filesystem::relative( entry.path(), sequence_root )
              .generic_string();
      if ( relative.empty() || relative.find_first_of( "\\\n" ) !=
                                   std::string::npos )
      {
        throw std::runtime_error(
            "manifest path requires sha256sum escaping" );
      }
      entries.push_back(
          ManifestEntry{ .m_absolute = entry.path(),
                         .m_relative = relative } );
    }
    std::sort( entries.begin(), entries.end(),
               []( const ManifestEntry& lhs, const ManifestEntry& rhs ) {
                 return lhs.m_relative < rhs.m_relative;
               } );

    Sha256 manifest;
    for ( const ManifestEntry& entry : entries )
    {
      const std::string line = sha256File( entry.m_absolute ) + "  ./" +
                               entry.m_relative + '\n';
      manifest.update( line );
    }
    return manifest.finish();
  }

  [[nodiscard]] double completionRate(
      const phad::apps::OfflineVoSessionResult& result )
  {
    if ( result.counts.image_frames == 0U )
    {
      return 0.0;
    }
    return static_cast<double>( result.counts.ok ) /
           static_cast<double>( result.counts.image_frames );
  }

  [[nodiscard]] double coverageRate(
      const phad::apps::OfflineVoSessionResult& result )
  {
    if ( !result.trajectory.has_value() || result.trajectory->size() == 0U )
    {
      return 0.0;
    }
    const std::int64_t image_span_ns =
        result.last_image_ts.nanoseconds() -
        result.first_image_ts.nanoseconds();
    if ( image_span_ns <= 0 )
    {
      return 0.0;
    }
    const auto&        poses = result.trajectory->poses();
    const std::int64_t pose_span_ns =
        poses.back().timestamp.nanoseconds() -
        poses.front().timestamp.nanoseconds();
    return static_cast<double>( pose_span_ns ) /
           static_cast<double>( image_span_ns );
  }

  [[nodiscard]] std::string metricValue( const double value )
  {
    if ( !std::isfinite( value ) )
    {
      return "not_evaluable:nonfinite";
    }
    std::ostringstream stream;
    stream << std::setprecision( 17 ) << value;
    return stream.str();
  }

  void recordEvaluation(
      const std::filesystem::path&              sequence_root,
      const phad::apps::OfflineVoSessionResult& result )
  {
    std::string ate_record = "not_evaluable:no_trajectory";
    std::string rpe_record = "not_evaluable:no_trajectory";
    auto        groundtruth =
        phad::io::dataset::euroc::openGroundtruth( sequence_root );
    if ( !groundtruth )
    {
      ate_record = "not_evaluable:" + groundtruth.error().describe();
      rpe_record = ate_record;
    }
    else if ( result.trajectory.has_value() )
    {
      phad::eval::AssociationOptions association;
      association.max_dt_ns      = 2'500'000;
      association.min_match_rate = 0.5;

      phad::eval::AteOptions ate_options;
      ate_options.association = association;
      auto ate                = phad::eval::computeAte(
          *result.trajectory, groundtruth.value(), ate_options );
      ate_record = ate ? metricValue( ate.value().trans_m.rmse )
                       : "not_evaluable:" + ate.error().describe();

      phad::eval::RpeOptions rpe_options;
      rpe_options.association = association;
      rpe_options.delta_ns    = 1'000'000'000;
      auto rpe                = phad::eval::computeRpe(
          *result.trajectory, groundtruth.value(), rpe_options );
      rpe_record = rpe ? metricValue( rpe.value().trans_m.rmse )
                       : "not_evaluable:" + rpe.error().describe();
    }

    ::testing::Test::RecordProperty( "ate_translation_rmse_m", ate_record );
    ::testing::Test::RecordProperty( "rpe_1s_translation_rmse_m",
                                     rpe_record );
  }

  TEST( V102InitialRootProductTest,
        ProcessesFixedPrefixThroughTypedMovingLifecycle )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_V102_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_V102_PATH is not set";
    }
    const std::filesystem::path sequence_root{ configured_path };
    ASSERT_TRUE( std::filesystem::is_directory( sequence_root ) );

    std::string manifest_digest;
    ASSERT_NO_THROW(
        manifest_digest = canonicalManifestDigest( sequence_root ) );
    ASSERT_EQ( manifest_digest, kExpectedManifestDigest );

    auto opened = phad::io::dataset::euroc::open( sequence_root );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();

    phad::apps::OfflineVoSessionOptions options;
    options.max_frames                          = kProcessedPackets;
    options.estimator.m_enable_moving_bootstrap = true;
    const phad::apps::OfflineVoSessionResult result =
        phad::apps::runOfflineVoSession(
            std::move( opened ).value(), options );

    std::ostringstream summary;
    summary << "error="
            << ( result.error.has_value() ? result.error->detail : "none" )
            << "; image_frames=" << result.counts.image_frames
            << "; diag_size=" << result.diag.size()
            << "; emitted_stereo=" << result.sync.emitted_stereo
            << "; failed=" << result.counts.failed
            << "; rejected=" << result.counts.rejected
            << "; segments=" << result.counts.segments
            << "; reanchors=" << result.counts.reanchors
            << "; completion=" << completionRate( result )
            << "; coverage=" << coverageRate( result );
    ::testing::Test::RecordProperty( "fixed_prefix_summary", summary.str() );
    ::testing::Test::RecordProperty( "manifest_digest", manifest_digest );
    ::testing::Test::RecordProperty(
        "completion_rate", metricValue( completionRate( result ) ) );
    ::testing::Test::RecordProperty(
        "coverage_rate", metricValue( coverageRate( result ) ) );
    ::testing::Test::RecordProperty(
        "emitted_stereo", std::to_string( result.sync.emitted_stereo ) );
    recordEvaluation( sequence_root, result );
    SCOPED_TRACE( summary.str() );

    ASSERT_FALSE( result.error.has_value() );
    ASSERT_EQ( result.counts.image_frames, kProcessedPackets );
    ASSERT_EQ( result.diag.size(), kProcessedPackets );
    EXPECT_EQ( result.sync.dropped_left, 0U );
    EXPECT_EQ( result.sync.dropped_right, 0U );
    EXPECT_EQ( result.sync.dropped_left_overflow, 0U );
    EXPECT_EQ( result.sync.dropped_right_overflow, 0U );
    EXPECT_EQ( result.counts.failed, 0U );
    EXPECT_EQ( result.counts.rejected, 0U );
    EXPECT_EQ( result.counts.segments, 1U );
    EXPECT_EQ( result.counts.reanchors, 0U );
    ASSERT_TRUE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.ok, result.trajectory->size() );

    const auto first_ok = std::find_if(
        result.diag.begin(), result.diag.end(),
        []( const phad::apps::VoDiagRow& row ) {
          return row.status == "ok";
        } );
    ASSERT_NE( first_ok, result.diag.end() );
    const std::size_t first_ok_index = static_cast<std::size_t>(
        std::distance( result.diag.begin(), first_ok ) );
    EXPECT_GE( first_ok_index, 5U );
    EXPECT_GE( first_ok->timestamp_ns - result.diag.front().timestamp_ns,
               250'000'000 );

    for ( std::size_t index = 0U; index < first_ok_index; ++index )
    {
      const phad::apps::VoDiagRow& row = result.diag[ index ];
      EXPECT_EQ( row.status, "initializing" ) << "diag_index=" << index;
      const auto& initialization = row.initialization;
      const bool  pending =
          initialization.m_outcome ==
          phad::estimator::InitializationOutcome::kPending;
      const bool rejected =
          initialization.m_outcome ==
          phad::estimator::InitializationOutcome::kRecoverableRejected;
      EXPECT_TRUE( pending || rejected ) << "diag_index=" << index;
      EXPECT_NE( initialization.m_reason,
                 phad::estimator::InitializationReason::kNone )
          << "diag_index=" << index;
      EXPECT_EQ(
          initialization.m_transaction,
          phad::estimator::InitializationTransactionResult::
              kEvidenceCommitted )
          << "diag_index=" << index;
      EXPECT_EQ( initialization.m_attempt_id.has_value(), rejected )
          << "diag_index=" << index;
    }

    const auto& root = first_ok->initialization;
    EXPECT_EQ( root.m_path,
               phad::estimator::InitializationPath::kMoving );
    EXPECT_EQ( root.m_phase,
               phad::estimator::InitializationPhase::kCommit );
    EXPECT_EQ( root.m_outcome,
               phad::estimator::InitializationOutcome::kCommitted );
    EXPECT_EQ( root.m_reason,
               phad::estimator::InitializationReason::kNone );
    EXPECT_TRUE( root.m_attempt_id.has_value() );
    EXPECT_EQ( root.m_connected_poses.m_required, 6U );
    EXPECT_GE( root.m_connected_poses.m_actual,
               root.m_connected_poses.m_required );
    EXPECT_EQ( root.m_duration.m_required_ns, 250'000'000 );
    EXPECT_GE( root.m_duration.m_actual_ns,
               root.m_duration.m_required_ns );
    EXPECT_EQ( root.m_connected_landmarks.m_required, 10U );
    EXPECT_GE( root.m_connected_landmarks.m_actual,
               root.m_connected_landmarks.m_required );
    ASSERT_TRUE( root.m_visual_support.has_value() );
    EXPECT_EQ( root.m_visual_support->m_required, 10U );
    EXPECT_GE( root.m_visual_support->m_actual,
               root.m_visual_support->m_required );
    EXPECT_EQ( root.m_current_graph,
               phad::estimator::InitializationGraphResult::kValidated );
    EXPECT_EQ(
        root.m_transaction,
        phad::estimator::InitializationTransactionResult::kRootCommitted );

    for ( std::size_t index = first_ok_index; index < result.diag.size();
          ++index )
    {
      EXPECT_EQ( result.diag[ index ].status, "ok" )
          << "diag_index=" << index;
    }

    const auto& poses      = result.trajectory->poses();
    std::size_t pose_index = 0U;
    for ( const phad::apps::VoDiagRow& row : result.diag )
    {
      if ( row.status != "ok" )
      {
        continue;
      }
      ASSERT_LT( pose_index, poses.size() );
      EXPECT_EQ( poses[ pose_index ].timestamp.nanoseconds(),
                 row.timestamp_ns );
      EXPECT_TRUE( poses[ pose_index ].T_W_B.matrix().allFinite() );
      ++pose_index;
    }
    EXPECT_EQ( pose_index, poses.size() );
    ASSERT_FALSE( poses.empty() );
    EXPECT_EQ( poses.front().timestamp.nanoseconds(),
               first_ok->timestamp_ns );
    EXPECT_EQ( poses.back().timestamp.nanoseconds(),
               result.diag.back().timestamp_ns );
  }

}  // namespace
