#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

  constexpr std::size_t kStateCount = 14U;
  constexpr std::size_t kAxisCount  = 3U;
  constexpr double      kRecoveryDt = 0.1;
  constexpr double      kRecoveryNd = 1e-4;
  constexpr double      kRecoveryRw = 1e-3;
  constexpr double      kPriorSigma = 0.1;

  const Eigen::Vector3d kBiasBase{ 0.012, -0.018, 0.025 };
  const Eigen::Vector3d kBiasStep{ 0.0001, -0.00015, 0.0002 };

  const std::array<Eigen::Vector3d, kStateCount> kExpectedBiases{
      Eigen::Vector3d{ 0.012061728029528, -0.018092592044293,
                       0.025123449878755 },
      Eigen::Vector3d{ 0.012123576676337, -0.018185365014506,
                       0.025247150992009 },
      Eigen::Vector3d{ 0.012209001999483, -0.018313502999225,
                       0.025418003097272 },
      Eigen::Vector3d{ 0.012303429322112, -0.018455143983168,
                       0.025606858299808 },
      Eigen::Vector3d{ 0.012401285966853, -0.018601928950280,
                       0.025802571802151 },
      Eigen::Vector3d{ 0.012500428578447, -0.018750642867671,
                       0.026000857106645 },
      Eigen::Vector3d{ 0.012599999768489, -0.018899999652733,
                       0.026199999517784 },
      Eigen::Vector3d{ 0.012699570727019, -0.019049356090529,
                       0.026399141446707 },
      Eigen::Vector3d{ 0.012798712412569, -0.019198068618854,
                       0.026597424822338 },
      Eigen::Vector3d{ 0.012896566510688, -0.019344849766032,
                       0.026793133020306 },
      Eigen::Vector3d{ 0.012990987119495, -0.019486480679243,
                       0.026981974238579 },
      Eigen::Vector3d{ 0.013076394847798, -0.019614592271697,
                       0.027152789695432 },
      Eigen::Vector3d{ 0.013138197423899, -0.019707296135849,
                       0.027276394847716 },
      Eigen::Vector3d{ 0.013138197423899, -0.019707296135849,
                       0.027276394847716 },
  };

  struct LinearSystem
  {
    Eigen::MatrixXd m_matrix;
    Eigen::VectorXd m_rhs;
  };

  struct MutantLinearSystem
  {
    Eigen::MatrixXd m_matrix;
    Eigen::MatrixXd m_rhs;
    Eigen::Index    m_bias_offset;
  };

  [[nodiscard]] Eigen::Index stateColumn( std::size_t state,
                                          std::size_t axis )
  {
    return static_cast<Eigen::Index>( state * kAxisCount + axis );
  }

  [[nodiscard]] Eigen::Vector3d truthBias( std::size_t state )
  {
    return kBiasBase + static_cast<double>( state ) * kBiasStep;
  }

  [[nodiscard]] Eigen::Matrix3d skew( const Eigen::Vector3d& vector )
  {
    Eigen::Matrix3d result;
    result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return result;
  }

  [[nodiscard]] Eigen::Matrix3d rodrigues( const Eigen::Vector3d& vector )
  {
    const double angle = vector.norm();
    EXPECT_GT( angle, 0.0 );
    const Eigen::Matrix3d generator = skew( vector ) / angle;
    return Eigen::Matrix3d::Identity() + std::sin( angle ) * generator +
           ( 1.0 - std::cos( angle ) ) * generator * generator;
  }

  [[nodiscard]] LinearSystem makeRecoverySystem()
  {
    constexpr std::size_t kLinkCount = kStateCount - 1U;
    constexpr std::size_t kRowCount =
        kAxisCount + 2U * kLinkCount * kAxisCount;
    constexpr std::size_t kColumnCount = kStateCount * kAxisCount;
    LinearSystem          system{
        Eigen::MatrixXd::Zero( static_cast<Eigen::Index>( kRowCount ),
                                        static_cast<Eigen::Index>( kColumnCount ) ),
        Eigen::VectorXd::Zero( static_cast<Eigen::Index>( kRowCount ) ),
    };

    const double rotation_sigma = kRecoveryNd * std::sqrt( kRecoveryDt );
    const double rotation_scale = kRecoveryDt / rotation_sigma;
    const double rw_sigma       = kRecoveryRw * std::sqrt( kRecoveryDt );
    const double rw_scale       = 1.0 / rw_sigma;
    const double prior_scale    = 1.0 / kPriorSigma;

    Eigen::Index row = 0;
    for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
    {
      system.m_matrix( row, stateColumn( 0U, axis ) ) = prior_scale;
      ++row;
    }
    for ( std::size_t link = 0; link < kLinkCount; ++link )
    {
      const Eigen::Vector3d observed_bias = truthBias( link );
      for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
      {
        system.m_matrix( row, stateColumn( link, axis ) ) = rotation_scale;
        system.m_rhs( row ) =
            rotation_scale * observed_bias[ static_cast<Eigen::Index>( axis ) ];
        ++row;
      }
    }
    for ( std::size_t link = 0; link < kLinkCount; ++link )
    {
      for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
      {
        system.m_matrix( row, stateColumn( link, axis ) )      = -rw_scale;
        system.m_matrix( row, stateColumn( link + 1U, axis ) ) = rw_scale;
        ++row;
      }
    }
    EXPECT_EQ( row, system.m_matrix.rows() );
    return system;
  }

  [[nodiscard]] Eigen::Matrix<double, 3, 3> projectionJacobian(
      const Eigen::Vector3d& point )
  {
    constexpr double kFx       = 400.0;
    constexpr double kFy       = 400.0;
    constexpr double kBaseline = 0.12;
    const double     x         = point.x();
    const double     y         = point.y();
    const double     z         = point.z();
    Eigen::Matrix3d  jacobian;
    jacobian << kFx / z, 0.0, -kFx * x / ( z * z ), kFx / z, 0.0,
        -kFx * ( x - kBaseline ) / ( z * z ), 0.0, kFy / z,
        -kFy * y / ( z * z );
    return jacobian;
  }

  [[nodiscard]] MutantLinearSystem makeNonIdentityJointSystem()
  {
    constexpr std::size_t kLandmarkCount = 20U;
    constexpr std::size_t kLinkCount     = kStateCount - 1U;
    constexpr std::size_t kPoseColumns   = 6U * kStateCount;
    constexpr std::size_t kPointColumns  = 3U * kLandmarkCount;
    constexpr std::size_t kBiasColumns   = 3U * kStateCount;
    constexpr std::size_t kColumnCount =
        kPoseColumns + kPointColumns + kBiasColumns;
    constexpr std::size_t kRowCount =
        3U * kStateCount * kLandmarkCount + 6U +
        6U * kLinkCount + 3U;
    constexpr double kStereoSigma = 0.003;
    constexpr double kPosePrior   = 1e-4;

    const std::array<Eigen::Vector3d, kLandmarkCount> landmarks{
        Eigen::Vector3d{ 0.40, 0.10, 5.0 },
        Eigen::Vector3d{ -0.30, 0.20, 4.5 },
        Eigen::Vector3d{ 0.10, -0.25, 6.0 },
        Eigen::Vector3d{ 0.60, -0.10, 5.5 },
        Eigen::Vector3d{ -0.50, -0.20, 4.8 },
        Eigen::Vector3d{ 0.00, 0.30, 5.2 },
        Eigen::Vector3d{ 0.25, 0.15, 4.2 },
        Eigen::Vector3d{ -0.20, -0.15, 5.8 },
        Eigen::Vector3d{ 0.35, -0.05, 5.3 },
        Eigen::Vector3d{ -0.15, 0.25, 4.6 },
        Eigen::Vector3d{ 1.40, -0.30, 5.4 },
        Eigen::Vector3d{ 0.90, 0.35, 4.9 },
        Eigen::Vector3d{ 1.10, -0.15, 6.2 },
        Eigen::Vector3d{ 1.60, 0.05, 5.1 },
        Eigen::Vector3d{ 0.65, -0.40, 4.7 },
        Eigen::Vector3d{ 1.00, 0.20, 5.6 },
        Eigen::Vector3d{ 1.25, -0.20, 4.4 },
        Eigen::Vector3d{ 0.80, 0.30, 5.9 },
        Eigen::Vector3d{ 1.35, -0.05, 5.0 },
        Eigen::Vector3d{ 0.85, 0.15, 4.8 },
    };
    const Eigen::Matrix3d R_B_left =
        rodrigues( Eigen::Vector3d{ 0.2, -0.1, 0.05 } );
    const Eigen::Vector3d t_B_left{ 0.1, -0.02, 0.03 };

    MutantLinearSystem system{
        Eigen::MatrixXd::Zero( static_cast<Eigen::Index>( kRowCount ),
                               static_cast<Eigen::Index>( kColumnCount ) ),
        Eigen::MatrixXd::Zero( static_cast<Eigen::Index>( kRowCount ), 3 ),
        static_cast<Eigen::Index>( kPoseColumns + kPointColumns ),
    };
    const Eigen::Index point_offset =
        static_cast<Eigen::Index>( kPoseColumns );
    Eigen::Index row = 0;

    for ( std::size_t frame = 0; frame < kStateCount; ++frame )
    {
      for ( std::size_t landmark = 0; landmark < landmarks.size(); ++landmark )
      {
        const Eigen::Vector3d point_left =
            R_B_left.transpose() * ( landmarks[ landmark ] - t_B_left );
        const Eigen::Matrix3d projection =
            projectionJacobian( point_left ) / kStereoSigma;
        const Eigen::Matrix3d rotation =
            projection * R_B_left.transpose() * skew( landmarks[ landmark ] );
        const Eigen::Matrix3d translation =
            -projection * R_B_left.transpose();
        const Eigen::Matrix3d point = projection * R_B_left.transpose();
        const Eigen::Index    pose_column =
            static_cast<Eigen::Index>( 6U * frame );
        const Eigen::Index point_column =
            point_offset + static_cast<Eigen::Index>( 3U * landmark );
        for ( Eigen::Index residual_axis = 0; residual_axis < 3;
              ++residual_axis )
        {
          system.m_matrix.block<1, 3>( row, pose_column ) =
              rotation.row( residual_axis );
          system.m_matrix.block<1, 3>( row, pose_column + 3 ) =
              translation.row( residual_axis );
          system.m_matrix.block<1, 3>( row, point_column ) =
              point.row( residual_axis );
          ++row;
        }
      }
    }

    for ( Eigen::Index axis = 0; axis < 6; ++axis )
    {
      system.m_matrix( row, axis ) = 1.0 / kPosePrior;
      ++row;
    }

    const double rotation_sigma = kRecoveryNd * std::sqrt( kRecoveryDt );
    const double rw_sigma       = kRecoveryRw * std::sqrt( kRecoveryDt );
    for ( std::size_t link = 0; link < kLinkCount; ++link )
    {
      const Eigen::Vector3d observed_body    = truthBias( link );
      const Eigen::Vector3d observed_forward = R_B_left * observed_body;
      const Eigen::Vector3d observed_inverse =
          R_B_left.transpose() * observed_body;
      for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
      {
        const Eigen::Index axis_index = static_cast<Eigen::Index>( axis );
        const Eigen::Index pose_i =
            static_cast<Eigen::Index>( 6U * link ) + axis_index;
        const Eigen::Index pose_j =
            static_cast<Eigen::Index>( 6U * ( link + 1U ) ) + axis_index;
        const Eigen::Index bias_i =
            system.m_bias_offset + stateColumn( link, axis );
        const Eigen::Index bias_j =
            system.m_bias_offset + stateColumn( link + 1U, axis );
        system.m_matrix( row, pose_i ) = -1.0 / rotation_sigma;
        system.m_matrix( row, pose_j ) = 1.0 / rotation_sigma;
        system.m_matrix( row, bias_i ) = kRecoveryDt / rotation_sigma;
        system.m_rhs( row, 0 ) =
            kRecoveryDt * observed_body[ axis_index ] / rotation_sigma;
        system.m_rhs( row, 1 ) =
            kRecoveryDt * observed_forward[ axis_index ] / rotation_sigma;
        system.m_rhs( row, 2 ) =
            kRecoveryDt * observed_inverse[ axis_index ] / rotation_sigma;
        ++row;

        system.m_matrix( row, bias_i ) = -1.0 / rw_sigma;
        system.m_matrix( row, bias_j ) = 1.0 / rw_sigma;
        ++row;
      }
    }

    for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
    {
      system.m_matrix( row, system.m_bias_offset + stateColumn( 0U, axis ) ) =
          1.0 / kPriorSigma;
      ++row;
    }
    EXPECT_EQ( row, system.m_matrix.rows() );
    return system;
  }

  [[nodiscard]] Eigen::MatrixXd makeObservabilityMatrix( bool include_root,
                                                         bool include_rotation )
  {
    constexpr std::size_t kStates = 4U;
    constexpr std::size_t kLinks  = kStates - 1U;
    constexpr double      kDt     = 0.25;
    constexpr double      kNd     = 4e-4;
    constexpr double      kRw     = 1e-3;
    constexpr double      kPrior  = 0.1;
    const std::size_t     row_count =
        kLinks * kAxisCount + ( include_root ? kAxisCount : 0U ) +
        ( include_rotation ? kLinks * kAxisCount : 0U );
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>( row_count ),
        static_cast<Eigen::Index>( kStates * kAxisCount ) );
    const double rw_scale       = 1.0 / ( kRw * std::sqrt( kDt ) );
    const double rotation_scale = kDt / ( kNd * std::sqrt( kDt ) );
    Eigen::Index row            = 0;
    for ( std::size_t link = 0; link < kLinks; ++link )
    {
      for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
      {
        matrix( row, stateColumn( link, axis ) )      = -rw_scale;
        matrix( row, stateColumn( link + 1U, axis ) ) = rw_scale;
        ++row;
      }
    }
    if ( include_root )
    {
      for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
      {
        matrix( row, stateColumn( 0U, axis ) ) = 1.0 / kPrior;
        ++row;
      }
    }
    if ( include_rotation )
    {
      for ( std::size_t link = 0; link < kLinks; ++link )
      {
        for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
        {
          matrix( row, stateColumn( link, axis ) ) = rotation_scale;
          ++row;
        }
      }
    }
    EXPECT_EQ( row, matrix.rows() );
    return matrix;
  }

  [[nodiscard]] Eigen::Index numericalRank( const Eigen::MatrixXd& matrix )
  {
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd( matrix );
    const Eigen::VectorXd                   singular_values = svd.singularValues();
    const double                            tolerance =
        singular_values[ 0 ] *
        static_cast<double>( std::max( matrix.rows(), matrix.cols() ) ) *
        std::numeric_limits<double>::epsilon();
    Eigen::Index rank = 0;
    for ( const double value : singular_values )
    {
      if ( value > tolerance )
      {
        ++rank;
      }
    }
    return rank;
  }

  void expectAbsoluteAndRelative( double actual, double expected,
                                  double absolute_tolerance,
                                  double relative_tolerance )
  {
    const double absolute_error = std::abs( actual - expected );
    const double relative_error = absolute_error / std::abs( expected );
    EXPECT_LE( absolute_error, absolute_tolerance );
    EXPECT_LE( relative_error, relative_tolerance );
  }

}  // namespace

TEST( OnlineGyroBiasOracle, RecoversAllFourteenBiasStates )
{
  const LinearSystem    system = makeRecoverySystem();
  const Eigen::VectorXd solution =
      system.m_matrix.bdcSvd( Eigen::ComputeThinU | Eigen::ComputeThinV )
          .solve( system.m_rhs );
  ASSERT_EQ( solution.size(), static_cast<Eigen::Index>( kStateCount * 3U ) );
  for ( std::size_t state = 0; state < kStateCount; ++state )
  {
    for ( std::size_t axis = 0; axis < kAxisCount; ++axis )
    {
      const double actual = solution[ stateColumn( state, axis ) ];
      EXPECT_NEAR( actual,
                   kExpectedBiases[ state ][ static_cast<Eigen::Index>( axis ) ],
                   1e-12 );
    }
  }
}

TEST( OnlineGyroBiasOracle, FreezesWhiteningCovarianceAndCost )
{
  constexpr double kDt               = 0.25;
  constexpr double kNd               = 4e-4;
  constexpr double kRw               = 1e-3;
  const double     rotation_variance = kNd * kNd * kDt;
  const double     rw_variance       = kRw * kRw * kDt;
  EXPECT_NEAR( rotation_variance, 4e-8, 1e-18 );
  EXPECT_NEAR( rw_variance, 2.5e-7, 1e-18 );

  const Eigen::Matrix3d rotation_covariance =
      rotation_variance * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d rw_covariance =
      rw_variance * Eigen::Matrix3d::Identity();
  EXPECT_TRUE( rotation_covariance.isApprox(
      4e-8 * Eigen::Matrix3d::Identity(), 1e-12 ) );
  EXPECT_TRUE(
      rw_covariance.isApprox( 2.5e-7 * Eigen::Matrix3d::Identity(), 1e-12 ) );

  const Eigen::Vector3d rotation_residual{ 1e-4, -2e-4, 3e-4 };
  const Eigen::Vector3d rw_residual{ 2e-4, -1e-4, 3e-4 };
  const Eigen::Vector3d whitened_rotation =
      rotation_residual / std::sqrt( rotation_variance );
  const Eigen::Vector3d whitened_rw = rw_residual / std::sqrt( rw_variance );
  EXPECT_TRUE( whitened_rotation.isApprox(
      Eigen::Vector3d( 0.5, -1.0, 1.5 ), 1e-12 ) );
  EXPECT_TRUE(
      whitened_rw.isApprox( Eigen::Vector3d( 0.4, -0.2, 0.6 ), 1e-12 ) );
  EXPECT_NEAR( whitened_rotation.squaredNorm(), 3.5, 1e-12 );
  EXPECT_NEAR( whitened_rw.squaredNorm(), 0.56, 1e-12 );
  EXPECT_NEAR( 0.5 * whitened_rotation.squaredNorm(), 1.75, 1e-12 );
  EXPECT_NEAR( 0.5 * whitened_rw.squaredNorm(), 0.28, 1e-12 );
  EXPECT_NEAR( 0.5 * ( whitened_rotation.squaredNorm() +
                       whitened_rw.squaredNorm() ),
               2.03, 1e-12 );
}

TEST( OnlineGyroBiasOracle, FreezesObservabilityRankAndCondition )
{
  const Eigen::MatrixXd between_only =
      makeObservabilityMatrix( false, false );
  ASSERT_EQ( between_only.rows(), 9 );
  ASSERT_EQ( between_only.cols(), 12 );
  EXPECT_EQ( numericalRank( between_only ), 9 );
  EXPECT_EQ( between_only.cols() - numericalRank( between_only ), 3 );

  const Eigen::MatrixXd prior_only =
      makeObservabilityMatrix( true, false );
  ASSERT_EQ( prior_only.rows(), 12 );
  ASSERT_EQ( prior_only.cols(), 12 );
  EXPECT_EQ( numericalRank( prior_only ), 12 );
  constexpr bool kHasRotationInformation = false;
  EXPECT_FALSE( kHasRotationInformation );

  const Eigen::MatrixXd observed = makeObservabilityMatrix( true, true );
  ASSERT_EQ( observed.rows(), 21 );
  ASSERT_EQ( observed.cols(), 12 );
  EXPECT_EQ( numericalRank( observed ), 12 );
  const Eigen::VectorXd singular_values =
      observed.bdcSvd().singularValues();
  const std::array<double, 4U> expected_unique{
      3888.522624890523,
      3037.265646588222,
      1823.594963735669,
      1008.221501460588,
  };
  ASSERT_EQ( singular_values.size(), 12 );
  for ( std::size_t group = 0; group < expected_unique.size(); ++group )
  {
    for ( std::size_t repeat = 0; repeat < kAxisCount; ++repeat )
    {
      const Eigen::Index index =
          static_cast<Eigen::Index>( group * kAxisCount + repeat );
      expectAbsoluteAndRelative( singular_values[ index ],
                                 expected_unique[ group ], 1e-9, 1e-10 );
    }
  }

  const double condition_h =
      singular_values[ 0 ] / singular_values[ singular_values.size() - 1 ];
  expectAbsoluteAndRelative( condition_h, 3.8568138244, 1e-9, 1e-10 );
  EXPECT_LE( condition_h, 4.0 );

  const Eigen::MatrixXd                                normal = observed.transpose() * observed;
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver( normal );
  ASSERT_EQ( eigen_solver.info(), Eigen::Success );
  const Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
  const double          condition_normal =
      eigenvalues[ eigenvalues.size() - 1 ] / eigenvalues[ 0 ];
  expectAbsoluteAndRelative( condition_normal, 14.8750128761, 1e-9,
                             1e-10 );
  EXPECT_LE( condition_normal, 16.0 );
}

TEST( OnlineGyroBiasOracle, NonIdentityFrameMutantsCrossRecoveryGate )
{
  const MutantLinearSystem system = makeNonIdentityJointSystem();
  const Eigen::MatrixXd    solutions =
      system.m_matrix.bdcSvd( Eigen::ComputeThinU | Eigen::ComputeThinV )
          .solve( system.m_rhs );
  ASSERT_EQ( solutions.rows(), system.m_matrix.cols() );
  ASSERT_EQ( solutions.cols(), 3 );

  std::array<double, 3U> worst_errors{};
  for ( Eigen::Index candidate = 0; candidate < solutions.cols(); ++candidate )
  {
    for ( std::size_t state = 0; state < kStateCount; ++state )
    {
      const Eigen::Vector3d bias = solutions.block<3, 1>(
          system.m_bias_offset + stateColumn( state, 0U ), candidate );
      worst_errors[ static_cast<std::size_t>( candidate ) ] = std::max(
          worst_errors[ static_cast<std::size_t>( candidate ) ],
          ( bias - truthBias( state ) ).cwiseAbs().maxCoeff() );
    }
  }

  const Eigen::Vector3d correct_final = solutions.block<3, 1>(
      system.m_bias_offset + stateColumn( kStateCount - 1U, 0U ), 0 );
  const Eigen::Vector3d kExpectedCorrectFinal{
      0.013126669336378, -0.019698172265706, 0.027261275987490 };
  EXPECT_LE( ( correct_final - kExpectedCorrectFinal )
                 .cwiseAbs()
                 .maxCoeff(),
             1e-12 );
  EXPECT_LE( worst_errors[ 0 ], 5e-4 );
  EXPECT_NEAR( worst_errors[ 1 ], 0.0044797796936201645, 1e-12 );
  EXPECT_NEAR( worst_errors[ 2 ], 0.0052243530191383447, 1e-12 );
  EXPECT_GT( worst_errors[ 1 ], 5e-4 );
  EXPECT_GT( worst_errors[ 2 ], 5e-4 );
  EXPECT_NE( worst_errors[ 1 ], worst_errors[ 2 ] );
}
