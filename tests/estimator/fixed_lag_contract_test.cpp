#include <gtest/gtest.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cstddef>
#include <map>
#include <string>

namespace
{

  using gtsam::symbol_shorthand::B;
  using gtsam::symbol_shorthand::X;
  using Smoother      = gtsam::IncrementalFixedLagSmoother;
  using Timestamps    = gtsam::FixedLagSmoother::KeyTimestampMap;
  using BetweenPoint2 = gtsam::BetweenFactor<gtsam::Point2>;

  gtsam::SharedNoiseModel pointNoise()
  {
    return gtsam::noiseModel::Isotropic::Sigma( 2, 0.1 );
  }

  struct FactorLedger
  {
    std::map<std::size_t, std::string> owners;
  };

  struct Snapshot
  {
    Smoother     smoother{ 1.0 };
    FactorLedger ledger;
  };

  TEST( FixedLagContractTest, CutoffIsStrictAndNewestUsesExactKey )
  {
    Smoother   smoother( 2.0 );
    const auto noise = pointNoise();

    {
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values               values;
      Timestamps                  timestamps;
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
          X( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
      values.insert( X( 0 ), gtsam::Point2( 0.0, 0.0 ) );
      timestamps.emplace( X( 0 ), 0.0 );
      smoother.update( factors, values, timestamps );
    }

    for ( std::size_t i = 1; i <= 2; ++i )
    {
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values               values;
      Timestamps                  timestamps;
      factors.emplace_shared<BetweenPoint2>(
          X( i - 1 ), X( i ), gtsam::Point2( 1.0, 0.0 ), noise );
      values.insert( X( i ),
                     gtsam::Point2( static_cast<double>( i ), 0.0 ) );
      timestamps.emplace( X( i ), static_cast<double>( i ) );
      smoother.update( factors, values, timestamps );
    }

    EXPECT_TRUE( smoother.getISAM2().valueExists( X( 0 ) ) );
    EXPECT_NE( smoother.timestamps().find( X( 0 ) ),
               smoother.timestamps().end() );

    gtsam::NonlinearFactorGraph factors;
    gtsam::Values               values;
    Timestamps                  timestamps;
    factors.emplace_shared<BetweenPoint2>(
        X( 2 ), X( 3 ), gtsam::Point2( 1.0, 0.0 ), noise );
    values.insert( X( 3 ), gtsam::Point2( 3.0, 0.0 ) );
    timestamps.emplace( X( 3 ), 3.0 );
    smoother.update( factors, values, timestamps );

    EXPECT_FALSE( smoother.getISAM2().valueExists( X( 0 ) ) );
    EXPECT_EQ( smoother.timestamps().find( X( 0 ) ),
               smoother.timestamps().end() );
    const gtsam::Point2 newest =
        smoother.calculateEstimate<gtsam::Point2>( X( 3 ) );
    EXPECT_NEAR( newest.x(), 3.0, 1e-9 );
    EXPECT_NEAR( newest.y(), 0.0, 1e-9 );
  }

  TEST( FixedLagContractTest,
        RefreshedGlobalSeparatorSurvivesAndBecomesFixed )
  {
    Smoother   smoother( 1.5 );
    const auto noise = pointNoise();

    {
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values               values;
      Timestamps                  timestamps;
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
          X( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
          B( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
      factors.emplace_shared<BetweenPoint2>(
          X( 0 ), B( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
      values.insert( X( 0 ), gtsam::Point2( 0.0, 0.0 ) );
      values.insert( B( 0 ), gtsam::Point2( 0.0, 0.0 ) );
      timestamps.emplace( X( 0 ), 0.0 );
      timestamps.emplace( B( 0 ), 0.0 );
      smoother.update( factors, values, timestamps );
    }

    {
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values               values;
      Timestamps                  timestamps;
      factors.emplace_shared<BetweenPoint2>(
          X( 0 ), X( 1 ), gtsam::Point2( 1.0, 0.0 ), noise );
      values.insert( X( 1 ), gtsam::Point2( 1.0, 0.0 ) );
      timestamps.emplace( X( 1 ), 1.0 );
      timestamps.emplace( B( 0 ), 1.0 );
      smoother.update( factors, values, timestamps );
    }

    EXPECT_EQ( smoother.getISAM2().getFixedVariables().count( B( 0 ) ),
               0U );

    {
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values               values;
      Timestamps                  timestamps;
      factors.emplace_shared<BetweenPoint2>(
          X( 1 ), X( 2 ), gtsam::Point2( 1.0, 0.0 ), noise );
      values.insert( X( 2 ), gtsam::Point2( 2.0, 0.0 ) );
      timestamps.emplace( X( 2 ), 2.0 );
      timestamps.emplace( B( 0 ), 2.0 );
      smoother.update( factors, values, timestamps );
    }

    EXPECT_TRUE( smoother.getISAM2().valueExists( B( 0 ) ) );
    const auto bias_timestamp = smoother.timestamps().find( B( 0 ) );
    ASSERT_NE( bias_timestamp, smoother.timestamps().end() );
    EXPECT_DOUBLE_EQ( bias_timestamp->second, 2.0 );
    EXPECT_EQ( smoother.getISAM2().getFixedVariables().count( B( 0 ) ),
               1U );
  }

  TEST( FixedLagContractTest, NewFactorIndicesTrackReusedSlots )
  {
    Smoother     smoother( 100.0 );
    const auto   noise = pointNoise();
    FactorLedger ledger;

    gtsam::NonlinearFactorGraph initial_factors;
    gtsam::Values               initial_values;
    Timestamps                  initial_timestamps;
    initial_factors.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
        X( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
    initial_factors.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
        B( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
    initial_factors.emplace_shared<BetweenPoint2>(
        X( 0 ), B( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
    initial_values.insert( X( 0 ), gtsam::Point2( 0.0, 0.0 ) );
    initial_values.insert( B( 0 ), gtsam::Point2( 0.0, 0.0 ) );
    initial_timestamps.emplace( X( 0 ), 0.0 );
    initial_timestamps.emplace( B( 0 ), 0.0 );
    smoother.update( initial_factors, initial_values, initial_timestamps );

    const gtsam::FactorIndices initial_slots =
        smoother.getISAM2Result().newFactorsIndices;
    ASSERT_EQ( initial_slots.size(), 3U );
    ledger.owners.emplace( initial_slots[ 0 ], "pose_prior" );
    ledger.owners.emplace( initial_slots[ 1 ], "global_prior" );
    ledger.owners.emplace( initial_slots[ 2 ], "old_between" );

    const std::size_t removed_slot = initial_slots[ 2 ];
    smoother.update( {}, {}, {}, gtsam::FactorIndices{ removed_slot } );
    EXPECT_FALSE( smoother.getFactors().exists( removed_slot ) );
    ledger.owners.erase( removed_slot );

    gtsam::NonlinearFactorGraph replacement;
    replacement.emplace_shared<BetweenPoint2>(
        X( 0 ), B( 0 ), gtsam::Point2( 0.01, 0.0 ), noise );
    smoother.update( replacement );

    const gtsam::FactorIndices replacement_slots =
        smoother.getISAM2Result().newFactorsIndices;
    ASSERT_EQ( replacement_slots.size(), 1U );
    EXPECT_EQ( replacement_slots.front(), removed_slot );
    EXPECT_TRUE( smoother.getFactors().exists( removed_slot ) );
    ledger.owners[ replacement_slots.front() ] = "new_between";
    EXPECT_EQ( ledger.owners.at( removed_slot ), "new_between" );
  }

  TEST( FixedLagContractTest,
        RemovingLastFactorLeavesTimestampOrphanAndExpiryThrows )
  {
    Smoother   smoother( 1.0 );
    const auto noise = pointNoise();

    gtsam::NonlinearFactorGraph root_factor;
    gtsam::Values               root_value;
    Timestamps                  root_timestamp;
    root_factor.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
        X( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
    root_value.insert( X( 0 ), gtsam::Point2( 0.0, 0.0 ) );
    root_timestamp.emplace( X( 0 ), 0.0 );
    smoother.update( root_factor, root_value, root_timestamp );
    const gtsam::FactorIndices root_slots =
        smoother.getISAM2Result().newFactorsIndices;
    ASSERT_EQ( root_slots.size(), 1U );

    smoother.update( {}, {}, {}, gtsam::FactorIndices{ root_slots.front() } );
    EXPECT_FALSE( smoother.getISAM2().valueExists( X( 0 ) ) );
    EXPECT_NE( smoother.timestamps().find( X( 0 ) ),
               smoother.timestamps().end() );

    gtsam::NonlinearFactorGraph next_factor;
    gtsam::Values               next_value;
    Timestamps                  next_timestamp;
    next_factor.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
        X( 1 ), gtsam::Point2( 1.0, 0.0 ), noise );
    next_value.insert( X( 1 ), gtsam::Point2( 1.0, 0.0 ) );
    next_timestamp.emplace( X( 1 ), 2.0 );
    EXPECT_THROW( smoother.update( next_factor, next_value, next_timestamp ),
                  std::exception );
  }

  TEST( FixedLagContractTest,
        SnapshotRestoresSmootherAndLedgerAfterFailedCandidate )
  {
    const auto noise = pointNoise();
    Snapshot   accepted;

    gtsam::NonlinearFactorGraph root_factor;
    gtsam::Values               root_value;
    Timestamps                  root_timestamp;
    root_factor.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
        X( 0 ), gtsam::Point2( 0.0, 0.0 ), noise );
    root_value.insert( X( 0 ), gtsam::Point2( 0.0, 0.0 ) );
    root_timestamp.emplace( X( 0 ), 0.0 );
    accepted.smoother.update( root_factor, root_value, root_timestamp );
    const gtsam::FactorIndices root_slots =
        accepted.smoother.getISAM2Result().newFactorsIndices;
    ASSERT_EQ( root_slots.size(), 1U );
    accepted.ledger.owners.emplace( root_slots.front(), "root" );

    Snapshot candidate = accepted;
    candidate.smoother.update(
        {}, {}, {}, gtsam::FactorIndices{ root_slots.front() } );
    candidate.ledger.owners.erase( root_slots.front() );
    candidate.ledger.owners.emplace( 99U, "failed_candidate" );

    gtsam::NonlinearFactorGraph bad_factor;
    gtsam::Values               bad_value;
    Timestamps                  bad_timestamp;
    bad_factor.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
        X( 1 ), gtsam::Point2( 2.0, 0.0 ), noise );
    bad_value.insert( X( 1 ), gtsam::Point2( 2.0, 0.0 ) );
    bad_timestamp.emplace( X( 1 ), 2.0 );
    EXPECT_THROW(
        candidate.smoother.update( bad_factor, bad_value, bad_timestamp ),
        std::exception );

    EXPECT_TRUE( accepted.smoother.getISAM2().valueExists( X( 0 ) ) );
    EXPECT_TRUE(
        accepted.smoother.getFactors().exists( root_slots.front() ) );
    EXPECT_EQ( accepted.ledger.owners.at( root_slots.front() ), "root" );
    EXPECT_EQ( accepted.ledger.owners.count( 99U ), 0U );

    Snapshot                    restored = accepted;
    gtsam::NonlinearFactorGraph valid_factor;
    gtsam::Values               valid_value;
    Timestamps                  valid_timestamp;
    valid_factor.emplace_shared<BetweenPoint2>(
        X( 0 ), X( 1 ), gtsam::Point2( 1.0, 0.0 ), noise );
    valid_value.insert( X( 1 ), gtsam::Point2( 1.0, 0.0 ) );
    valid_timestamp.emplace( X( 1 ), 1.0 );
    EXPECT_NO_THROW( restored.smoother.update(
        valid_factor, valid_value, valid_timestamp ) );
    EXPECT_EQ( restored.ledger.owners.at( root_slots.front() ), "root" );
    EXPECT_EQ( restored.ledger.owners.count( 99U ), 0U );

    const gtsam::Point2 newest =
        restored.smoother.calculateEstimate<gtsam::Point2>( X( 1 ) );
    EXPECT_NEAR( newest.x(), 1.0, 1e-9 );
    EXPECT_NEAR( newest.y(), 0.0, 1e-9 );
  }

}  // namespace
