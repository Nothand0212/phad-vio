#include "phad/bench/config_snapshot.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace
{

  using phad::bench::ConfigSnapshot;

  TEST( ConfigSnapshotTest, InsertionOrderDoesNotAffectHash )
  {
    ConfigSnapshot a;
    a.set( "z.flag", true );
    a.set( "a.value", 0.1 );
    a.set( "m.count", static_cast<std::int64_t>( 3 ) );

    ConfigSnapshot b;
    b.set( "m.count", static_cast<std::int64_t>( 3 ) );
    b.set( "z.flag", true );
    b.set( "a.value", 0.1 );

    EXPECT_EQ( a.canonicalText(), b.canonicalText() );
    EXPECT_EQ( a.hash8(), b.hash8() );
  }

  TEST( ConfigSnapshotTest, SameContentSameHash )
  {
    ConfigSnapshot a;
    a.set( "tracker.max_features", static_cast<std::int64_t>( 200 ) );
    a.set( "estimator.window", static_cast<std::int64_t>( 10 ) );

    ConfigSnapshot b;
    b.set( "tracker.max_features", static_cast<std::int64_t>( 200 ) );
    b.set( "estimator.window", static_cast<std::int64_t>( 10 ) );

    EXPECT_EQ( a.hash8(), b.hash8() );
    EXPECT_EQ( a.hash8().size(), 8U );
  }

  TEST( ConfigSnapshotTest, FieldChangeChangesHash )
  {
    ConfigSnapshot a;
    a.set( "x", 1.0 );
    ConfigSnapshot b;
    b.set( "x", 1.0000001 );
    EXPECT_NE( a.hash8(), b.hash8() );
  }

  TEST( ConfigSnapshotTest, FloatFormatsAreStable )
  {
    ConfigSnapshot a;
    a.set( "scale", 0.1 );
    ConfigSnapshot b;
    b.set( "scale", 1e-1 );
    // %.17g round-trips binary doubles; 0.1 and 1e-1 share one text form.
    EXPECT_EQ( a.canonicalText(), b.canonicalText() );
    EXPECT_EQ( a.hash8(), b.hash8() );
    EXPECT_EQ( a.canonicalText(), "scale=0.10000000000000001\n" );
  }

  TEST( ConfigSnapshotTest, EmptySnapshotHasDeterministicHash )
  {
    const ConfigSnapshot empty;
    EXPECT_TRUE( empty.empty() );
    EXPECT_EQ( empty.canonicalText(), "" );
    EXPECT_EQ( empty.hash8(), "cbf29ce4" );
  }

  TEST( ConfigSnapshotTest, ToJsonRoundTripsScalars )
  {
    ConfigSnapshot snap;
    snap.set( "name", std::string( "default" ) );
    snap.set( "enabled", true );
    snap.set( "count", static_cast<std::int64_t>( 7 ) );
    snap.set( "scale", 0.5 );
    const std::string text = snap.toJson();
    EXPECT_NE( text.find( "\"name\":\"default\"" ), std::string::npos );
    EXPECT_NE( text.find( "\"enabled\":true" ), std::string::npos );
    EXPECT_NE( text.find( "\"count\":7" ), std::string::npos );
  }

  TEST( ConfigSnapshotTest, EmptyKeyThrows )
  {
    ConfigSnapshot snap;
    EXPECT_THROW( snap.set( "", 1.0 ), std::invalid_argument );
  }

  TEST( ConfigSnapshotTest, MaxOutlierReoptsChangesConfigHash )
  {
    ConfigSnapshot defaults;
    defaults.set( "estimator.enable_outlier_reopt", true );
    defaults.set( "estimator.max_outlier_reopts",
                  static_cast<std::int64_t>( 3 ) );

    ConfigSnapshot override_max;
    override_max.set( "estimator.enable_outlier_reopt", true );
    override_max.set( "estimator.max_outlier_reopts",
                      static_cast<std::int64_t>( 1 ) );

    EXPECT_NE( defaults.hash8(), override_max.hash8() );
    EXPECT_NE( defaults.canonicalText().find(
                   "estimator.max_outlier_reopts=3" ),
               std::string::npos );
    EXPECT_NE( override_max.canonicalText().find(
                   "estimator.max_outlier_reopts=1" ),
               std::string::npos );
  }

}  // namespace
