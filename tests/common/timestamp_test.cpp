#include "phad/common/timestamp.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <type_traits>

namespace
{

  using phad::common::Timestamp;

  TEST( TimestampTest, PreservesAndOrdersFullInt64Nanoseconds )
  {
    constexpr std::int64_t earlier_ns = 1'403'636'579'763'555'584;
    constexpr std::int64_t later_ns   = earlier_ns + 1;

    constexpr Timestamp earlier{ earlier_ns };
    constexpr Timestamp later{ later_ns };

    static_assert( std::is_same_v<decltype( earlier.nanoseconds() ), std::int64_t> );
    EXPECT_EQ( earlier.nanoseconds(), earlier_ns );
    EXPECT_LT( earlier, later );
    EXPECT_NE( earlier, later );
    EXPECT_EQ( Timestamp{ std::numeric_limits<std::int64_t>::max() }.nanoseconds(),
               std::numeric_limits<std::int64_t>::max() );
  }

}  // namespace
