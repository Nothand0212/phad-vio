#include "apps/stereo_vo_glue.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "phad/sensor/stereo_imu_packet.hpp"

namespace
{

  TEST( StereoVoGlueTest, AttachesPacketImuSegmentAsNonOwningView )
  {
    const phad::common::Timestamp t_prev{ 1'000'000'000 };
    const phad::common::Timestamp t_cur{ 1'050'000'000 };
    phad::frontend::FrameTracks   tracks{
          .timestamp    = t_cur,
          .observations = {},
          .stats        = {},
    };

    phad::sensor::StereoImuPacket packet{
        .frame =
            {
                .timestamp = t_cur,
                .left      = phad::sensor::Image(
                    1, 1, 1, std::vector<std::uint8_t>{ 0 } ),
                .right = phad::sensor::Image(
                    1, 1, 1, std::vector<std::uint8_t>{ 0 } ),
            },
        .samples =
            {
                { .timestamp = t_prev, .gyro_radps = { 0.1, 0.2, 0.3 } },
                { .timestamp = t_cur, .gyro_radps = { 0.4, 0.5, 0.6 } },
            },
        .t_prev  = t_prev,
        .imu_gap = false,
    };

    const phad::estimator::KeyframeMeasurement measurement =
        phad::apps::toKeyframeMeasurement( tracks, packet );

    EXPECT_EQ( measurement.timestamp, t_cur );
    EXPECT_EQ( measurement.imu.t_prev, t_prev );
    EXPECT_FALSE( measurement.imu.gap );
    ASSERT_EQ( measurement.imu.samples.size(), 2U );
    EXPECT_EQ( measurement.imu.samples.data(), packet.samples.data() );
    EXPECT_DOUBLE_EQ( measurement.imu.samples.back().gyro_radps[ 2 ], 0.6 );
  }

}  // namespace
