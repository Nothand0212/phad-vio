#pragma once

/**
 * @file synthetic_euroc_fixture.hpp
 * @brief 测试共用：带纹理点阵的 EuRoC 形状临时序列。
 *
 * 供 offline_vo_session_test 与 phad_vo_bench_cli_test 复用：pinhole 零畸变
 * 双目 + 4×5 静态点阵 + 静止 IMU 段 + identity groundtruth，使
 * runOfflineVoSession() / phad_vo_bench 走成功路径，candidate twin 可见真实
 * tracks/poses。tracker 需关 stereo_uniq_ratio / stereo_check_bidir
 * （合成全同 blob 无区分度，与 candidate_pipeline_test 的 trackerOptions
 * 一致）。
 */

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include "phad/sensor/stereo_frame.hpp"
#include "tests/frontend/synthetic_stereo.hpp"

namespace phad::testing
{

  class SyntheticEurocFixture
  {
  public:
    static constexpr std::int64_t kFirstTimestampNs =
        1'403'636'579'763'555'584LL;
    static constexpr std::int64_t kStepNs = 50'000'000LL;

    SyntheticEurocFixture()
    {
      m_root = std::filesystem::temp_directory_path() /
               "phad_offline_vo_session_synthetic_seq";
      std::filesystem::remove_all( m_root );
      for ( const auto* sensor :
            { "cam0", "cam1", "imu0", "state_groundtruth_estimate0" } )
      {
        std::filesystem::create_directories( m_root / "mav0" / sensor /
                                             "data" );
      }
      const auto calibration =
          phad::testing::makeRectifiedCalibration( 320, 240, 250.0, 0.11 );
      const auto points = phad::testing::makePointGrid( calibration, 4, 5 );
      // 逐帧非直线运动（x/y 分量不成比例）：静止或一维运动的 trajectory
      // 会使 ATE SVD 对齐退化（collinear/coincident）。
      const auto shift = [ &points ]( double x_m, double y_m ) {
        auto moved = points;
        for ( Eigen::Vector3d& point : moved )
        {
          point.x() += x_m;
          point.y() += y_m;
        }
        return moved;
      };
      writeCalibration();
      writeCsv( "imu0", imuHeader() + imuRows() );
      writeGroundtruth();
      const auto first = phad::testing::renderStereo(
          calibration, points,
          phad::common::Timestamp{ kFirstTimestampNs }, 2.0 );
      const auto second = phad::testing::renderStereo(
          calibration, shift( 0.1, 0.05 ),
          phad::common::Timestamp{ kFirstTimestampNs + kStepNs }, 2.0 );
      const auto third = phad::testing::renderStereo(
          calibration, shift( 0.2, 0.02 ),
          phad::common::Timestamp{ kFirstTimestampNs + 2 * kStepNs }, 2.0 );
      writeImage( "cam0", "left-a.png", first.left );
      writeImage( "cam0", "left-b.png", second.left );
      writeImage( "cam0", "left-c.png", third.left );
      writeImage( "cam1", "right-a.png", first.right );
      writeImage( "cam1", "right-b.png", second.right );
      writeImage( "cam1", "right-c.png", third.right );
      writeCsv( "cam0", "#timestamp [ns],filename\n" +
                            std::to_string( kFirstTimestampNs ) +
                            ",left-a.png\n" +
                            std::to_string( kFirstTimestampNs + kStepNs ) +
                            ",left-b.png\n" +
                            std::to_string( kFirstTimestampNs + 2 * kStepNs ) +
                            ",left-c.png\n" );
      writeCsv( "cam1", "#timestamp [ns],filename\n" +
                            std::to_string( kFirstTimestampNs ) +
                            ",right-a.png\n" +
                            std::to_string( kFirstTimestampNs + kStepNs ) +
                            ",right-b.png\n" +
                            std::to_string( kFirstTimestampNs + 2 * kStepNs ) +
                            ",right-c.png\n" );
    }

    ~SyntheticEurocFixture() { std::filesystem::remove_all( m_root ); }

    SyntheticEurocFixture( const SyntheticEurocFixture& )            = delete;
    SyntheticEurocFixture& operator=( const SyntheticEurocFixture& ) = delete;

    [[nodiscard]] const std::filesystem::path& root() const { return m_root; }

  private:
    static std::string imuHeader()
    {
      return "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
             "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
             "a_RS_S_z [m s^-2]\n";
    }

    static std::string imuRows()
    {
      // 静止段 [t0, t0+25ms, t1] 与 [t1, t1+25ms, t2]：端点匹配 sync
      // 段合同，全零测量。
      return std::to_string( kFirstTimestampNs ) + ",0,0,0,0,0,0\n" +
             std::to_string( kFirstTimestampNs + kStepNs / 2 ) +
             ",0,0,0,0,0,0\n" +
             std::to_string( kFirstTimestampNs + kStepNs ) +
             ",0,0,0,0,0,0\n" +
             std::to_string( kFirstTimestampNs + 3 * kStepNs / 2 ) +
             ",0,0,0,0,0,0\n" +
             std::to_string( kFirstTimestampNs + 2 * kStepNs ) +
             ",0,0,0,0,0,0\n";
    }

    static std::string groundtruthHeader()
    {
      // 必须与 euroc_groundtruth.cpp 的 kGroundtruthHeader 逐字节一致。
      return "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], "
             "q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], "
             "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], "
             "b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], "
             "b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], "
             "b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n";
    }

    void writeGroundtruth()
    {
      // 点阵非直线运动 → 相机相对世界反方向运动；GT 平移与估计同向
      // 且三点不共线（避免 ATE SVD 对齐退化）。
      const std::string row0 = ",0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0\n";
      const std::string row1 = ",-0.1,-0.05,0,1,0,0,0,0,0,0,0,0,0,0,0,0\n";
      const std::string row2 = ",-0.2,-0.02,0,1,0,0,0,0,0,0,0,0,0,0,0,0\n";
      writeCsv( "state_groundtruth_estimate0",
                groundtruthHeader() +
                    std::to_string( kFirstTimestampNs ) + row0 +
                    std::to_string( kFirstTimestampNs + kStepNs ) + row1 +
                    std::to_string( kFirstTimestampNs + 2 * kStepNs ) + row2 );
      std::ofstream( m_root / "mav0" / "state_groundtruth_estimate0" /
                     "sensor.yaml" )
          << "sensor_type: imu\n"
             "T_BS:\n"
             "  rows: 4\n"
             "  cols: 4\n"
             "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
             "rate_hz: 200\n";
    }

    void writeCsv( const std::string& sensor, const std::string& contents )
    {
      std::ofstream( m_root / "mav0" / sensor / "data.csv" ) << contents;
    }

    void writeImage( const std::string& sensor, const std::string& filename,
                     const phad::sensor::Image& image )
    {
      const auto pixels = image.pixels<std::uint8_t>();
      if ( !pixels.has_value() )
      {
        throw std::runtime_error( "synthetic image is not uint8" );
      }
      cv::Mat mat( image.height(), image.width(), CV_8UC1 );
      std::memcpy( mat.data, pixels->data(), pixels->size() );
      cv::imwrite(
          ( m_root / "mav0" / sensor / "data" / filename ).string(), mat );
    }

    void writeCalibration()
    {
      const std::string cam0_yaml =
          "sensor_type: camera\n"
          "T_BS:\n"
          "  rows: 4\n"
          "  cols: 4\n"
          "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
          "rate_hz: 20\n"
          "resolution: [320, 240]\n"
          "camera_model: pinhole\n"
          "intrinsics: [250, 250, 159.5, 119.5]\n"
          "distortion_model: radial-tangential\n"
          "distortion_coefficients: [0, 0, 0, 0]\n";
      const std::string cam1_yaml =
          "sensor_type: camera\n"
          "T_BS:\n"
          "  rows: 4\n"
          "  cols: 4\n"
          "  data: [1, 0, 0, 0.11, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
          "rate_hz: 20\n"
          "resolution: [320, 240]\n"
          "camera_model: pinhole\n"
          "intrinsics: [250, 250, 159.5, 119.5]\n"
          "distortion_model: radial-tangential\n"
          "distortion_coefficients: [0, 0, 0, 0]\n";
      const std::string imu_yaml =
          "sensor_type: imu\n"
          "T_BS:\n"
          "  rows: 4\n"
          "  cols: 4\n"
          "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
          "rate_hz: 200\n"
          "gyroscope_noise_density: 0.0001\n"
          "gyroscope_random_walk: 0.00001\n"
          "accelerometer_noise_density: 0.002\n"
          "accelerometer_random_walk: 0.003\n";
      std::ofstream( m_root / "mav0" / "cam0" / "sensor.yaml" ) << cam0_yaml;
      std::ofstream( m_root / "mav0" / "cam1" / "sensor.yaml" ) << cam1_yaml;
      std::ofstream( m_root / "mav0" / "imu0" / "sensor.yaml" ) << imu_yaml;
    }

    std::filesystem::path m_root;
  };

}  // namespace phad::testing
