#include <array>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

int main()
{
  const Eigen::Vector3d vector{ 1.0, 2.0, 3.0 };
  const cv::Mat         identity = cv::Mat::eye( 2, 2, CV_64F );
  const gtsam::Pose3    pose;
  const YAML::Node      yaml = YAML::Load( "ready: true" );
  const nlohmann::json  json{ { "ready", yaml[ "ready" ].as<bool>() } };
  const std::string     payload = json.dump();
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  SHA256( reinterpret_cast<const unsigned char*>( payload.data() ),
          payload.size(), digest.data() );

  const bool ready = vector.sum() == 6.0 && identity.at<double>( 0, 0 ) == 1.0 &&
                     pose.equals( gtsam::Pose3{}, 0.0 ) && json[ "ready" ] &&
                     digest.front() != digest.back() &&
                     testing::UnitTest::GetInstance() != nullptr;
  if ( !ready )
  {
    return 1;
  }
  std::cout << "remote-ci dependency smoke: ok\n";
  return 0;
}
