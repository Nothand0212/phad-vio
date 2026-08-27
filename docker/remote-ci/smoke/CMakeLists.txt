cmake_minimum_required(VERSION 3.20)

project(phad_remote_ci_smoke LANGUAGES CXX)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(GTest 1.14 REQUIRED)
find_package(GTSAM 4.3 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs imgproc calib3d video)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(yaml-cpp 0.8 REQUIRED)

add_executable(phad-remote-ci-smoke smoke.cpp)
target_compile_features(phad-remote-ci-smoke PRIVATE cxx_std_20)
target_link_libraries(phad-remote-ci-smoke PRIVATE
  Eigen3::Eigen
  GTest::gtest
  gtsam
  nlohmann_json::nlohmann_json
  opencv_calib3d
  opencv_core
  opencv_imgcodecs
  opencv_imgproc
  opencv_video
  OpenSSL::Crypto
  yaml-cpp::yaml-cpp
)
