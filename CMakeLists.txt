cmake_minimum_required(VERSION 3.20)

project(phad_vio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PHAD_BUILD_TESTS "Build phad-vio tests" ON)
option(PHAD_ENABLE_MH01_TESTS "Enable local EuRoC MH_01 integration tests" OFF)
option(PHAD_ENABLE_TUMVI_CORRIDOR1_TESTS
  "Enable local TUM VI corridor1 integration tests" OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS imgcodecs)
find_package(yaml-cpp 0.8 REQUIRED)

add_library(phad_dataset
  phad/dataset/internal/dataset_adapter_utils.cpp
  phad/dataset/stereo_imu_dataset.cpp
  phad/dataset/euroc/euroc_dataset.cpp
  phad/dataset/tum_vi/tum_vi_dataset.cpp
)
add_library(phad::dataset ALIAS phad_dataset)

target_compile_features(phad_dataset PUBLIC cxx_std_20)
target_include_directories(phad_dataset PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_dataset PRIVATE opencv_imgcodecs yaml-cpp::yaml-cpp)
target_compile_options(phad_dataset PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_executable(phad_euroc_inspect apps/phad_euroc_inspect.cpp)
target_link_libraries(phad_euroc_inspect PRIVATE phad::dataset)

if(PHAD_BUILD_TESTS)
  enable_testing()
  find_package(GTest 1.14 REQUIRED)
  include(GoogleTest)

  add_executable(phad_dataset_tests
    tests/common/timestamp_test.cpp
    tests/dataset/euroc/euroc_dataset_test.cpp
    tests/dataset/tum_vi/tum_vi_dataset_test.cpp
  )
  target_link_libraries(phad_dataset_tests PRIVATE
    phad::dataset GTest::gtest_main opencv_imgcodecs
  )
  gtest_discover_tests(phad_dataset_tests PROPERTIES LABELS "unit")

  if(PHAD_ENABLE_MH01_TESTS)
    add_executable(phad_euroc_mh01_test
      tests/dataset/euroc/euroc_mh01_test.cpp
    )
    target_link_libraries(phad_euroc_mh01_test PRIVATE phad::dataset GTest::gtest_main)
    gtest_discover_tests(phad_euroc_mh01_test PROPERTIES LABELS "mh01")
  endif()

  if(PHAD_ENABLE_TUMVI_CORRIDOR1_TESTS)
    add_executable(phad_tumvi_corridor1_test
      tests/dataset/tum_vi/tum_vi_corridor1_test.cpp
    )
    target_link_libraries(phad_tumvi_corridor1_test
      PRIVATE phad::dataset GTest::gtest_main
    )
    gtest_discover_tests(phad_tumvi_corridor1_test
      PROPERTIES LABELS "tumvi-corridor1"
    )
  endif()
endif()
