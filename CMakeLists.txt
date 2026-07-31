cmake_minimum_required(VERSION 3.20)

project(phad_vio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PHAD_BUILD_TESTS "Build phad-vio tests" ON)
option(PHAD_ENABLE_MH01_TESTS "Enable local EuRoC MH_01 integration tests" OFF)
option(PHAD_ENABLE_TUMVI_CORRIDOR1_TESTS
  "Enable local TUM VI corridor1 integration tests" OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS
  core highgui imgcodecs imgproc calib3d video
)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(yaml-cpp 0.8 REQUIRED)
find_package(GTSAM 4.3 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

add_library(phad_common
  phad/common/trajectory.cpp
)
add_library(phad::common ALIAS phad_common)
target_compile_features(phad_common PUBLIC cxx_std_20)
target_include_directories(phad_common PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_common PUBLIC Eigen3::Eigen)
target_compile_options(phad_common PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_sensor
  phad/sensor/camera_parameters.cpp
  phad/sensor/imu_parameters.cpp
  phad/sensor/rigid_transform.cpp
  phad/sensor/stereo_imu_calibration.cpp
)
add_library(phad::sensor ALIAS phad_sensor)
target_compile_features(phad_sensor PUBLIC cxx_std_20)
target_include_directories(phad_sensor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_sensor PUBLIC Eigen3::Eigen)
target_compile_options(phad_sensor PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_camera
  phad/camera/camera_model.cpp
  phad/camera/rectified_stereo_calibration.cpp
  phad/camera/stereo_rectifier.cpp
)
add_library(phad::camera ALIAS phad_camera)
target_compile_features(phad_camera PUBLIC cxx_std_20)
target_include_directories(phad_camera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_camera
  PUBLIC phad::sensor
  PRIVATE opencv_core opencv_imgproc opencv_calib3d
)
target_compile_options(phad_camera PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_io INTERFACE)
add_library(phad::io ALIAS phad_io)
target_compile_features(phad_io INTERFACE cxx_std_20)
target_include_directories(phad_io INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_io INTERFACE phad::sensor)

add_library(phad_io_dataset
  phad/io/dataset/internal/dataset_adapter_utils.cpp
  phad/io/dataset/dataset_replay_source.cpp
  phad/io/dataset/stereo_imu_dataset.cpp
  phad/io/dataset/euroc/euroc_dataset.cpp
  phad/io/dataset/euroc/euroc_groundtruth.cpp
  phad/io/dataset/euroc/internal/euroc_yaml.cpp
  phad/io/dataset/tum_vi/tum_vi_dataset.cpp
)
add_library(phad::io_dataset ALIAS phad_io_dataset)

target_link_libraries(phad_io_dataset
  PUBLIC phad::io phad::common
  PRIVATE opencv_imgcodecs yaml-cpp::yaml-cpp
)
target_compile_options(phad_io_dataset PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_eval
  phad/eval/align.cpp
  phad/eval/associate.cpp
  phad/eval/ate.cpp
  phad/eval/error_stats.cpp
  phad/eval/rpe.cpp
  phad/eval/tum_io.cpp
)
add_library(phad::eval ALIAS phad_eval)
target_compile_features(phad_eval PUBLIC cxx_std_20)
target_include_directories(phad_eval PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_eval PUBLIC phad::common)
target_compile_options(phad_eval PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_frontend
  phad/frontend/stereo_tracker.cpp
)
add_library(phad::frontend ALIAS phad_frontend)
target_compile_features(phad_frontend PUBLIC cxx_std_20)
target_include_directories(phad_frontend PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_frontend
  PUBLIC phad::common phad::camera
  PRIVATE opencv_core opencv_imgproc opencv_video
)
target_compile_options(phad_frontend PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_viz
  phad/viz/image_window.cpp
  phad/viz/trajectory_panel.cpp
)
add_library(phad::viz ALIAS phad_viz)
target_compile_features(phad_viz PUBLIC cxx_std_20)
target_include_directories(phad_viz PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_viz
  PUBLIC phad::common opencv_core
  PRIVATE opencv_highgui opencv_imgproc
)
target_compile_options(phad_viz PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_estimator
  phad/estimator/stereo_vo_estimator.cpp
)
add_library(phad::estimator ALIAS phad_estimator)
target_compile_features(phad_estimator PUBLIC cxx_std_20)
target_include_directories(phad_estimator PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_estimator
  PUBLIC phad::common phad::camera
  PRIVATE gtsam
)
# GTSAM headers trip -Wconversion / -Wpedantic; keep them out of our warning set.
target_include_directories(phad_estimator SYSTEM PRIVATE
  $<TARGET_PROPERTY:gtsam,INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_options(phad_estimator PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_bench
  phad/bench/code_identity.cpp
  phad/bench/config_snapshot.cpp
  phad/bench/run_paths.cpp
  phad/bench/run_summary.cpp
)
add_library(phad::bench ALIAS phad_bench)
target_compile_features(phad_bench PUBLIC cxx_std_20)
target_include_directories(phad_bench PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_bench PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(phad_bench PRIVATE
  PHAD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_options(phad_bench PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

# phad::bench must stay free of phad::eval (and other phad::* libs).
get_target_property( _phad_bench_link_libs phad_bench LINK_LIBRARIES )
if( _phad_bench_link_libs MATCHES "phad_eval|phad::eval" )
  message( FATAL_ERROR "phad_bench must not link phad_eval" )
endif()
unset( _phad_bench_link_libs )

add_library(phad_offline_vo_session STATIC apps/offline_vo_session.cpp)
target_compile_features(phad_offline_vo_session PUBLIC cxx_std_20)
target_include_directories(phad_offline_vo_session PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(phad_offline_vo_session
  PUBLIC phad::io_dataset phad::camera phad::frontend phad::estimator
)
target_compile_options(phad_offline_vo_session PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

get_target_property( _phad_session_link_libs phad_offline_vo_session
  LINK_LIBRARIES )
if( _phad_session_link_libs MATCHES "phad_eval|phad::eval" )
  message( FATAL_ERROR "phad_offline_vo_session must not link phad_eval" )
endif()
unset( _phad_session_link_libs )

add_executable(phad_euroc_inspect apps/phad_euroc_inspect.cpp)
target_link_libraries(phad_euroc_inspect PRIVATE phad::io_dataset)

add_executable(phad_euroc_runner apps/phad_euroc_runner.cpp)
target_link_libraries(phad_euroc_runner
  PRIVATE phad::io_dataset phad::frontend phad::estimator phad::viz
          opencv_core opencv_imgproc
)

add_executable(phad_stereo_frontend_probe apps/phad_stereo_frontend_probe.cpp)
target_link_libraries(phad_stereo_frontend_probe
  PRIVATE phad::io_dataset phad::frontend
)

add_executable(phad_stereo_vo_probe apps/phad_stereo_vo_probe.cpp)
target_link_libraries(phad_stereo_vo_probe
  PRIVATE phad_offline_vo_session phad::eval
)

add_executable(phad_euroc_gt_export apps/phad_euroc_gt_export.cpp)
target_link_libraries(phad_euroc_gt_export PRIVATE phad::io_dataset phad::eval)

add_executable(phad_traj_eval apps/phad_traj_eval.cpp)
target_link_libraries(phad_traj_eval PRIVATE phad::io_dataset phad::eval)

if(PHAD_BUILD_TESTS)
  enable_testing()
  find_package(GTest 1.14 REQUIRED)
  include(GoogleTest)

  add_executable(phad_sensor_tests
    tests/sensor/calibration_error_header_test.cpp
    tests/sensor/camera_parameters_header_test.cpp
    tests/sensor/imu_parameters_header_test.cpp
    tests/sensor/rigid_transform_header_test.cpp
    tests/sensor/sensor_parameters_test.cpp
    tests/sensor/stereo_imu_calibration_header_test.cpp
    tests/sensor/stereo_imu_calibration_test.cpp
  )
  target_link_libraries(phad_sensor_tests PRIVATE
    phad::sensor GTest::gtest_main
  )
  target_compile_options(phad_sensor_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_sensor_tests PROPERTIES LABELS "unit")

  add_executable(phad_camera_tests
    tests/camera/camera_model_header_test.cpp
    tests/camera/camera_model_test.cpp
    tests/camera/rectified_stereo_calibration_test.cpp
    tests/camera/stereo_rectifier_test.cpp
  )
  target_link_libraries(phad_camera_tests PRIVATE
    phad::camera GTest::gtest_main
  )
  target_compile_options(phad_camera_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_camera_tests PROPERTIES LABELS "unit")

  add_executable(phad_eval_tests
    tests/common/trajectory_test.cpp
    tests/eval/align_test.cpp
    tests/eval/associate_test.cpp
    tests/eval/ate_test.cpp
    tests/eval/rpe_test.cpp
    tests/eval/tum_io_test.cpp
  )
  target_link_libraries(phad_eval_tests PRIVATE phad::eval GTest::gtest_main)
  target_include_directories(phad_eval_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(phad_eval_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_eval_tests PROPERTIES LABELS "unit")

  add_executable(phad_frontend_tests
    tests/frontend/stereo_tracker_test.cpp
  )
  target_link_libraries(phad_frontend_tests PRIVATE
    phad::frontend GTest::gtest_main
  )
  target_include_directories(phad_frontend_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_compile_options(phad_frontend_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_frontend_tests PROPERTIES LABELS "unit")

  add_executable(phad_estimator_tests
    tests/estimator/gtsam_smoke_test.cpp
    tests/estimator/types_test.cpp
    tests/estimator/stereo_point_mapping_test.cpp
    tests/estimator/stereo_vo_estimator_test.cpp
    tests/estimator/stereo_vo_diagnostics_test.cpp
  )
  target_link_libraries(phad_estimator_tests PRIVATE
    phad::estimator gtsam GTest::gtest_main
  )
  target_include_directories(phad_estimator_tests SYSTEM PRIVATE
    $<TARGET_PROPERTY:gtsam,INTERFACE_INCLUDE_DIRECTORIES>
  )
  target_compile_options(phad_estimator_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_estimator_tests PROPERTIES LABELS "unit")

  add_executable(phad_viz_tests
    tests/viz/trajectory_panel_test.cpp
  )
  target_link_libraries(phad_viz_tests PRIVATE
    phad::viz opencv_core GTest::gtest_main
  )
  target_include_directories(phad_viz_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(phad_viz_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_viz_tests PROPERTIES LABELS "unit")

  add_executable(phad_bench_tests
    tests/bench/code_identity_test.cpp
    tests/bench/config_snapshot_test.cpp
    tests/bench/run_paths_test.cpp
    tests/bench/run_summary_test.cpp
  )
  target_link_libraries(phad_bench_tests PRIVATE
    phad::bench GTest::gtest_main nlohmann_json::nlohmann_json
  )
  target_compile_options(phad_bench_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_bench_tests PROPERTIES LABELS "unit")

  add_executable(phad_apps_tests
    tests/apps/offline_vo_session_test.cpp
  )
  target_link_libraries(phad_apps_tests PRIVATE
    phad_offline_vo_session GTest::gtest_main
  )
  target_compile_options(phad_apps_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_apps_tests PROPERTIES LABELS "unit")

  add_executable(phad_io_dataset_tests
    tests/common/timestamp_test.cpp
    tests/io/dataset/dataset_replay_source_test.cpp
    tests/io/dataset/euroc/euroc_dataset_test.cpp
    tests/io/dataset/euroc/euroc_groundtruth_test.cpp
    tests/io/dataset/tum_vi/tum_vi_dataset_test.cpp
  )
  target_link_libraries(phad_io_dataset_tests PRIVATE
    phad::io_dataset GTest::gtest_main opencv_imgcodecs
  )
  target_compile_definitions(phad_io_dataset_tests PRIVATE
    PHAD_EUROC_INSPECT_PATH="$<TARGET_FILE:phad_euroc_inspect>"
  )
  add_dependencies(phad_io_dataset_tests phad_euroc_inspect)
  gtest_discover_tests(phad_io_dataset_tests PROPERTIES LABELS "unit")

  if(PHAD_ENABLE_MH01_TESTS)
    add_executable(phad_euroc_mh01_test
      tests/io/dataset/euroc/euroc_mh01_test.cpp
    )
    target_link_libraries(phad_euroc_mh01_test
      PRIVATE phad::io_dataset GTest::gtest_main
    )
    gtest_discover_tests(phad_euroc_mh01_test PROPERTIES LABELS "mh01")

    add_executable(phad_eval_mh01_test
      tests/eval/euroc_mh01_groundtruth_test.cpp
    )
    target_link_libraries(phad_eval_mh01_test
      PRIVATE phad::io_dataset phad::eval GTest::gtest_main
    )
    gtest_discover_tests(phad_eval_mh01_test PROPERTIES LABELS "mh01")

    add_executable(phad_camera_mh01_test
      tests/camera/euroc_mh01_rectify_test.cpp
    )
    target_link_libraries(phad_camera_mh01_test
      PRIVATE phad::camera phad::io_dataset GTest::gtest_main
              opencv_core opencv_imgproc opencv_video
    )
    gtest_discover_tests(phad_camera_mh01_test PROPERTIES LABELS "mh01")

    add_executable(phad_frontend_mh01_test
      tests/frontend/euroc_mh01_frontend_test.cpp
    )
    target_link_libraries(phad_frontend_mh01_test
      PRIVATE phad::frontend phad::io_dataset GTest::gtest_main
    )
    gtest_discover_tests(phad_frontend_mh01_test PROPERTIES LABELS "mh01")
  endif()

  if(PHAD_ENABLE_TUMVI_CORRIDOR1_TESTS)
    add_executable(phad_tumvi_corridor1_test
      tests/io/dataset/tum_vi/tum_vi_corridor1_test.cpp
    )
    target_link_libraries(phad_tumvi_corridor1_test
      PRIVATE phad::io_dataset GTest::gtest_main
    )
    gtest_discover_tests(phad_tumvi_corridor1_test
      PROPERTIES LABELS "tumvi-corridor1"
    )
  endif()
endif()
