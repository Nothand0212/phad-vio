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
find_package(OpenSSL REQUIRED COMPONENTS Crypto)

set(PHAD_Q3_SOURCE_COMMIT "" CACHE STRING
  "Git commit embedded in the Q3 gyro-alignment runner")
set(PHAD_Q3_SOURCE_TREE "" CACHE STRING
  "Git tree embedded in the Q3 gyro-alignment runner")
execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  RESULT_VARIABLE _phad_q3_git_commit_result
  OUTPUT_VARIABLE _phad_q3_git_commit
  ERROR_QUIET
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
  COMMAND git rev-parse HEAD^{tree}
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  RESULT_VARIABLE _phad_q3_git_tree_result
  OUTPUT_VARIABLE _phad_q3_git_tree
  ERROR_QUIET
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(_phad_q3_git_commit_result EQUAL 0 AND _phad_q3_git_tree_result EQUAL 0)
  if(NOT PHAD_Q3_SOURCE_COMMIT STREQUAL "" AND
      NOT PHAD_Q3_SOURCE_COMMIT STREQUAL _phad_q3_git_commit)
    message(FATAL_ERROR
      "PHAD_Q3_SOURCE_COMMIT must equal git rev-parse HEAD")
  endif()
  if(NOT PHAD_Q3_SOURCE_TREE STREQUAL "" AND
      NOT PHAD_Q3_SOURCE_TREE STREQUAL _phad_q3_git_tree)
    message(FATAL_ERROR
      "PHAD_Q3_SOURCE_TREE must equal git rev-parse HEAD^{tree}")
  endif()
  set(PHAD_Q3_SOURCE_COMMIT "${_phad_q3_git_commit}")
  set(PHAD_Q3_SOURCE_TREE "${_phad_q3_git_tree}")
elseif(PHAD_Q3_SOURCE_COMMIT STREQUAL "" OR PHAD_Q3_SOURCE_TREE STREQUAL "")
  message(FATAL_ERROR
    "A source archive without Git must provide PHAD_Q3_SOURCE_COMMIT and PHAD_Q3_SOURCE_TREE")
endif()
unset(_phad_q3_git_commit_result)
unset(_phad_q3_git_tree_result)
unset(_phad_q3_git_commit)
unset(_phad_q3_git_tree)
foreach(_phad_q3_identity IN ITEMS
    PHAD_Q3_SOURCE_COMMIT PHAD_Q3_SOURCE_TREE)
  string(LENGTH "${${_phad_q3_identity}}" _phad_q3_identity_length)
  if(NOT _phad_q3_identity_length EQUAL 40 OR
      NOT ${_phad_q3_identity} MATCHES "^[0-9a-f]+$")
    message(FATAL_ERROR
      "${_phad_q3_identity} must be an exact lowercase 40-hex Git object")
  endif()
endforeach()
unset(_phad_q3_identity)
unset(_phad_q3_identity_length)

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

add_library(phad_sync
  phad/sync/stereo_pair_synchronizer.cpp
)
add_library(phad::sync ALIAS phad_sync)
target_compile_features(phad_sync PUBLIC cxx_std_20)
target_include_directories(phad_sync PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_sync PUBLIC phad::sensor phad::common)
target_compile_options(phad_sync PRIVATE
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
  phad/estimator/gyro_rotation_predictor.cpp
  phad/estimator/internal/gyro_interval_reducer.cpp
  phad/estimator/internal/imu_interval.cpp
  phad/estimator/vio_estimator.cpp
)
add_library(phad::estimator ALIAS phad_estimator)
target_compile_features(phad_estimator PUBLIC cxx_std_20)
target_include_directories(phad_estimator PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(phad_estimator
  PUBLIC phad::common phad::camera phad::sensor
  PRIVATE gtsam opencv_calib3d
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

add_library(phad_stereo_pair_stream STATIC apps/stereo_pair_stream.cpp)
target_compile_features(phad_stereo_pair_stream PUBLIC cxx_std_20)
target_include_directories(phad_stereo_pair_stream PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(phad_stereo_pair_stream
  PUBLIC phad::io_dataset phad::sync
)
target_compile_options(phad_stereo_pair_stream PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_offline_vo_session STATIC
  apps/offline_vo_session.cpp
  apps/probe_b_writer.cpp
)
target_compile_features(phad_offline_vo_session PUBLIC cxx_std_20)
target_include_directories(phad_offline_vo_session PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(phad_offline_vo_session
  PUBLIC phad_stereo_pair_stream phad::camera phad::frontend phad::estimator
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

add_library(phad_gyro_observe_writer STATIC
  apps/gyro_observe_writer.cpp
)
target_compile_features(phad_gyro_observe_writer PUBLIC cxx_std_20)
target_include_directories(phad_gyro_observe_writer PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(phad_gyro_observe_writer
  PUBLIC phad_offline_vo_session
)
target_compile_options(phad_gyro_observe_writer PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_library(phad_gyro_alignment STATIC
  apps/gyro_alignment.cpp
  apps/gyro_alignment.hpp
)
target_compile_features(phad_gyro_alignment PUBLIC cxx_std_20)
target_include_directories(phad_gyro_alignment PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(phad_gyro_alignment
  PUBLIC phad::estimator Eigen3::Eigen
)
target_compile_options(phad_gyro_alignment PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

# Q3's analyzer reaches GTSAM only behind the Q2 estimator helper. It must not
# grow a direct dependency or acquire dataset/GT I/O.
get_target_property(_phad_q3_analyzer_link_libs phad_gyro_alignment
  LINK_LIBRARIES)
if(_phad_q3_analyzer_link_libs MATCHES
    "(^|;)(gtsam|phad_io_dataset|phad::io_dataset)(;|$)")
  message(FATAL_ERROR
    "phad_gyro_alignment has a forbidden direct GTSAM/dataset dependency")
endif()
unset(_phad_q3_analyzer_link_libs)

add_library(phad_gyro_alignment_runner STATIC
  apps/gyro_alignment_runner.cpp
  apps/gyro_alignment_runner.hpp
)
target_compile_features(phad_gyro_alignment_runner PUBLIC cxx_std_20)
target_include_directories(phad_gyro_alignment_runner PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(phad_gyro_alignment_runner
  PUBLIC phad_gyro_alignment
  PRIVATE phad::eval nlohmann_json::nlohmann_json OpenSSL::Crypto
)
target_compile_definitions(phad_gyro_alignment_runner PRIVATE
  PHAD_Q3_DESIGN_COMMIT="16affcd2fc271f1db4d6a060b3d2864004d8a40b"
  PHAD_Q3_DESIGN_TREE="634b048fd9501a9c807ebfcc645f5ce9fe4d8bb3"
  PHAD_Q3_DESIGN_BLOB="a6eaa88b4553cb03f012bd090d502bc3ff3576e3"
  PHAD_Q3_DESIGN_SHA256="1e2f20d181af5453b383ceb09d53850045e7b68ec17b13f8d40c883d417c081b"
  PHAD_Q3_SOURCE_COMMIT="${PHAD_Q3_SOURCE_COMMIT}"
  PHAD_Q3_SOURCE_TREE="${PHAD_Q3_SOURCE_TREE}"
  PHAD_Q3_BUILD_TYPE="$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,Unspecified>"
  PHAD_Q3_COMPILER="${CMAKE_CXX_COMPILER_ID}"
  PHAD_Q3_COMPILER_VERSION="${CMAKE_CXX_COMPILER_VERSION}"
)
target_compile_options(phad_gyro_alignment_runner PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

get_target_property(_phad_q3_runner_link_libs phad_gyro_alignment_runner
  LINK_LIBRARIES)
if(_phad_q3_runner_link_libs MATCHES
    "(^|;)(phad_io_dataset|phad::io_dataset)(;|$)")
  message(FATAL_ERROR
    "phad_gyro_alignment_runner must not link dataset or GT I/O")
endif()
unset(_phad_q3_runner_link_libs)

add_executable(phad_gyro_align apps/phad_gyro_align.cpp)
target_link_libraries(phad_gyro_align PRIVATE phad_gyro_alignment_runner)
target_compile_options(phad_gyro_align PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)

add_executable(phad_euroc_inspect apps/phad_euroc_inspect.cpp)
target_link_libraries(phad_euroc_inspect PRIVATE phad::io_dataset)

add_executable(phad_euroc_runner apps/phad_euroc_runner.cpp)
target_link_libraries(phad_euroc_runner
  PRIVATE phad_stereo_pair_stream phad::frontend phad::estimator phad::viz
          opencv_core opencv_imgproc
)

add_executable(phad_stereo_frontend_probe apps/phad_stereo_frontend_probe.cpp)
target_link_libraries(phad_stereo_frontend_probe
  PRIVATE phad_stereo_pair_stream phad::frontend
)

add_executable(phad_stereo_vo_probe apps/phad_stereo_vo_probe.cpp)
target_link_libraries(phad_stereo_vo_probe
  PRIVATE phad_offline_vo_session phad::eval
)

add_executable(phad_vo_bench apps/phad_vo_bench.cpp)
target_link_libraries(phad_vo_bench
  PRIVATE phad_offline_vo_session phad_gyro_observe_writer phad::bench
          phad::eval phad::io_dataset
)
target_compile_definitions(phad_vo_bench PRIVATE
  PHAD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_options(phad_vo_bench PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
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

  add_executable(phad_sync_tests
    tests/sync/stereo_pair_synchronizer_test.cpp
  )
  target_link_libraries(phad_sync_tests PRIVATE
    phad::sync GTest::gtest_main
  )
  target_compile_options(phad_sync_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_sync_tests PROPERTIES LABELS "unit")

  add_executable(phad_camera_tests
    tests/camera/camera_model_header_test.cpp
    tests/camera/camera_model_test.cpp
    tests/camera/rectified_stereo_calibration_test.cpp
    tests/camera/stereo_rectifier_test.cpp
  )
  target_link_libraries(phad_camera_tests PRIVATE
    phad::camera GTest::gtest_main
    opencv_core opencv_imgproc opencv_calib3d
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
    tests/estimator/gyro_rotation_predictor_test.cpp
    tests/estimator/gtsam_smoke_test.cpp
    tests/estimator/types_test.cpp
    tests/estimator/stereo_point_mapping_test.cpp
    tests/estimator/stereo_vo_estimator_test.cpp
    tests/estimator/stereo_vo_diagnostics_test.cpp
    tests/estimator/vio_lifecycle_test.cpp
    tests/estimator/stereo_vo_pnp_test.cpp
    tests/estimator/stereo_vo_outlier_cull_test.cpp
    tests/estimator/stereo_vo_outlier_reopt_test.cpp
    tests/estimator/stereo_vo_cull_rebirth_test.cpp
    tests/estimator/keyframe_update_test.cpp
    tests/estimator/multiframe_triangulation_test.cpp
    tests/estimator/vio_full_state_test.cpp
    tests/estimator/vio_eviction_test.cpp
    tests/estimator/mapped_landmark_bearing_test.cpp
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
    tests/apps/gyro_observe_artifact_test.cpp
    tests/apps/offline_vo_session_test.cpp
    tests/apps/phad_vo_bench_cli_test.cpp
    tests/apps/keyframe_selection_test.cpp
  )
  target_link_libraries(phad_apps_tests PRIVATE
    phad_offline_vo_session phad_gyro_observe_writer GTest::gtest_main
    nlohmann_json::nlohmann_json opencv_imgcodecs
  )
  target_compile_definitions(phad_apps_tests PRIVATE
    PHAD_VO_BENCH_PATH="$<TARGET_FILE:phad_vo_bench>"
  )
  add_dependencies(phad_apps_tests phad_vo_bench)
  target_compile_options(phad_apps_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_apps_tests PROPERTIES LABELS "unit")

  add_executable(phad_gyro_alignment_tests
    tests/apps/gyro_alignment_test.cpp
    tests/apps/gyro_alignment_cli_test.cpp
  )
  target_link_libraries(phad_gyro_alignment_tests PRIVATE
    phad_gyro_alignment_runner GTest::gtest_main
    nlohmann_json::nlohmann_json OpenSSL::Crypto
  )
  target_compile_definitions(phad_gyro_alignment_tests PRIVATE
    PHAD_GYRO_ALIGN_PATH="$<TARGET_FILE:phad_gyro_align>"
    PHAD_Q3_EXPECTED_SOURCE_COMMIT="${PHAD_Q3_SOURCE_COMMIT}"
    PHAD_Q3_EXPECTED_SOURCE_TREE="${PHAD_Q3_SOURCE_TREE}"
    PHAD_Q3_EXPECTED_BUILD_TYPE="$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,Unspecified>"
    PHAD_Q3_EXPECTED_COMPILER="${CMAKE_CXX_COMPILER_ID}"
    PHAD_Q3_EXPECTED_COMPILER_VERSION="${CMAKE_CXX_COMPILER_VERSION}"
  )
  add_dependencies(phad_gyro_alignment_tests phad_gyro_align)
  target_compile_options(phad_gyro_alignment_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  )
  gtest_discover_tests(phad_gyro_alignment_tests PROPERTIES LABELS "unit")

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
    add_executable(phad_apps_mh01_test
      tests/apps/gyro_observe_mh01_test.cpp
    )
    target_link_libraries(phad_apps_mh01_test PRIVATE
      phad_offline_vo_session GTest::gtest_main nlohmann_json::nlohmann_json
    )
    target_compile_definitions(phad_apps_mh01_test PRIVATE
      PHAD_VO_BENCH_PATH="$<TARGET_FILE:phad_vo_bench>"
    )
    add_dependencies(phad_apps_mh01_test phad_vo_bench)
    target_compile_options(phad_apps_mh01_test PRIVATE
      $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
    )
    gtest_discover_tests(phad_apps_mh01_test PROPERTIES LABELS "mh01")

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
      PRIVATE phad::camera phad_stereo_pair_stream GTest::gtest_main
              opencv_core opencv_imgproc opencv_video
    )
    gtest_discover_tests(phad_camera_mh01_test PROPERTIES LABELS "mh01")

    add_executable(phad_frontend_mh01_test
      tests/frontend/euroc_mh01_frontend_test.cpp
    )
    target_link_libraries(phad_frontend_mh01_test
      PRIVATE phad::frontend phad_stereo_pair_stream GTest::gtest_main
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

    add_executable(phad_tumvi_corridor1_product_test
      tests/apps/tum_vi_corridor1_product_test.cpp
    )
    target_link_libraries(phad_tumvi_corridor1_product_test
      PRIVATE phad::io_dataset phad_offline_vo_session GTest::gtest_main
    )
    target_compile_options(phad_tumvi_corridor1_product_test PRIVATE
      $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
    )
    gtest_discover_tests(phad_tumvi_corridor1_product_test
      PROPERTIES LABELS "tumvi-corridor1"
    )
  endif()
endif()
