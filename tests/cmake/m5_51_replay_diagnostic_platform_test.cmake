foreach(_required_variable IN ITEMS
    PHAD_TEST_SOURCE_DIR
    PHAD_TEST_BINARY_DIR
    PHAD_TEST_GENERATOR)
  if(NOT DEFINED ${_required_variable} OR "${${_required_variable}}" STREQUAL "")
    message(FATAL_ERROR "${_required_variable} must be provided")
  endif()
endforeach()

string(RANDOM LENGTH 32 ALPHABET 0123456789abcdef
  _m5_51_build_suffix)
set(_non_linux_build_dir
  "${PHAD_TEST_BINARY_DIR}/m5-51-replay-diagnostic-non-linux-${_m5_51_build_suffix}")
set(_error_reason "")
execute_process(
  COMMAND "${CMAKE_COMMAND}"
    -S "${PHAD_TEST_SOURCE_DIR}"
    -B "${_non_linux_build_dir}"
    -G "${PHAD_TEST_GENERATOR}"
    -DCMAKE_SYSTEM_NAME=Darwin
    -DPHAD_BUILD_TESTS=OFF
    -DPHAD_ENABLE_M5_51_REPLAY_DIAGNOSTIC=ON
  RESULT_VARIABLE _configure_result
  OUTPUT_VARIABLE _configure_stdout
  ERROR_VARIABLE _configure_stderr
)

string(CONCAT _configure_output
  "${_configure_stdout}"
  "${_configure_stderr}")
if(_configure_result EQUAL 0)
  set(_error_reason
    "Non-Linux diagnostic configuration unexpectedly succeeded:\n${_configure_output}")
elseif(NOT _configure_output MATCHES
    "PHAD_ENABLE_M5_51_REPLAY_DIAGNOSTIC requires Linux")
  set(_error_reason
    "Non-Linux diagnostic configuration did not report its platform contract:\n"
    "${_configure_output}")
endif()

if(EXISTS "${_non_linux_build_dir}")
  file(REMOVE_RECURSE "${_non_linux_build_dir}")
endif()
if(EXISTS "${_non_linux_build_dir}")
  string(APPEND _error_reason
    "Failed to clean diagnostic platform-test directory: "
    "${_non_linux_build_dir}\n")
endif()

if(NOT _error_reason STREQUAL "")
  message(FATAL_ERROR "${_error_reason}")
endif()
