# Provides the imported target onnxruntime::onnxruntime.
#
# By default the official prebuilt ONNX Runtime (CPU) release matching the
# target architecture is downloaded at configure time. To use a local install
# instead pass -DONNXRUNTIME_ROOT=/path/to/onnxruntime (folder containing
# include/ and lib/).

set(ONNXRUNTIME_VERSION "1.30.0" CACHE STRING "Prebuilt ONNX Runtime version to download")
set(ONNXRUNTIME_ROOT "" CACHE PATH "Local ONNX Runtime install (skips download)")

if(NOT ONNXRUNTIME_ROOT)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_ort_arch "aarch64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
    set(_ort_arch "x64")
  else()
    message(FATAL_ERROR "No prebuilt ONNX Runtime for ${CMAKE_SYSTEM_PROCESSOR}; set ONNXRUNTIME_ROOT")
  endif()

  include(FetchContent)
  set(_ort_name "onnxruntime-linux-${_ort_arch}-${ONNXRUNTIME_VERSION}")
  # DOWNLOAD_EXTRACT_TIMESTAMP only exists since CMake 3.24 (older versions,
  # e.g. 3.16 on Ubuntu 20.04 / Foxy, would parse it as part of the URL).
  set(_ort_extra_args "")
  if(NOT CMAKE_VERSION VERSION_LESS 3.24)
    set(_ort_extra_args DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  endif()
  FetchContent_Declare(onnxruntime_prebuilt
    URL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${_ort_name}.tgz"
    ${_ort_extra_args}
  )
  FetchContent_MakeAvailable(onnxruntime_prebuilt)
  set(ONNXRUNTIME_ROOT "${onnxruntime_prebuilt_SOURCE_DIR}")
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_ROOT}/include" "${ONNXRUNTIME_ROOT}/include/onnxruntime"
  NO_DEFAULT_PATH REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")

# Ship the runtime library with the package so the nodes run without a
# system-wide install.
file(GLOB ONNXRUNTIME_SHARED_LIBS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so*")
install(FILES ${ONNXRUNTIME_SHARED_LIBS} DESTINATION lib)
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
