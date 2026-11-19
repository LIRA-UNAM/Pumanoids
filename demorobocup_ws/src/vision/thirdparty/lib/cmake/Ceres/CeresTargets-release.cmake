#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Ceres::ceres_cuda_kernels" for configuration "Release"
set_property(TARGET Ceres::ceres_cuda_kernels APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Ceres::ceres_cuda_kernels PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CUDA"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libceres_cuda_kernels.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS Ceres::ceres_cuda_kernels )
list(APPEND _IMPORT_CHECK_FILES_FOR_Ceres::ceres_cuda_kernels "${_IMPORT_PREFIX}/lib/libceres_cuda_kernels.a" )

# Import target "Ceres::ceres" for configuration "Release"
set_property(TARGET Ceres::ceres APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Ceres::ceres PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libceres.so.2.2.0"
  IMPORTED_SONAME_RELEASE "libceres.so.4"
  )

list(APPEND _IMPORT_CHECK_TARGETS Ceres::ceres )
list(APPEND _IMPORT_CHECK_FILES_FOR_Ceres::ceres "${_IMPORT_PREFIX}/lib/libceres.so.2.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
