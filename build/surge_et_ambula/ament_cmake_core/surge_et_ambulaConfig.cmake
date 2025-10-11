# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_surge_et_ambula_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED surge_et_ambula_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(surge_et_ambula_FOUND FALSE)
  elseif(NOT surge_et_ambula_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(surge_et_ambula_FOUND FALSE)
  endif()
  return()
endif()
set(_surge_et_ambula_CONFIG_INCLUDED TRUE)

# output package information
if(NOT surge_et_ambula_FIND_QUIETLY)
  message(STATUS "Found surge_et_ambula: 0.0.0 (${surge_et_ambula_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'surge_et_ambula' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT surge_et_ambula_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(surge_et_ambula_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${surge_et_ambula_DIR}/${_extra}")
endforeach()
