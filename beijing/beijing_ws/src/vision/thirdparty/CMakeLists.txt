# Note: CMake support is community-based. The maintainers do not use CMake
# internally.

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

include(CMakeDependentOption)
include(GNUInstallDirs)

if(BUILD_TEST)
  add_subdirectory( googletest )
endif()
