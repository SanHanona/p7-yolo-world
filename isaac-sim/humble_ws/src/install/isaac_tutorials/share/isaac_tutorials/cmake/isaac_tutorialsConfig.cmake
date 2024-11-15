# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_isaac_tutorials_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED isaac_tutorials_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(isaac_tutorials_FOUND FALSE)
  elseif(NOT isaac_tutorials_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(isaac_tutorials_FOUND FALSE)
  endif()
  return()
endif()
set(_isaac_tutorials_CONFIG_INCLUDED TRUE)

# output package information
if(NOT isaac_tutorials_FIND_QUIETLY)
  message(STATUS "Found isaac_tutorials: 0.1.0 (${isaac_tutorials_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'isaac_tutorials' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${isaac_tutorials_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(isaac_tutorials_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${isaac_tutorials_DIR}/${_extra}")
endforeach()
