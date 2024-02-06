# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_reflex-control-cpp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED reflex-control-cpp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(reflex-control-cpp_FOUND FALSE)
  elseif(NOT reflex-control-cpp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(reflex-control-cpp_FOUND FALSE)
  endif()
  return()
endif()
set(_reflex-control-cpp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT reflex-control-cpp_FIND_QUIETLY)
  message(STATUS "Found reflex-control-cpp: 0.0.0 (${reflex-control-cpp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'reflex-control-cpp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${reflex-control-cpp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(reflex-control-cpp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${reflex-control-cpp_DIR}/${_extra}")
endforeach()
