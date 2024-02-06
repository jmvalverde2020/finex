# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_reflex-exo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED reflex-exo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(reflex-exo_FOUND FALSE)
  elseif(NOT reflex-exo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(reflex-exo_FOUND FALSE)
  endif()
  return()
endif()
set(_reflex-exo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT reflex-exo_FIND_QUIETLY)
  message(STATUS "Found reflex-exo: 0.0.0 (${reflex-exo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'reflex-exo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${reflex-exo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(reflex-exo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${reflex-exo_DIR}/${_extra}")
endforeach()
