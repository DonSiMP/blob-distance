#.rst:
# FindRealSense
# --------------

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(RealSense_FOUND FALSE)
    unset(_DIR)
  endif()
endmacro()

# Check Files Macro
macro(CHECK_FILES _FILES _DIR)
  set(_MISSING_FILES)
  foreach(_FILE ${${_FILES}})
    if(NOT EXISTS "${_FILE}")
      get_filename_component(_FILE ${_FILE} NAME)
      set(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
    endif()
  endforeach()
  if(_MISSING_FILES)
    message(WARNING "In directory \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
    set(RealSense_FOUND FALSE)
    unset(_FILES)
  endif()
endmacro()

# Target Platform
set(TARGET_PLATFORM)
if(NOT CMAKE_CL_64)
  set(TARGET_PLATFORM x86)
else()
  set(TARGET_PLATFORM x64)
endif()

##### Find Intel Real Sense #####

# Found
set(RealSense_FOUND TRUE)
if(MSVC_VERSION LESS 1700)
  message(WARNING "Real Sense SDK supported Visual Studio 2012 or later.")
  set(RealSense_FOUND FALSE)
endif()

# Root Directoty
set(RealSenseSDK_DIR)
if(RealSense_FOUND)
  set(RealSenseSDK_DIR $ENV{RSSDK_DIR} CACHE PATH "Real Sense SDK Install Path." FORCE)
  check_dir(RealSenseSDK_DIR)
endif()

# Include Directories
set(RealSenseSDK_INCLUDE_DIRS)
if(RealSense_FOUND)
  set(RealSenseSDK_INCLUDE_DIRS ${RealSenseSDK_DIR}/Include)
  check_dir(RealSenseSDK_INCLUDE_DIRS)
endif()

# Library Directories
set(RealSenseSDK_LIBRARY_DIRS)
if(RealSense_FOUND)
  set(RealSenseSDK_LIBRARY_DIRS ${RealSenseSDK_DIR}/lib/${TARGET_PLATFORM})
  check_dir(RealSenseSDK_LIBRARY_DIRS)
endif()

# Dependencies
set(RealSenseSDK_LIBRARIES)
if(RealSense_FOUND)
  set(RealSenseSDK_LIBRARIES ${RealSenseSDK_LIBRARY_DIRS}/libpxc.lib)
    set(RealSenseSDK_LIBRARIES ${RealSenseSDK_LIBRARIES};${RealSenseSDK_LIBRARY_DIRS}/libpxcmd.lib)

  check_files(RealSenseSDK_LIBRARIES RealSenseSDK_LIBRARY_DIRS)
endif()

message(STATUS "RealSense_FOUND : ${RealSense_FOUND}")