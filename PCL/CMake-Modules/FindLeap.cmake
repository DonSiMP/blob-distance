#.rst:
# FindLeap
# --------------

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(Leap_FOUND FALSE)
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
    set(Leap_FOUND FALSE)
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

##### Find Leap Motion #####

# Found
set(Leap_FOUND TRUE)
if(MSVC_VERSION LESS 1700)
  message(WARNING "Leap Motion SDK supported Visual Studio 2012 or later.")
  set(Leap_FOUND FALSE)
endif()

# Root Directoty
set(LeapSDK_DIR)
if(Leap_FOUND)
  set(LeapSDK_DIR $ENV{LEAP_DIR} CACHE PATH "Leap Motion SDK Install Path." FORCE)
  check_dir(LeapSDK_DIR)
endif()

# Include Directories
set(LeapSDK_INCLUDE_DIRS)
if(Leap_FOUND)
  set(LeapSDK_INCLUDE_DIRS ${LeapSDK_DIR}/include)
  check_dir(LeapSDK_INCLUDE_DIRS)
endif()

# Library Directories
set(LeapSDK_LIBRARY_DIRS)
if(Leap_FOUND)
  set(LeapSDK_LIBRARY_DIRS ${LeapSDK_DIR}/lib/${TARGET_PLATFORM})
  check_dir(LeapSDK_LIBRARY_DIRS)
endif()

# Dependencies
set(LeapSDK_LIBRARIES)
if(Leap_FOUND)
  set(LeapSDK_LIBRARIES ${LeapSDK_LIBRARY_DIRS}/Leap.lib)
#set(LeapSDK_LIBRARIES ${LeapSDK_LIBRARIES};${LeapSDK_LIBRARY_DIRS}/libpxcmd.lib)

  check_files(LeapSDK_LIBRARIES LeapSDK_LIBRARY_DIRS)
endif()

message(STATUS "Leap_FOUND : ${Leap_FOUND}")