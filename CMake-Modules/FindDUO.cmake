#.rst:
# FindDUO
# --------------

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(DUO_FOUND FALSE)
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
    set(DUO_FOUND FALSE)
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

##### Find DUO Motion #####

# Found
set(DUO_FOUND TRUE)
if(MSVC_VERSION LESS 1700)
  message(WARNING "DUO Motion SDK supported Visual Studio 2012 or later.")
  set(DUO_FOUND FALSE)
endif()

# Root Directoty
set(DUOSDK_DIR)
if(DUO_FOUND)
  set(DUOSDK_DIR $ENV{DUO3D_DIR} CACHE PATH "DUO Motion SDK Install Path." FORCE)
  check_dir(DUOSDK_DIR)
endif()

# Include Directories
set(DUOSDK_INCLUDE_DIRS)
if(DUO_FOUND)
  set(DUOSDK_INCLUDE_DIRS ${DUOSDK_DIR}/include)
  check_dir(DUOSDK_INCLUDE_DIRS)
endif()

# Library Directories
set(DUOSDK_LIBRARY_DIRS)
if(DUO_FOUND)
  set(DUOSDK_LIBRARY_DIRS ${DUOSDK_DIR}/windows/${TARGET_PLATFORM})
  check_dir(DUOSDK_LIBRARY_DIRS)
endif()

# Dependencies
set(DUOSDK_LIBRARIES)
if(DUO_FOUND)
  set(DUOSDK_LIBRARIES ${DUOSDK_LIBRARY_DIRS}/Dense3D.lib)
  set(DUOSDK_LIBRARIES ${DUOSDK_LIBRARIES};${DUOSDK_LIBRARY_DIRS}/DUOLib.lib)

  check_files(DUOSDK_LIBRARIES DUOSDK_LIBRARY_DIRS)
endif()

message(STATUS "DUO_FOUND : ${DUO_FOUND}")