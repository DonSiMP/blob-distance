#.rst:
# FindBoost
# --------------

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(BOOST_FOUND FALSE)
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
    set(BOOST_FOUND FALSE)
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

##### Find BOOST  #####

# Found
set(BOOST_FOUND TRUE)

# Root Directoty
set(BOOST_DIR)
if(BOOST_FOUND)
  set(BOOST_DIR $ENV{BOOST_DIR} CACHE PATH "BOOST  SDK Install Path." FORCE)
  check_dir(BOOST_DIR)
endif()

# Include Directories
set(BOOST_INCLUDE_DIRS)
if(BOOST_FOUND)
  set(BOOST_INCLUDE_DIRS ${BOOST_DIR}/include/boost-1_60)
  check_dir(BOOST_INCLUDE_DIRS)
endif()

# Library Directories
set(BOOST_LIBRARY_DIRS)
if(BOOST_FOUND)
  set(BOOST_LIBRARY_DIRS ${BOOST_DIR}/lib)
  check_dir(BOOST_LIBRARY_DIRS)
endif()

# Dependencies
set(BOOST_LIBRARIES)
if(BOOST_FOUND)
  set(BOOST_LIBRARIES ${BOOST_LIBRARY_DIRS}/libboost_atomic-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_chrono-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_container-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_context-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_coroutine-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_date_time-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_exception-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_filesystem-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_graph-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_iostreams-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_locale-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_log_setup-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_log-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_math_c99f-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_math_c99l-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_math_c99-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_math_tr1f-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_math_tr1l-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_math_tr1-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_mpi-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_prg_exec_monitor-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_program_options-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_random-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_regex-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_serialization-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_signals-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_system-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_test_exec_monitor-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_thread-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_timer-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_type_erasure-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_unit_test_framework-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_wave-vc140-mt-1_60.lib)
  set(BOOST_LIBRARIES ${BOOST_LIBRARIES};${BOOST_LIBRARY_DIRS}/libboost_wserialization-vc140-mt-1_60.lib)

  check_files(BOOST_LIBRARIES BOOST_LIBRARY_DIRS)
endif()

message(STATUS "BOOST_FOUND : ${BOOST_FOUND}")