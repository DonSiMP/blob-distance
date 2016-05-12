cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMake-Modules" ${CMAKE_MODULE_PATH} )

project( LEAP )

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package( Leap REQUIRED )

if( PCL_FOUND AND Leap_FOUND )
  add_executable( LEAP ../grabber/leap_pcl_grabber.h ../detection-utils.h ../main.cpp )
  
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${LeapSDK_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( -DLEAP_GRABBER )
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${LeapSDK_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( LEAP ${PCL_LIBRARIES} )
  target_link_libraries( LEAP ${LeapSDK_LIBRARIES} )
endif()

