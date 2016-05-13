cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../CMake-Modules" ${CMAKE_MODULE_PATH} )

project( R200 )

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package( RealSense REQUIRED )

if( PCL_FOUND AND RealSense_FOUND )
  add_executable( R200 ../grabber/r200_pcl_grabber.h ../detection-utils.h ../main.cpp )
  
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${RealSenseSDK_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( -DR200_GRABBER )
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${RealSenseSDK_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( R200 ${PCL_LIBRARIES} )
  target_link_libraries( R200 ${RealSenseSDK_LIBRARIES} )
endif()

