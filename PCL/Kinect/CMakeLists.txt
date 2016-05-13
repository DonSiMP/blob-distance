cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../CMake-Modules" ${CMAKE_MODULE_PATH} )

project( kinect )

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package( KinectSDK2 REQUIRED )

if( PCL_FOUND AND KinectSDK2_FOUND )
  add_executable( kinect ../grabber/kinect_pcl_grabber.h ../detection-utils.h ../main.cpp )
  
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( -DKINECT2_GRABBER )
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( kinect ${PCL_LIBRARIES} )
  target_link_libraries( kinect ${KinectSDK2_LIBRARIES} )
endif()

