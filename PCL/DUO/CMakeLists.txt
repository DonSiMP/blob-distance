cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMake-Modules" ${CMAKE_MODULE_PATH} )

project( DUO_3D )

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package( DUO REQUIRED )
find_package( OpenCV REQUIRED )

if( PCL_FOUND AND DUO_FOUND AND OPENCV_FOUND )
  add_executable( DUO_3D DUO.h ../grabber/duo_pcl_grabber.h ../detection-utils.h ../main.cpp )
  
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${OPENCVSDK_INCLUDE_DIRS} )
  include_directories( ${DUOSDK_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( -DDUO_GRABBER )
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${OPENCVSDK_LIBRARY_DIRS} )
  link_directories( ${DUOSDK_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( DUO_3D ${PCL_LIBRARIES} )
  target_link_libraries( DUO_3D ${OPENCVSDK_LIBRARIES} )
  target_link_libraries( DUO_3D ${DUOSDK_LIBRARIES} )
endif()

