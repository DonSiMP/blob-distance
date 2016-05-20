cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../CMake-Modules" ${CMAKE_MODULE_PATH} )

project( blob-detect )

# Find Packages
find_package( OpenCV REQUIRED )
find_package( MyBoost REQUIRED )
find_package( RealSense REQUIRED )

if( OPENCV_FOUND AND RealSense_FOUND )
  add_executable( blob-detect ../prog.cpp ../grabber/realSense_grabber.h )
  
  # Additional Include Directories
  include_directories( ${OPENCVSDK_INCLUDE_DIRS} )
  include_directories( ${MY_BOOST_INCLUDE_DIRS} )
  include_directories( ${RealSenseSDK_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( -DR200_GRABBER )
  add_definitions( ${OPENCV_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${OPENCVSDK_LIBRARY_DIRS} )
  link_directories( ${MY_BOOST_LIBRARY_DIRS} )
  link_directories( ${RealSenseSDK_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( blob-detect ${OPENCVSDK_LIBRARIES} )
  target_link_libraries( blob-detect ${MY_BOOST_LIBRARIES} )
  target_link_libraries( blob-detect ${RealSenseSDK_LIBRARIES} )
endif()

