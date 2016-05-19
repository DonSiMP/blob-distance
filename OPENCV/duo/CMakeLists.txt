cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../CMake-Modules" ${CMAKE_MODULE_PATH} )

project( blob-detect )

# Find Packages
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
find_package( DUO REQUIRED )

if( OPENCV_FOUND AND DUO_FOUND )
  add_executable( blob-detect DUO.h ../prog.cpp ../grabber/duo_grabber.h )
  
  # Additional Include Directories
  include_directories( ${OPENCVSDK_INCLUDE_DIRS} )
  include_directories( ${BOOST_INCLUDE_DIRS} )
  include_directories( ${DUOSDK_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( -DDUO_GRABBER )
  add_definitions( ${OPENCV_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${OPENCVSDK_LIBRARY_DIRS} )
  link_directories( ${BOOST_LIBRARY_DIRS} )
  link_directories( ${DUOSDK_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( blob-detect ${OPENCVSDK_LIBRARIES} )
  target_link_libraries( blob-detect ${BOOST_LIBRARIES} )
  target_link_libraries( blob-detect ${DUOSDK_LIBRARIES} )
endif()

