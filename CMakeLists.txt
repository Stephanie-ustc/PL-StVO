project( poliSVO )

cmake_minimum_required(VERSION 2.7)
find_package( OpenCV REQUIRED)	
if(COMMAND cmake_policy)
  	cmake_policy(SET CMP0003 NEW)	
endif(COMMAND cmake_policy)

SET(DEFAULT_HAS_MRPT ON)
SET(HAS_MRPT ${DEFAULT_HAS_MRPT} CACHE BOOL "Build the PointGrey Bumblebee2 SVO application that employs the MRPT library")

if(HAS_MRPT)
find_package( MRPT REQUIRED base opengl gui hwdrivers)
add_executable       ( bumblebeeSVO  	src/bumblebeeSVO.cpp )
target_link_libraries( bumblebeeSVO	${OpenCV_LIBS} ${MRPT_LIBS} )
endif(HAS_MRPT)

add_executable	     ( testSVO	src/testSVO.cpp   )
target_link_libraries( testSVO	${OpenCV_LIBS}    )

SET(BUILD_SHARED_LIBS ON)
SET(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -mtune=native")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -O3")



