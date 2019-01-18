cmake_minimum_required (VERSION 2.8)

PROJECT(Navigation)

# Requires OpenCV
FIND_PACKAGE( OpenCV 2.4.8 REQUIRED )
MESSAGE("OpenCV version : ${OpenCV_VERSION}")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

# Use yaml-cpp package
link_directories(yaml-cpp/build)
include_directories(yaml-cpp/include)


# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIRS})
# include_directories( "/usr/include/eigen3" )
 
# 
ADD_EXECUTABLE( mat_test cvmat_test.cpp )
TARGET_LINK_LIBRARIES( mat_test ${OpenCV_LIBS} ) 

ADD_EXECUTABLE( astar_test src/astar.cpp src/astar_wrapper.cpp src/path_tracking.cpp)
TARGET_LINK_LIBRARIES( astar_test yaml-cpp ${OpenCV_LIBS} )
