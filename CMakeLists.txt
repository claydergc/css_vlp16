cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(curvature_estimation)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} /usr/include/python2.7 .)
link_directories(${PCL_LIBRARY_DIRS} /usr/lib/python2.7/config-x86_64-linux-gnu)
add_definitions(${PCL_DEFINITIONS})

#add_executable (wall_curvature wall_curvature.cpp)
#target_link_libraries (wall_curvature ${PCL_LIBRARIES} python2.7)

#add_executable (range_measure range_measure.cpp)
#target_link_libraries (range_measure ${PCL_LIBRARIES} python2.7)

add_executable (main main.cpp css.cpp gaussians.cpp util.cpp)
target_link_libraries (main ${PCL_LIBRARIES} python2.7 boost_iostreams boost_system boost_filesystem)

