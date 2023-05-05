cmake_minimum_required(VERSION 3.0.2)
project(pcl_demo)

message(STATUS "a pcl transform demo")
message(STATUS "authur:ninedayhx")
message(STATUS "email:1170535490@qq.com")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)      
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE}) 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIR}
)

###########
## Build ##
###########
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) # 设置静态库文件目录
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) # 设置动态库文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(pcd_scale
	src/pcd_scale.cpp
)
target_link_libraries(pcd_scale
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)