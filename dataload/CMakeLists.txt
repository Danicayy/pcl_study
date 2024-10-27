cmake_minimum_required(VERSION 3.0)
project(dataload)
include_directories("/usr/local/include/pcl-1.14")
include_directories("/usr/include/eigen3")
# 查找并配置VTK

find_package(PCL 1.14 REQUIRED)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 你的源代码文件
add_executable(dataload dataload.cpp)

# 链接VTK库
target_link_libraries(dataload ${VTK_LIBRARIES})

# 链接PCL库
target_link_libraries(dataload ${PCL_LIBRARIES})


