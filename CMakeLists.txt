cmake_minimum_required(VERSION 3.0.0)  
project(Test)  

# find_library(QUIRC_LIB quirc)  # 查找 QUIRC 库
find_package(OpenCV REQUIRED)  
set(CMAKE_BUILD_TYPE Debug)
  
include_directories("/home/ubuntu/smart-car/opencv/quirc/*")

include_directories("/home/ubuntu/smart-car/opencv/tracking.cpp")
add_executable(opencv opencv.cpp)  
target_link_libraries(opencv ${OpenCV_LIBS})
