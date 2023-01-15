cmake_minimum_required(VERSION 3.20)
project(PCV02)

set(OpenCV_DIR F:/Cpp/libs/opencv-4.5.1/opencv/build/x64/vc15/lib)

find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(PCV02 main.cpp Pcv2.cpp Pcv2.h Helper.cpp Helper.h)

target_link_libraries(PCV02 ${OpenCV_LIBS})