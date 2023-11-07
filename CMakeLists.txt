cmake_minimum_required(VERSION 3.15)
project(simple_example)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)


add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS})