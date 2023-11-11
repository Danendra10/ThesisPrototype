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

add_executable(positioning src/positioning.cpp)
target_link_libraries(positioning ${OpenCV_LIBS})

add_executable(machine_state src/machine_state.cpp)
target_link_libraries(machine_state ${OpenCV_LIBS})

add_executable(line_circle_interception src/line_circle_interception.cpp)
target_link_libraries(line_circle_interception ${OpenCV_LIBS})