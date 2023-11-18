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

add_executable(main_game src/main_game.cpp)
target_link_libraries(main_game ${OpenCV_LIBS})

add_executable(test src/test.cpp)
target_link_libraries(test ${OpenCV_LIBS})

add_executable(static_game src/static_game.cpp)
target_link_libraries(static_game ${OpenCV_LIBS})

add_executable(convert_matlab_to_cpp src/final/convert_matlab_to_cpp.cpp)
target_link_libraries(convert_matlab_to_cpp ${OpenCV_LIBS})

add_executable(attack_single src/final/attack_single.cpp)
target_link_libraries(attack_single ${OpenCV_LIBS})

add_executable(attack_double src/final/attack_double.cpp)
target_link_libraries(attack_double ${OpenCV_LIBS})

add_executable(robot_motion src/final/robot_motion.cpp)
target_link_libraries(robot_motion ${OpenCV_LIBS})

add_executable(defend_single src/final/defend_single.cpp)
target_link_libraries(defend_single ${OpenCV_LIBS})