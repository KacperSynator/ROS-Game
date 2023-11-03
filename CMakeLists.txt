cmake_minimum_required(VERSION 3.12)
project(ros_game CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Curses REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_game REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectData.msg"
 )

set_target_properties(ros_game__rosidl_generator_c PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(ros_game__rosidl_typesupport_introspection_c PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(ros_game__rosidl_generator_py PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(ros_game__rosidl_typesupport_introspection_c__pyext PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(ros_game__rosidl_typesupport_fastrtps_c__pyext PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(ros_game__rosidl_typesupport_c__pyext PROPERTIES LINKER_LANGUAGE CXX)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(keyboard_reader src/keyboard_reader.cpp)
ament_target_dependencies(keyboard_reader rclcpp std_msgs ros_game)
target_link_libraries(keyboard_reader ${CURSES_LIBRARIES})

add_executable(game_controller src/game_controller.cpp)
ament_target_dependencies(game_controller rclcpp std_msgs ros_game)

add_executable(game_display src/game_display.cpp)
ament_target_dependencies(game_display rclcpp std_msgs ros_game)
target_link_libraries(game_display ${SDL2_LIBRARIES})

install(TARGETS keyboard_reader game_display game_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
