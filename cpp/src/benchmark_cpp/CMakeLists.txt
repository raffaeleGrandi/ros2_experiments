cmake_minimum_required(VERSION 3.5)

project(benchmark_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable target)
  add_executable(${target} ${target}/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "std_msgs"
    "rcutils")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(sender)
custom_executable(receiver)

ament_package()