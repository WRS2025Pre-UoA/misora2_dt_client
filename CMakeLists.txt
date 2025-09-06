cmake_minimum_required(VERSION 3.8)
project(misora2_dt_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(misora2_custom_msg REQUIRED)

include_directories(
  include
)

set(dependencies
    rclcpp
    rclcpp_components
    cv_bridge
    OpenCV
    sensor_msgs
    std_msgs
    misora2_custom_msg
)

#add_library(dt_client_library SHARED src/dt_client_component.cpp src/gui_tool.cpp)
#ament_target_dependencies(dt_client_library ${dependencies})
#rclcpp_components_register_nodes(dt_client_library "dt_client_component::DTClient")

#add_executable(dt_client_node src/dt_client_component.cpp src/dt_client_node.cpp src/gui_tool.cpp)
#ament_target_dependencies(dt_client_node ${dependencies})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  #dt_client_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  #dt_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/client_node_value.py
  scripts/client_node_pos.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
