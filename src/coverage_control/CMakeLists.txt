cmake_minimum_required(VERSION 3.8) 
project(coverage_control) 
 
find_package(ament_cmake REQUIRED) 
find_package(rclpy REQUIRED) 
 
ament_python_install_package(${PROJECT_NAME}) 
 
install(PROGRAMS 
  coverage_control/adaptive_controller.py 
  DESTINATION lib/${PROJECT_NAME} 
) 
 
ament_package()
