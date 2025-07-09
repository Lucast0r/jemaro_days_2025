cmake_minimum_required(VERSION 3.5)
project(jemaro)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS script/jemaro_path.py
  DESTINATION lib/${PROJECT_NAME})
  
install(PROGRAMS script/turn_left.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS script/path_planner.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
