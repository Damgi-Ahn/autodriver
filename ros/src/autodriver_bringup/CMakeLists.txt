cmake_minimum_required(VERSION 3.14)
project(autodriver_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
