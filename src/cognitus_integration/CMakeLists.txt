cmake_minimum_required(VERSION 3.5)
project(cognitus_integration)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
