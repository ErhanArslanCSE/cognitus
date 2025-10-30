cmake_minimum_required(VERSION 3.5)
project(cognitus_perception)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/SpatialRelation.msg"
  "msg/SceneGraph.msg"
  "msg/AnomalyEvent.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
