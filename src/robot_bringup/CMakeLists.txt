# cmake_minimum_required(VERSION 3.8)
# project(robot_bringup)

# find_package(ament_cmake REQUIRED)
# find_package(rclpy REQUIRED)

# # Install launch files
# install(
#   DIRECTORY launch
#   # DIRECTORY worlds
  # DIRECTORY params
#   DIRECTORY rviz
  
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install launch files
install(DIRECTORY 
  launch
  worlds
  params
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()