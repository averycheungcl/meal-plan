cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
foreach(dir config launch meshes urdf srdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()