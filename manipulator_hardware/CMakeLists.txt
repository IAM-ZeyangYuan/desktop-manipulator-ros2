cmake_minimum_required(VERSION 3.8)
project(manipulator_hardware)

# Use C++17 standard (required by ros2_control).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# build the source file as  a shared library (.so file)

add_library(${PROJECT_NAME} SHARED
  src/simulated_robot.cpp
)

# Tell the compiler where to find your header files.
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against the ROS2 libraries your code uses. (the find_package()above just makes sure the pkgs exist)
# ament_target_dependencies is a ROS2 convenience macro that
# handles both include paths and library linking.

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Tell pluginlib about your plugin description XML.
# This registers the plugin so controller_manager can find it.
pluginlib_export_plugin_description_file(
  hardware_interface manipulator_hardware_plugin.xml)

# Install the compiled library to the right location.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install your header files.
install(DIRECTORY include/
  DESTINATION include
)

# Export the package so other packages can depend on it.
ament_export_include_directories(include) #the location of the header file
ament_export_libraries(${PROJECT_NAME}) #this says, other packages that decalre a dependency 
#on this package should linke against the library that the package builds
#why: the package builds a library from the source code, other packages migh want to use
#the classes / functions in the package
#they can include the headers to declare so and so, but the implementation is in the library (.so)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
) #this says if someone depends on the package, they also implicitly depned on these packages




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
