cmake_minimum_required(VERSION 3.8)
project(sobang_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(uwb_driver REQUIRED)

# find external packages
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalState.msg"
  "msg/UwbData.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(sobang_navigation_lib SHARED
  src/cores/navigation.cpp
  src/cores/radarEstimator.cpp  
  src/cores/uwbLocalizer.cpp
  src/tools/navTools.cpp  
)

add_executable(sobang_navigation_node src/nodes/main.cpp)

ament_target_dependencies(sobang_navigation_node 
  rclcpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions  
  uwb_driver
  visualization_msgs
)

ament_target_dependencies(sobang_navigation_lib 
  rclcpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  uwb_driver
  visualization_msgs
 )

# 해당 노드에서 rosidl_generate_interfaces 로 생성된 메시지 타입을 사용하기 위해 typesupport_target 을 링크해야 함
target_link_libraries(sobang_navigation_node
  sobang_navigation_lib   
  ${typesupport_target}
  ${PCL_LIBRARIES}
)

# 해당 lib 에서도 typesupport_target 을 링크해야 rosidl_generate_interfaces 로 생성된 메시지 타입을 사용할 수 있음
target_link_libraries(sobang_navigation_lib
  ${typesupport_target}
)

# Install rules
install(TARGETS
  sobang_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sobang_navigation_lib
  EXPORT sobang_navigation_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY 
  include/
  DESTINATION include
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME})

# Export dependencies and interfaces for downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_targets(sobang_navigation_targets)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
