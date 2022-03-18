cmake_minimum_required(VERSION 3.5)
project(point_cloud_registering)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL 1.10 REQUIRED common io)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

endif()

#find_package(pcl)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# point_cloud_talker
add_executable(point_cloud_talker src/publisher_member_function.cpp)
ament_target_dependencies(point_cloud_talker rclcpp sensor_msgs std_msgs PCL)
install(TARGETS
point_cloud_talker
DESTINATION lib/${PROJECT_NAME}
)

# [Linker]>[Input]>[Additional Dependencies]
target_link_libraries( point_cloud_talker ${PCL_LIBRARIES} )




# pcd_reader
add_executable(pcd_reader src/pcd_read.cpp)
ament_target_dependencies(pcd_reader PCL
sensor_msgs
 rclcpp)
install(TARGETS
pcd_reader
DESTINATION lib/${PROJECT_NAME}
)

# [Linker]>[Input]>[Additional Dependencies]
target_link_libraries( pcd_reader ${PCL_LIBRARIES} )



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
