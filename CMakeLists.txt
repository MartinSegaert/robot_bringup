cmake_minimum_required(VERSION 3.10)
project(robot_bringup)

# Detect ROS version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  # Fallback detection
  if(DEFINED ENV{ROS_DISTRO})
    if($ENV{ROS_DISTRO} STREQUAL "noetic" OR 
       $ENV{ROS_DISTRO} STREQUAL "melodic" OR 
       $ENV{ROS_DISTRO} STREQUAL "kinetic")
      set(ROS_VERSION 1)
    else()
      set(ROS_VERSION 2)
    endif()
  endif()
endif()

if(ROS_VERSION EQUAL 1)
  # ROS1 Build
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    )

    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES ros1_launcher
    #  CATKIN_DEPENDS roscpp rospy
    #  DEPENDS system_lib
    )

    include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    )

else()
  # ROS2 Build
    # find dependencies
    find_package(ament_cmake REQUIRED)

    # Install launch files
    install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
    )

    # Install config files
    install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
    )

    # Install Gazebo resources used by custom simulation launch files.
    install(DIRECTORY
    gz
    DESTINATION share/${PROJECT_NAME}/
    )

    file(GLOB PY_SCRIPTS scripts/ros2/*.py)
    install(PROGRAMS
        ${PY_SCRIPTS}
        DESTINATION lib/${PROJECT_NAME}
    )

    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
endif()
