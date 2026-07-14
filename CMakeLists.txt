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
    geometry_msgs
    planner_msgs
    roscpp
    rospy
    std_msgs
    std_srvs
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

    catkin_install_python(PROGRAMS
      scripts/ros1/waypoint_follower.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY config/ros1/waypoint_follower
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/ros1
    )

    install(FILES launch/ros1/waypoint_follower.launch
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/ros1
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
