execute_process(COMMAND "/home/bit/ROBOTLAB_WS/src/cmake-build-debug/gazebo_ros_pkgs/gazebo_ros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bit/ROBOTLAB_WS/src/cmake-build-debug/gazebo_ros_pkgs/gazebo_ros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()