execute_process(COMMAND "/home/whl/ROBOTLAB_WS/src/cmake-build-debug/turtlebot3-melodic-devel/turtlebot3_teleop/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/whl/ROBOTLAB_WS/src/cmake-build-debug/turtlebot3-melodic-devel/turtlebot3_teleop/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
