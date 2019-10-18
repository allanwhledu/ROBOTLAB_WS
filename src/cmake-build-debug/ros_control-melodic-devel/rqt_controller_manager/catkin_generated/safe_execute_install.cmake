execute_process(COMMAND "/home/whl/ROBOTLAB_WS/src/cmake-build-debug/ros_control-melodic-devel/rqt_controller_manager/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/whl/ROBOTLAB_WS/src/cmake-build-debug/ros_control-melodic-devel/rqt_controller_manager/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
