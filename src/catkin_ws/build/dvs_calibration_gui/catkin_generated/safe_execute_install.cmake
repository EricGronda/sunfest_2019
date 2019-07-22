execute_process(COMMAND "/home/egronda/sunfest_2019/src/catkin_ws/build/dvs_calibration_gui/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/egronda/sunfest_2019/src/catkin_ws/build/dvs_calibration_gui/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
