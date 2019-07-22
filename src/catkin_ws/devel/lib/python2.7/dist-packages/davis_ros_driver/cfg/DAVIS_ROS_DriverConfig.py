## *********************************************************
##
## File autogenerated for the davis_ros_driver package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'HARDWARE_FILTERS', 'lower': 'hardware_filters', 'srcline': 124, 'name': 'Hardware_Filters', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::HARDWARE_FILTERS', 'field': 'DEFAULT::hardware_filters', 'state': True, 'parentclass': 'DEFAULT', 'groups': [{'upper': 'DVS_REGION_OF_INTEREST', 'lower': 'dvs_region_of_interest', 'srcline': 124, 'name': 'DVS_region_of_interest', 'parent': 1, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Hardware_Filters', 'class': 'DEFAULT::HARDWARE_FILTERS::DVS_REGION_OF_INTEREST', 'field': 'DEFAULT::HARDWARE_FILTERS::dvs_region_of_interest', 'state': True, 'parentclass': 'DEFAULT::HARDWARE_FILTERS', 'groups': [], 'parameters': [{'srcline': 38, 'description': 'start position on the X axis for DVS region of interest', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'roi_start_column', 'edit_method': '', 'default': 0, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 39, 'description': 'start position on the Y axis for DVS region of interest', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'roi_start_row', 'edit_method': '', 'default': 0, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 40, 'description': 'end position on the X axis for DVS region of interest', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'roi_end_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 41, 'description': 'end position on the Y axis for DVS region of interest', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'roi_end_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}], 'type': '', 'id': 2}, {'upper': 'APS_REGION_OF_INTEREST', 'lower': 'aps_region_of_interest', 'srcline': 124, 'name': 'APS_region_of_interest', 'parent': 1, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Hardware_Filters', 'class': 'DEFAULT::HARDWARE_FILTERS::APS_REGION_OF_INTEREST', 'field': 'DEFAULT::HARDWARE_FILTERS::aps_region_of_interest', 'state': True, 'parentclass': 'DEFAULT::HARDWARE_FILTERS', 'groups': [], 'parameters': [{'srcline': 43, 'description': 'start position on the X axis for APS region of interest', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'aps_roi_start_column', 'edit_method': '', 'default': 0, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 44, 'description': 'start position on the Y axis for APS region of interest', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'aps_roi_start_row', 'edit_method': '', 'default': 0, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 45, 'description': 'end position on the X axis for APS region of interest', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'aps_roi_end_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 46, 'description': 'end position on the Y axis for APS region of interest', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'aps_roi_end_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}], 'type': '', 'id': 3}, {'upper': 'PIXEL_FILTER', 'lower': 'pixel_filter', 'srcline': 124, 'name': 'Pixel_Filter', 'parent': 1, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Hardware_Filters', 'class': 'DEFAULT::HARDWARE_FILTERS::PIXEL_FILTER', 'field': 'DEFAULT::HARDWARE_FILTERS::pixel_filter', 'state': True, 'parentclass': 'DEFAULT::HARDWARE_FILTERS', 'groups': [], 'parameters': [{'srcline': 48, 'description': 'pixel 0, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_0_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 49, 'description': 'pixel 0, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_0_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 50, 'description': 'pixel 1, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_1_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 51, 'description': 'pixel 1, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_1_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 52, 'description': 'pixel 2, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_2_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 53, 'description': 'pixel 2, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_2_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 54, 'description': 'pixel 3, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_3_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 55, 'description': 'pixel 3, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_3_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 56, 'description': 'pixel 4, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_4_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 57, 'description': 'pixel 4, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_4_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 58, 'description': 'pixel 5, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_5_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 59, 'description': 'pixel 5, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_5_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 60, 'description': 'pixel 6, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_6_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 61, 'description': 'pixel 6, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_6_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 62, 'description': 'pixel 3, Y axis setting', 'max': 259, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_7_row', 'edit_method': '', 'default': 259, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 63, 'description': 'pixel 7, X axis setting', 'max': 345, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_7_column', 'edit_method': '', 'default': 345, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 64, 'description': 'enable automatic filtering of 8 most active pixels (above ~5KHz)', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'pixel_auto_train', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}], 'type': '', 'id': 4}], 'parameters': [{'srcline': 28, 'description': 'enables background activity filter', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'background_activity_filter_enabled', 'edit_method': '', 'default': True, 'level': 1, 'min': False, 'type': 'bool'}, {'srcline': 29, 'description': 'background activity filter time (in blocks of 250 microseconds).', 'max': 400, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'background_activity_filter_time', 'edit_method': '', 'default': 80, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 30, 'description': 'enables refractory period filter', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'refractory_period_enabled', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}, {'srcline': 31, 'description': 'refractory period time (in blocks of 250 microseconds)', 'max': 20, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'refractory_period_time', 'edit_method': '', 'default': 2, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 32, 'description': 'enables the event skip filter, which throws away one event every N events (decimation filter)', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'skip_enabled', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}, {'srcline': 33, 'description': 'number of events to let through before skipping one', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'skip_every', 'edit_method': '', 'default': 0, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 34, 'description': 'suppress one of the two ON/OFF polarities completely', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'polarity_suppress', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}, {'srcline': 35, 'description': 'polarity to suppress (0=OFF, 1=ON)', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'polarity_suppress_type', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}, {'srcline': 36, 'description': 'flatten all polarities to OFF; applied after supression', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'polarity_flatten', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}], 'type': '', 'id': 1}, {'upper': 'DAVIS_BIASES_STAGE_1', 'lower': 'davis_biases_stage_1', 'srcline': 124, 'name': 'DAVIS_Biases_Stage_1', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::DAVIS_BIASES_STAGE_1', 'field': 'DEFAULT::davis_biases_stage_1', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 77, 'description': 'Amplifier in first stage. Limits the speed. Higher=faster and more noise (coarse, logarithmic)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'PrBp_coarse', 'edit_method': '', 'default': 2, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 78, 'description': 'Amplifier in first stage. Limits the speed. Higher=faster and more noise (fine, linear)', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'PrBp_fine', 'edit_method': '', 'default': 58, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 77, 'description': 'Source follower: set high to allow higher pixel bandwidth (has no influence if set high enough) (coarse, logarithmic)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'PrSFBp_coarse', 'edit_method': '', 'default': 1, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 78, 'description': 'Source follower: set high to allow higher pixel bandwidth (has no influence if set high enough) (fine, linear)', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'PrSFBp_fine', 'edit_method': '', 'default': 33, 'level': 1, 'min': 0, 'type': 'int'}], 'type': '', 'id': 5}, {'upper': 'DAVIS_BIASES_STAGE_2', 'lower': 'davis_biases_stage_2', 'srcline': 124, 'name': 'DAVIS_Biases_Stage_2', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::DAVIS_BIASES_STAGE_2', 'field': 'DEFAULT::davis_biases_stage_2', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 77, 'description': 'Amplifier of second stage. Indep of actual illumination level. rebalance ONBn/OFFBn after changing this (coarse, logarithmic)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'DiffBn_coarse', 'edit_method': '', 'default': 4, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 78, 'description': 'Amplifier of second stage. Indep of actual illumination level. rebalance ONBn/OFFBn after changing this (fine, linear)', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'DiffBn_fine', 'edit_method': '', 'default': 39, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 77, 'description': 'Threshold for ON events (contrast sensitivity). Must be >DiffBn. (coarse, logarithmic)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'ONBn_coarse', 'edit_method': '', 'default': 6, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 78, 'description': 'Threshold for ON events (contrast sensitivity). Must be >DiffBn. (fine, linear)', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'ONBn_fine', 'edit_method': '', 'default': 200, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 77, 'description': 'Threshold for OFF events (contrast sensitivity). Must be <DiffBn. (coarse, logarithmic)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'OFFBn_coarse', 'edit_method': '', 'default': 4, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 78, 'description': 'Threshold for OFF events (contrast sensitivity). Must be <DiffBn. (fine, linear)', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'OFFBn_fine', 'edit_method': '', 'default': 0, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 77, 'description': 'Maximum pixel firing rate (reset time before it can start to detect changes again) (coarse, logarithmic)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'RefrBp_coarse', 'edit_method': '', 'default': 4, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 78, 'description': 'Maximum pixel firing rate (reset time before it can start to detect changes again) (fine, linear)', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'RefrBp_fine', 'edit_method': '', 'default': 25, 'level': 1, 'min': 0, 'type': 'int'}], 'type': '', 'id': 6}, {'upper': 'DAVIS_BIASES_APS', 'lower': 'davis_biases_aps', 'srcline': 124, 'name': 'DAVIS_Biases_APS', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::DAVIS_BIASES_APS', 'field': 'DEFAULT::davis_biases_aps', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 83, 'description': 'APS ADC upper conversion limit (voltage)', 'max': 63, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'ADC_RefHigh_volt', 'edit_method': '', 'default': 27, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 84, 'description': 'APS ADC upper conversion limit (current)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'ADC_RefHigh_curr', 'edit_method': '', 'default': 7, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 83, 'description': 'APS ADC lower conversion limit (voltage)', 'max': 63, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'ADC_RefLow_volt', 'edit_method': '', 'default': 1, 'level': 1, 'min': 0, 'type': 'int'}, {'srcline': 84, 'description': 'APS ADC lower conversion limit (current)', 'max': 7, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'name': 'ADC_RefLow_curr', 'edit_method': '', 'default': 7, 'level': 1, 'min': 0, 'type': 'int'}], 'type': '', 'id': 7}], 'parameters': [{'srcline': 291, 'description': 'integration time (0 is instant sending)', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'streaming_rate', 'edit_method': '', 'default': 30, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'max. events per packet (0 is no limit)', 'max': 100000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'max_events', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'enables APS', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'aps_enabled', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'enables DVS', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'dvs_enabled', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'enables IMU', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'imu_enabled', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'enables the RPG auto-exposure algorithm (experimental)', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'autoexposure_enabled', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'auto-exposure gain (lower: slow response time, without oscillations / higher: faster response time, with oscillations', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'autoexposure_gain', 'edit_method': '', 'default': 3, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 291, 'description': 'desired mean image intensity: controls the overall image brightness', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'autoexposure_desired_intensity', 'edit_method': '', 'default': 75, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'exposure (frame exposure time in microseconds)', 'max': 1000000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'exposure', 'edit_method': '', 'default': 5000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'Desired type of frame output', 'max': 2, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'frame_mode', 'edit_method': "{'enum_description': 'Desired type of frame output', 'enum': [{'srcline': 19, 'description': 'Grayscale for mono cameras and RGB for color cameras', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Default'}, {'srcline': 20, 'description': 'Always a grayscale intensity frame', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Grayscale'}, {'srcline': 21, 'description': 'Exactly as it comes from the device (will show grid pattern on color cameras)', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Original'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'frame interval (best-effort time between consecutive frames in microseconds)', 'max': 8388607, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'frame_interval', 'edit_method': '', 'default': 25000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'Full scale of the accelerometer', 'max': 3, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'imu_acc_scale', 'edit_method': "{'enum_description': 'Full scale of the accelerometer', 'enum': [{'srcline': 106, 'description': '+/- 2 g', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': '2g'}, {'srcline': 107, 'description': '+/- 4 g', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': '4g'}, {'srcline': 108, 'description': '+/- 8 g', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': '8g'}, {'srcline': 109, 'description': '+/- 16 g', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': '16g'}]}", 'default': 3, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'Full scale of the gyroscope', 'max': 3, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'imu_gyro_scale', 'edit_method': "{'enum_description': 'Full scale of the gyroscope', 'enum': [{'srcline': 112, 'description': '+/- 250 deg/s', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': '250degps'}, {'srcline': 113, 'description': '+/- 500 deg/s', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': '500degps'}, {'srcline': 114, 'description': '+/- 1000 deg/s', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': '1000degps'}, {'srcline': 115, 'description': '+/- 2000 deg/s', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': '2000degps'}]}", 'default': 3, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'Bandwidth of the low-pass filter applied to the accelerometer/gyroscope', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'imu_low_pass_filter', 'edit_method': "{'enum_description': 'Bandwidth of the low pass filter applied to the accelerometer/gyroscope', 'enum': [{'srcline': 118, 'description': '260/256 Hz - delay: 0.0/0.98 ms', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': '260hz'}, {'srcline': 119, 'description': '184/188 Hz - delay: 2.0/1.9 ms', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': '184hz'}, {'srcline': 120, 'description': '94/98 Hz - delay: 3.0/2.8 ms', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': '94hz'}, {'srcline': 121, 'description': '44/42 Hz - delay: 4.9/4.8 ms', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': '44hz'}, {'srcline': 122, 'description': '21/20 Hz - delay: 8.5/8.3 ms', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': '21hz'}, {'srcline': 123, 'description': '10/10 Hz - delay: 13.8/13.4 ms', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 5, 'ctype': 'int', 'type': 'int', 'name': '10hz'}, {'srcline': 124, 'description': '5/5 Hz - delay: 19.0/18.6 ms', 'srcfile': '/home/egronda/sunfest/src/catkin_ws/src/rpg_dvs_ros/davis_ros_driver/cfg/DAVIS_ROS_Driver.cfg', 'cconsttype': 'const int', 'value': 6, 'ctype': 'int', 'type': 'int', 'name': '5hz'}]}", 'default': 1, 'level': 0, 'min': 0, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

DAVIS_ROS_Driver_Default = 0
DAVIS_ROS_Driver_Grayscale = 1
DAVIS_ROS_Driver_Original = 2
DAVIS_ROS_Driver_2g = 0
DAVIS_ROS_Driver_4g = 1
DAVIS_ROS_Driver_8g = 2
DAVIS_ROS_Driver_16g = 3
DAVIS_ROS_Driver_250degps = 0
DAVIS_ROS_Driver_500degps = 1
DAVIS_ROS_Driver_1000degps = 2
DAVIS_ROS_Driver_2000degps = 3
DAVIS_ROS_Driver_260hz = 0
DAVIS_ROS_Driver_184hz = 1
DAVIS_ROS_Driver_94hz = 2
DAVIS_ROS_Driver_44hz = 3
DAVIS_ROS_Driver_21hz = 4
DAVIS_ROS_Driver_10hz = 5
DAVIS_ROS_Driver_5hz = 6
