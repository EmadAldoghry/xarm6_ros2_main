# Description of the data structure used by xarm_ros2


## msg
- [xarm6_msgs::msg::RobotMsg](./msg/RobotMsg.msg)
    - xarm6_api->topic: __robot_states__
- [xarm6_msgs::msg::CIOState](./msg/CIOState.msg)
    - xarm6_api->topic: __xarm_cgpio_states__
- [sensor_msgs::msg::JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)
    - xarm6_api->topic: __joint_states__

## srv

- [xarm6_msgs::srv::Call](./srv/Call.srv)
    - xarm6_api->service: __clean_error__
    - xarm6_api->service: __clean_warn__
    - xarm6_api->service: __clean_conf__
    - xarm6_api->service: __save_conf__
    - xarm6_api->service: __reload_dynamics__
    - xarm6_api->service: __set_counter_reset__
    - xarm6_api->service: __set_counter_increase__
    - xarm6_api->service: __clean_gripper_error__
    - xarm6_api->service: __clean_bio_gripper_error__
    - xarm6_api->service: __start_record_trajectory__
    - xarm6_api->service: __stop_record_trajectory__
    - xarm6_api->service: __ft_sensor_set_zero__
    - xarm6_api->service: __set_linear_track_stop__
    - xarm6_api->service: __clean_linear_track_error__
    - xarm6_api->service: __open_lite6_gripper__
    - xarm6_api->service: __close_lite6_gripper__
    - xarm6_api->service: __stop_lite6_gripper__

- [xarm6_msgs::srv::GetInt16](./srv/GetInt16.srv)
    - xarm6_api->service: __get_state__
    - xarm6_api->service: __get_cmdnum__
    - xarm6_api->service: __get_vacuum_gripper__
    - xarm6_api->service: __get_gripper_err_code__
    - xarm6_api->service: __get_bio_gripper_status__
    - xarm6_api->service: __get_bio_gripper_error__
    - xarm6_api->service: __get_reduced_mode__
    - xarm6_api->service: __get_report_tau_or_i__
    - xarm6_api->service: __ft_sensor_app_get__
    - xarm6_api->service: __get_ft_sensor_error__
    - xarm6_api->service: __get_trajectory_rw_status__
    - xarm6_api->service: __get_linear_track_pos__
    - xarm6_api->service: __get_linear_track_status__
    - xarm6_api->service: __get_linear_track_error__
    - xarm6_api->service: __get_linear_track_is_enabled__
    - xarm6_api->service: __get_linear_track_on_zero__
    - xarm6_api->service: __get_linear_track_sci__

- [xarm6_msgs::srv::GetInt16List](./srv/GetInt16List.srv)
    - xarm6_api->service: __get_err_warn_code__
    - xarm6_api->service: __get_linear_track_sco__

- [xarm6_msgs::srv::SetInt16](./srv/SetInt16.srv)
    - xarm6_api->service: __set_mode__
    - xarm6_api->service: __set_state__
    - xarm6_api->service: __set_collision_sensitivity__
    - xarm6_api->service: __set_teach_sensitivity__
    - xarm6_api->service: __set_gripper_mode__
    - xarm6_api->service: __set_gripper_enable__
    - xarm6_api->service: __set_bio_gripper_speed__
    - xarm6_api->service: __set_fence_mode__
    - xarm6_api->service: __set_reduced_mode__
    - xarm6_api->service: __set_self_collision_detection__
    - xarm6_api->service: __set_simulation_robot__
    - xarm6_api->service: __set_baud_checkset_enable__
    - xarm6_api->service: __set_report_tau_or_i__
    - xarm6_api->service: __ft_sensor_enable__
    - xarm6_api->service: __ft_sensor_app_set__
    - xarm6_api->service: __set_linear_track_enable__
    - xarm6_api->service: __set_linear_track_speed__
    - xarm6_api->service: __set_cartesian_velo_continuous__
    - xarm6_api->service: __set_allow_approx_motion__
    - xarm6_api->service: __set_only_check_type__
    - xarm6_api->service: __config_tgpio_reset_when_stop__
    - xarm6_api->service: __config_cgpio_reset_when_stop__

- [xarm6_msgs::srv::SetInt16ById](./srv/SetInt16ById.srv)
    - xarm6_api->service: __motion_enable__
    - xarm6_api->service: __set_servo_attach__
    - xarm6_api->service: __set_servo_detach__

- [xarm6_msgs::srv::SetInt16List](./srv/SetInt16List.srv)
    - xarm6_api->service: __set_reduced_tcp_boundary__

- [xarm6_msgs::srv::GetInt32](./srv/GetInt32.srv)
    - xarm6_api->service: __get_tgpio_modbus_baudrate__

- [xarm6_msgs::srv::SetInt32](./srv/SetInt32.srv)
    - xarm6_api->service: __set_tgpio_modbus_baudrate__

- [xarm6_msgs::srv::GetFloat32](./srv/GetFloat32.srv)
    - xarm6_api->service: __get_gripper_position__

- [xarm6_msgs::srv::GetFloat32List](./srv/GetFloat32List.srv)
    - xarm6_api->service: __get_position__
    - xarm6_api->service: __get_servo_angle__
    - xarm6_api->service: __get_position_aa__
    - xarm6_api->service: __get_ft_sensor_data__

- [xarm6_msgs::srv::SetFloat32](./srv/SetFloat32.srv)
    - xarm6_api->service: __set_pause_time__
    - xarm6_api->service: __set_tcp_jerk__
    - xarm6_api->service: __set_tcp_maxacc__
    - xarm6_api->service: __set_joint_jerk__
    - xarm6_api->service: __set_joint_maxacc__
    - xarm6_api->service: __set_gripper_speed__
    - xarm6_api->service: __set_reduced_max_tcp_speed__
    - xarm6_api->service: __set_reduced_max_joint_speed__

- [xarm6_msgs::srv::SetFloat32List](./srv/SetFloat32List.srv)
    - xarm6_api->service: __set_gravity_direction__
    - xarm6_api->service: __set_tcp_offset__
    - xarm6_api->service: __set_world_offset__
    - xarm6_api->service: __set_reduced_joint_range__

- [xarm6_msgs::srv::SetTcpLoad](./srv/SetTcpLoad.srv)
    - xarm6_api->service: __set_tcp_load__

- [xarm6_msgs::srv::MoveCartesian](./srv/MoveCartesian.srv)
    - xarm6_api->service: __set_position__
    - xarm6_api->service: __set_tool_position__
    - xarm6_api->service: __set_position_aa__
    - xarm6_api->service: __set_servo_cartesian__
    - xarm6_api->service: __set_servo_cartesian_aa__

- [xarm6_msgs::srv::MoveJoint](./srv/MoveJoint.srv)
    - xarm6_api->service: __set_servo_angle__
    - xarm6_api->service: __set_servo_angle_j__

- [xarm6_msgs::srv::MoveCircle](./srv/MoveCircle.srv)
    - xarm6_api->service: __move_circle__

- [xarm6_msgs::srv::MoveHome](./srv/MoveHome.srv)
    - xarm6_api->service: __move_gohome__

- [xarm6_msgs::srv::MoveVelocity](./srv/MoveVelocity.srv)
    - xarm6_api->service: __vc_set_joint_velocity__
    - xarm6_api->service: __vc_set_cartesian_velocity__

- [xarm6_msgs::srv::GetDigitalIO](./srv/GetDigitalIO.srv)
    - xarm6_api->service: __get_tgpio_digital__
    - xarm6_api->service: __get_cgpio_digital__

- [xarm6_msgs::srv::GetAnalogIO](./srv/GetAnalogIO.srv)
    - xarm6_api->service: __get_tgpio_analog__
    - xarm6_api->service: __get_cgpio_analog__

- [xarm6_msgs::srv::SetDigitalIO](./srv/SetDigitalIO.srv)
    - xarm6_api->service: __set_tgpio_digital__
    - xarm6_api->service: __set_cgpio_digital__
    - xarm6_api->service: __set_tgpio_digital_with_xyz__
    - xarm6_api->service: __set_cgpio_digital_with_xyz__

- [xarm6_msgs::srv::SetAnalogIO](./srv/SetAnalogIO.srv)
    - xarm6_api->service: __set_cgpio_analog__
    - xarm6_api->service: __set_cgpio_analog_with_xyz__

- [xarm6_msgs::srv::VacuumGripperCtrl](./srv/VacuumGripperCtrl.srv)
    - xarm6_api->service: __set_vacuum_gripper__

- [xarm6_msgs::srv::GripperMove](./srv/GripperMove.srv)
    - xarm6_api->service: __set_gripper_position__

- [xarm6_msgs::srv::BioGripperEnable](./srv/BioGripperEnable.srv)
    - xarm6_api->service: __set_bio_gripper_enable__

- [xarm6_msgs::srv::BioGripperCtrl](./srv/BioGripperCtrl.srv)
    - xarm6_api->service: __open_bio_gripper__
    - xarm6_api->service: __close_bio_gripper__

- [xarm6_msgs::srv::RobotiqReset](./srv/RobotiqReset.srv)
    - xarm6_api->service: __robotiq_reset__

- [xarm6_msgs::srv::RobotiqActivate](./srv/RobotiqActivate.srv)
    - xarm6_api->service: __robotiq_set_activate__

- [xarm6_msgs::srv::RobotiqMove](./srv/RobotiqMove.srv)
    - xarm6_api->service: __robotiq_set_position__
    - xarm6_api->service: __robotiq_open__
    - xarm6_api->service: __robotiq_close__

- [xarm6_msgs::srv::RobotiqGetStatus](./srv/RobotiqGetStatus.srv)
    - xarm6_api->service: __robotiq_get_status__

- [xarm6_msgs::srv::SetModbusTimeout](./srv/SetModbusTimeout.srv)
    - xarm6_api->service: __set_tgpio_modbus_timeout__

- [xarm6_msgs::srv::GetSetModbusData](./srv/GetSetModbusData.srv)
    - xarm6_api->service: __getset_tgpio_modbus_data__

- [xarm6_msgs::srv::TrajCtrl](./srv/TrajCtrl.srv)
    - xarm6_api->service: __save_record_trajectory__
    - xarm6_api->service: __load_trajectory__

- [xarm6_msgs::srv::TrajPlay](./srv/TrajPlay.srv)
    - xarm6_api->service: __playback_trajectory__

- [xarm6_msgs::srv::IdenLoad](./srv/IdenLoad.srv)
    - xarm6_api->service: __iden_tcp_load__
    - xarm6_api->service: __ft_sensor_iden_load__

- [xarm6_msgs::srv::FtCaliLoad](./srv/FtCaliLoad.srv)
    - xarm6_api->service: __ft_sensor_cali_load__

- [xarm6_msgs::srv::FtForceConfig](./srv/FtForceConfig.srv)
    - xarm6_api->service: __config_force_control__

- [xarm6_msgs::srv::FtForcePid](./srv/FtForcePid.srv)
    - xarm6_api->service: __set_force_control_pid__

- [xarm6_msgs::srv::FtImpedance](./srv/FtImpedance.srv)
    - xarm6_api->service: __set_impedance__
    - xarm6_api->service: __set_impedance_mbk__
    - xarm6_api->service: __set_impedance_config__

- [xarm6_msgs::srv::LinearTrackBackOrigin](./srv/LinearTrackBackOrigin.srv)
    - xarm6_api->service: __set_linear_track_back_origin__

- [xarm6_msgs::srv::LinearTrackSetPos](./srv/LinearTrackSetPos.srv)
    - xarm6_api->service: __set_linear_track_pos__

- [xarm6_msgs::srv::PlanPose](./srv/PlanPose.srv)
    - xarm_planner->service: __xarm_pose_plan__
- [xarm6_msgs::srv::PlanJoint](./srv/PlanJoint.srv)
    - xarm_planner->service: __xarm_joint_plan__
- [xarm6_msgs::srv::PlanSingleStraight](./srv/PlanSingleStraight.srv)
    - xarm_planner->service: __xarm_straight_plan__
- [xarm6_msgs::srv::PlanExec](./srv/PlanExec.srv)
    - xarm_planner->service: __xarm_exec_plan__


## action
- [control_msgs::action::GripperCommand](http://docs.ros.org/en/api/control_msgs/html/action/GripperCommand.html)
    - - xarm6_api->action: __xarm_gripper/gripper_action__

