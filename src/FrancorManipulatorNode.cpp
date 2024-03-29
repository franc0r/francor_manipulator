#include "francor_manipulator/FrancorManipulatorNode.hpp"



FrancorManipulatorNode::FrancorManipulatorNode() : rclcpp::Node("francor_manipulator_node")
{
    this->declare_parameter<double>     ("rate_loop",     _params.rate_loop);
    this->declare_parameter<double>     ("rate_heartbeat", _params.rate_heartbeat);

    _params.rate_loop   = this->get_parameter("rate_loop").as_double();
    _params.rate_heartbeat = this->get_parameter("rate_heartbeat").as_double();


    RCLCPP_INFO(_logger, "------------------------------------------------------");
    RCLCPP_INFO(_logger, "-------    Parameter, THI_Joy2Vel_Node      ----------");
    RCLCPP_INFO(_logger, "------------------------------------------------------");
    RCLCPP_INFO(_logger, "rate_loop:     %f", _params.rate_loop);
    RCLCPP_INFO(_logger, "rate_heartbeat:%f", _params.rate_heartbeat);
    RCLCPP_INFO(_logger, "------------------------------------------------------");
    RCLCPP_INFO(_logger, "------------------------------------------------------");

    //clock
    _clock = this->get_clock();
    //set time 
    _base_axis_time_last_pos.axis_0 = _clock->now();
    _base_axis_time_last_pos.axis_1 = _clock->now();
    _base_axis_time_last_pos.axis_2 = _clock->now();


    //create pubs
    _pub_head_pos_pan     = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_pan/pos", 10);
    _pub_head_pos_tilt    = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_tilt/pos", 10);
    _pub_head_pos_roll    = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_roll/pos", 10);
    _pub_head_speed_pan   = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_pan/speed", 10);
    _pub_head_speed_tilt  = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_tilt/speed", 10);
    _pub_head_speed_roll  = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_roll/speed", 10);
    _pub_sh_pitch         = this->create_publisher<std_msgs::msg::Float64>("/servo_lx16a/sensor_head_pitch/pos", 10);
    _pub_sh_yaw           = this->create_publisher<std_msgs::msg::Float64>("/servo_lx16a/sensor_head_yaw/pos", 10);
    _pub_status       = this->create_publisher<std_msgs::msg::String>("manipulator/status", 10);
    _pub_status_pos   = this->create_publisher<francor_msgs::msg::ManipulatorCmd>("manipulator/status/pos", 10);
    //pub firmware
    _pub_set_pos_axis = this->create_publisher<std_msgs::msg::Int32MultiArray>("manipulator/set_pos/axis", rclcpp::QoS(10).best_effort());
    _pub_heartbeat    = this->create_publisher<std_msgs::msg::UInt8>("manipulator/heartbeat", rclcpp::QoS(10).best_effort());
    _pub_enable       = this->create_publisher<std_msgs::msg::UInt8>("manipulator/enable", rclcpp::QoS(10).best_effort());


    //crate subs
    _sub_cmd_axis_speed    = this->create_subscription<francor_msgs::msg::ManipulatorCmd>("manipulator/speed/axis", 10, std::bind(&FrancorManipulatorNode::sub_cmd_axis_speed_callback, this, std::placeholders::_1));
    // _sub_cmd_axis_pos   = this->create_subscription<francor_msgs::msg::ManipulatorCmd>("manipulator/pos/axis", 10, std::bind(&FrancorManipulatorNode::sub_cmd_axis_pos_callback, this, std::placeholders::_1));
    _sub_cmd_inverse_speed = this->create_subscription<geometry_msgs::msg::Vector3>("manipulator/speed/inverse", 10, std::bind(&FrancorManipulatorNode::sub_cmd_inverse_callback, this, std::placeholders::_1));
    _sub_axis_pos          = this->create_subscription<std_msgs::msg::Int32MultiArray>("manipulator/pos/axis", rclcpp::QoS(10).best_effort(), std::bind(&FrancorManipulatorNode::sub_pos_axis_callback, this, std::placeholders::_1));
    _sub_axis_state        = this->create_subscription<std_msgs::msg::UInt8MultiArray>("manipulator/pos/status", rclcpp::QoS(10).best_effort(), std::bind(&FrancorManipulatorNode::sub_state_axis_callback, this, std::placeholders::_1));

    _sub_head_pan    = this->create_subscription<francor_msgs::msg::ServoLx16a>("/servo_lx16a/arm_pan/status", 10, std::bind(&FrancorManipulatorNode::sub_pan_callback, this, std::placeholders::_1));
    _sub_head_tilt   = this->create_subscription<francor_msgs::msg::ServoLx16a>("/servo_lx16a/arm_tilt/status", 10, std::bind(&FrancorManipulatorNode::sub_tilt_callback, this, std::placeholders::_1));
    _sub_head_roll   = this->create_subscription<francor_msgs::msg::ServoLx16a>("/servo_lx16a/arm_roll/status", 10, std::bind(&FrancorManipulatorNode::sub_roll_callback, this, std::placeholders::_1));

    //create srvs
    _srv_set_mode_inverse = this->create_service<std_srvs::srv::Empty>("manipulator/set/mode/inverse", std::bind(&FrancorManipulatorNode::srv_set_mode_inverse_callback, this, std::placeholders::_1, std::placeholders::_2));
    _srv_set_mode_axis    = this->create_service<std_srvs::srv::Empty>("manipulator/set/mode/axis", std::bind(&FrancorManipulatorNode::srv_set_mode_axis_callback, this, std::placeholders::_1, std::placeholders::_2));
    _srv_set_mode_active  = this->create_service<std_srvs::srv::Empty>("manipulator/set/mode/active", std::bind(&FrancorManipulatorNode::srv_set_mode_active_callback, this, std::placeholders::_1, std::placeholders::_2));
    _srv_set_mode_standby = this->create_service<std_srvs::srv::Empty>("manipulator/set/mode/standby", std::bind(&FrancorManipulatorNode::srv_set_mode_standby_callback, this, std::placeholders::_1, std::placeholders::_2));
    _srv_set_init_enable  = this->create_service<std_srvs::srv::Empty>("manipulator/set/init_enable", std::bind(&FrancorManipulatorNode::srv_set_init_enable_callback, this, std::placeholders::_1, std::placeholders::_2));
}

// FrancorManipulatorNode::~FrancorManipulatorNode()
// { }

void FrancorManipulatorNode::init()
{

  //create timers
  _timer_loop = this->create_wall_timer(std::chrono::duration<double>(1/_params.rate_loop), std::bind(&FrancorManipulatorNode::timer_loop_callback, this));
  _timer_heartbeat = this->create_wall_timer(std::chrono::duration<double>(1/_params.rate_heartbeat), std::bind(&FrancorManipulatorNode::timer_heartbeat_callback, this));
}

void FrancorManipulatorNode::timer_loop_callback()
{
  //check last time from pos from base
  this->update_pos_flags();
  //pub mode
  std_msgs::msg::String msg_mode;
  msg_mode.data = FrancorManipulatorNode::enum_mode_to_str(_mode);
  _pub_status->publish(msg_mode);
  
  //pub status pos
  francor_msgs::msg::ManipulatorCmd msg_status_pos;
  msg_status_pos.joint_0 = _base_axis_pos.axis_0;
  msg_status_pos.joint_1 = _base_axis_pos.axis_1;
  msg_status_pos.joint_2 = _base_axis_pos.axis_2;
  msg_status_pos.joint_3 = 0.0;
  _pub_status_pos->publish(msg_status_pos);

  //pub enable
  std_msgs::msg::UInt8 msg_enable;
  if(_mode == ENM_MODE::IDLE || _mode == ENM_MODE::STANDBY)
  {
    msg_enable.data = 0;
  }
  else
  {
    msg_enable.data = 1;
  }
  _pub_enable->publish(msg_enable);

  //check power and valid pos -> else go into IDLE
  if((!this->base_manipulator_has_pwr() || !this->got_manipulator_pos() ) && _mode != ENM_MODE::IDLE)
  {
    RCLCPP_WARN(this->get_logger(), "No Power or No valid pos while beeing not in IDLE -> switch to IDLE");
    this->set_mode(ENM_MODE::IDLE);
  }

  if(_mode == ENM_MODE::IDLE)
  {
    //wait for valid pos and pwr commands, then set mode to standby
    // RCLCPP_INFO(this->get_logger(), "got_pos: %s",  this->base_manipulator_has_pwr() ? "true" : "false");
    // RCLCPP_INFO(this->get_logger(), "got_pos: %s",  this->got_manipulator_pos() ? "true" : "false");
    if(this->base_manipulator_has_pwr() && this->got_manipulator_pos())
    {
      RCLCPP_INFO(this->get_logger(), "Got valid pos and pwr -> switch to standby");
      this->set_mode(ENM_MODE::STANDBY);
    }
    return; //todo check if ok
  }

  if(_mode == ENM_MODE::STANDBY)
  {
    //do nothing //wait for activating command

  }

  if(_mode == ENM_MODE::AXIS)
  {
    if(_selected_mode == ENM_MODE::INVERSE)
    {
      this->set_mode(ENM_MODE::INVERSE);
      return;
    }
    //publish pos to base and head
    _desired_base_axis_pos.axis_0 += _params.angle_increment_speed * _last_cmd_axis_speed->joint_0;
    _desired_base_axis_pos.axis_1 += _params.angle_increment_speed * _last_cmd_axis_speed->joint_1;
    _desired_base_axis_pos.axis_2 += _params.angle_increment_speed * _last_cmd_axis_speed->joint_2;
    
    _desired_base_axis_pos.axis_0 = constrain(_desired_base_axis_pos.axis_0, _SERVO_MAX_RAD * -1, _SERVO_MAX_RAD);
    _desired_base_axis_pos.axis_1 = constrain(_desired_base_axis_pos.axis_1, _SERVO_MAX_RAD * -1, _SERVO_MAX_RAD);
    _desired_base_axis_pos.axis_2 = constrain(_desired_base_axis_pos.axis_2, _SERVO_MAX_RAD * -1, _SERVO_MAX_RAD);

    // //compute forward kinematics
    // auto pos_tcp = ManipulatorSimpleInverse::compute2DForward(_desired_base_axis_pos.axis_1, _desired_base_axis_pos.axis_2);
    // RCLCPP_INFO(this->get_logger(), "pos_tcp: %f, %f", pos_tcp.x, pos_tcp.z);

    // // dummy inverse:
    // auto inverse = ManipulatorSimpleInverse::compute2DInverse(pos_tcp.x, pos_tcp.z);
    // RCLCPP_INFO(this->get_logger(), "inverse: %f, %f", inverse.x, inverse.z);

    // RCLCPP_INFO(this->get_logger(), "axis0_desired: %f, increment: %f, speed_cmd: %f", (float)_desired_base_axis_pos.axis_0, (float)_params.angle_increment_speed, (float)_last_cmd_axis_speed->joint_0);
    // RCLCPP_INFO(this->get_logger(), "axis1_desired: %f, increment: %f, speed_cmd: %f", (float)_desired_base_axis_pos.axis_1, (float)_params.angle_increment_speed, (float)_last_cmd_axis_speed->joint_1);
    // RCLCPP_INFO(this->get_logger(), "axis2_desired: %f, increment: %f, speed_cmd: %f", (float)_desired_base_axis_pos.axis_2, (float)_params.angle_increment_speed, (float)_last_cmd_axis_speed->joint_2);
    this->set_gripper_speed(_last_cmd_axis_speed->head_gripper);
    this->send_pos_to_base(_desired_base_axis_pos);

    //send pos to head
    this->send_speed_to_head(Manipulator_head_axis<double>(*_last_cmd_axis_speed));
  }

  if(_mode == ENM_MODE::INVERSE)
  {
    if(_selected_mode == ENM_MODE::AXIS)
    {
      this->set_mode(ENM_MODE::AXIS);
      return;
    } 


    //manipulate desierd _inverse_pos
    
    const double l0 = 0.38;
    const double l1 = 0.245;

    const double l_all = (l0 + l1) * 0.95;

    double x_diff = _params.angle_increment_speed * 0.1 * _last_cmd_inverse_speed->x;
    double z_diff = _params.angle_increment_speed * 0.1 * _last_cmd_inverse_speed->z;


    _target_base_axis_pos.axis_0 =  _target_base_axis_pos.axis_0 + _params.angle_increment_speed * 0.5 * _last_cmd_inverse_speed->y;
    
    double x_extra_length = (_desired_inverse_pos.x / std::cos(_desired_base_axis_pos.axis_0)) - _desired_inverse_pos.x;

    // RCLCPP_INFO(this->get_logger(), "x_extra_length: %f", (float)x_extra_length);
    Eigen::Vector2d tmp_pos(_desired_inverse_pos.x + x_diff + x_extra_length, _desired_inverse_pos.z + z_diff);

    if(tmp_pos.norm() < l_all)
    {
      _desired_inverse_pos.x += x_diff;
      // _desired_inverse_pos.y += _params.angle_increment_speed * _last_cmd_inverse_speed->y;
      _desired_inverse_pos.z += z_diff;
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "INVLAID POS");
      return;
    }





    // RCLCPP_INFO(this->get_logger(), "inverse_pos.x: %f, inverse.z: %f", (float)_desired_inverse_pos.x, (float)_desired_inverse_pos.z);

    auto axis_angle = ManipulatorSimpleInverse::compute2DInverse(_desired_inverse_pos.x + x_extra_length, _desired_inverse_pos.z);

    // if(axis_angle.x < -0.35 || axis_angle.x > 1.55 ||
    //    axis_angle.z < -0.65 || axis_angle.z > 2.0)
    // {//revert x and z diff if new angles are grater
    //   _desired_inverse_pos.x -= x_diff;
    //   _desired_inverse_pos.y -= z_diff;
    //   return;
    // }

    
    if(std::abs(axis_angle.x - _target_base_axis_pos.axis_1) > 0.4 ||
       std::abs(axis_angle.z - _target_base_axis_pos.axis_2) > 0.4)
    {
      RCLCPP_INFO(this->get_logger(), "angle to much stop inverse stuff");
      return;
    }

    _target_base_axis_pos.axis_1 = FrancorManipulatorNode::constrain(axis_angle.x, -0.4, 1.6);
    _target_base_axis_pos.axis_2 = FrancorManipulatorNode::constrain(axis_angle.z, -0.7, 2.2);

    this->set_gripper_speed(_last_cmd_axis_speed->head_gripper);
    this->move_base_to(_target_base_axis_pos, 0.5);
    //publish inversed calculated pos to base  and passing head speed commands as usual
    this->send_speed_to_head(Manipulator_head_axis<double>(*_last_cmd_axis_speed));
  }

  if(_mode == ENM_MODE::IS_ACTIVATING)
  {
    //send target pos command untill rdy and set selected mode afterwards
    //send safe pos to sh
    this->send_pos_to_sh(_params.sh_safe_pitch, _params.sh_safe_yaw);

    if(!this->move_base_to(_params.base_pos_active, _params.homing_speed, true))
    {
      return;
    }
    _gripper_pos = 500;
    if(this->move_head_to(_params.head_pos_active, 1.0, false))
    {
      this->set_mode(_selected_mode);
    }
  
  }

  if(_mode == ENM_MODE::IS_HOMING)
  {
    this->send_pos_to_sh(_params.sh_safe_pitch, _params.sh_safe_yaw);
    if(!this->move_head_to(_params.head_pos_standby, 1.0, false))
    {
      return;
    }

    _gripper_pos = 2000;

    //send target pos command untill rdy and set standby mode afterwards
    if(this->move_base_to(_params.base_pos_standby, _params.homing_speed))
    {//arrrived
      this->set_mode(ENM_MODE::STANDBY);
    }
  }

}


void FrancorManipulatorNode::sub_cmd_axis_speed_callback(const francor_msgs::msg::ManipulatorCmd::SharedPtr msg)
{
  // RCLCPP_INFO(_logger, "sub_cmd_axis_speed_callback");
  _last_cmd_axis_speed = msg;
}

void FrancorManipulatorNode::sub_cmd_inverse_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  // (void)msg;
  // RCLCPP_INFO(_logger, "sub_cmd_inverse_callback");
  _last_cmd_inverse_speed = msg;
}

//from firmware
void FrancorManipulatorNode::sub_pos_axis_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  // RCLCPP_INFO(_logger, "###sub_pos_axis_callback");
  if(msg->data.size() != 3)
  {
    RCLCPP_ERROR(_logger, "sub_pos_axis_callback: wrong size");
    return;
  }

  if(msg->data[0] > 0)
  {
    // RCLCPP_INFO(this->get_logger(), "sub_pos_axis_callback: %d", msg->data[0]);
    _base_axis_pos.axis_0 = this->servo_pos_to_rad(msg->data[0], 0);
    // RCLCPP_INFO(this->get_logger(), "servo_pos: %d, rad_pos: %f", msg->data[0], (float)_base_axis_pos.axis_0);
    _base_axis_got_pos.axis_0 = true;
    _base_axis_time_last_pos.axis_0 = _clock->now();
  }
  if(msg->data[1] > 0)
  {
    // RCLCPP_INFO(this->get_logger(), "sub_pos_axis_callback: %d", msg->data[1]);
    _base_axis_pos.axis_1 = this->servo_pos_to_rad(msg->data[1], 1);
    _base_axis_got_pos.axis_1 = true;
    _base_axis_time_last_pos.axis_1 = _clock->now();
  }
  if(msg->data[2] > 0)
  {
    // RCLCPP_INFO(this->get_logger(), "sub_pos_axis_callback: %d", msg->data[2]);
    _base_axis_pos.axis_2 = this->servo_pos_to_rad(msg->data[2], 2);
    _base_axis_got_pos.axis_2 = true;
    _base_axis_time_last_pos.axis_2 = _clock->now();
  }
  
}

//from firmware
//shity implementation!!!!
void FrancorManipulatorNode::sub_state_axis_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  // RCLCPP_INFO(_logger, "################################ callback");
  if(msg->data.size() != 3)
  {
    RCLCPP_ERROR(_logger, "sub_pos_axis_callback: wrong size");
    return;
  }
  
  //axis0 
  if(msg->data[0] & 0x01) //has power
  { 
    // RCLCPP_INFO(this->get_logger(), "axis0 has power");
    _base_axis_pwr_state.axis_0 = true;
  }
  else
  {
    // RCLCPP_INFO(this->get_logger(), "axis0 no power");
    _base_axis_pwr_state.axis_0 = false;
  }
  if(msg->data[0] & 0x02) //is enabled
  {
    // RCLCPP_INFO(this->get_logger(), "axis0 is enabled");
    _base_axis_active_state.axis_0 = true;
  }
  else
  {
    // RCLCPP_INFO(this->get_logger(), "axis0 is not enabled");
    _base_axis_active_state.axis_0 = false;
  }

  //axis1
  if(msg->data[1] & 0x01) //has power
  {
    // RCLCPP_INFO(this->get_logger(), "axis1 has power");
    _base_axis_pwr_state.axis_1 = true;
  }
  else
  {
    // RCLCPP_INFO(this->get_logger(), "axis1 no power");
    _base_axis_pwr_state.axis_1 = false;
  }
  if(msg->data[1] & 0x02) //is enabled
  {
    // RCLCPP_INFO(this->get_logger(), "axis1 is enabled");
    _base_axis_active_state.axis_1 = true;
  }
  else
  {
    // RCLCPP_INFO(this->get_logger(), "axis1 is not enabled");
    _base_axis_active_state.axis_1 = false;
  }

  //axis2
  if(msg->data[2] & 0x01) //has power
  {
    // RCLCPP_INFO(this->get_logger(), "axis2 has power");
    _base_axis_pwr_state.axis_2 = true;
  }
  else
  {
    // RCLCPP_INFO(this->get_logger(), "axis2 no power");
    _base_axis_pwr_state.axis_2 = false;
  }
  if(msg->data[2] & 0x02) //is enabled
  {
    // RCLCPP_INFO(this->get_logger(), "axis2 is enabled");
    _base_axis_active_state.axis_2 = true;
  }
  else
  {
    // RCLCPP_INFO(this->get_logger(), "axis2 is not enabled");
    _base_axis_active_state.axis_2 = false;
  }

}

void FrancorManipulatorNode::srv_set_mode_inverse_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(_logger, "srv_set_mode_inverse_callback");
  _selected_mode = ENM_MODE::INVERSE;
  // if(_mode == ENM_MODE::AXIS || _mode == ENM_MODE::STANDBY)
  // {
  //   this->set_mode(ENM_MODE::INVERSE);
  // }
  // else
  // {
  //   RCLCPP_ERROR(_logger, "srv_set_mode_inverse_callback: wrong mode");
  // }
}

void FrancorManipulatorNode::srv_set_mode_axis_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{ 
  (void)req;
  (void)res;
  RCLCPP_INFO(_logger, "srv_set_mode_axis_callback");
  _selected_mode = ENM_MODE::AXIS;
  // if(_mode == ENM_MODE::INVERSE || _mode == ENM_MODE::STANDBY)
  // {
  //   this->set_mode(ENM_MODE::AXIS);
  // }
  // else
  // {
  //   RCLCPP_ERROR(_logger, "srv_set_mode_axis_callback: wrong mode");
  // }
}

void FrancorManipulatorNode::srv_set_mode_active_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(_logger, "srv_set_mode_active_callback");
  if(_mode != ENM_MODE::IDLE)
  {
    this->set_mode(ENM_MODE::IS_ACTIVATING);
    // _target_base_axis_pos = _params.base_pos_active; //-> not needed here useing params ..
  }
}

void FrancorManipulatorNode::srv_set_mode_standby_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(_logger, "srv_set_mode_standby_callback");
  if(_mode == ENM_MODE::AXIS || _mode == ENM_MODE::INVERSE)
  {
    this->set_mode(ENM_MODE::IS_HOMING);
  }
}

void FrancorManipulatorNode::srv_set_init_enable_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(_logger, "srv_set_init_enable_callback");
  if(_mode == ENM_MODE::STANDBY)
  {
    this->set_mode(ENM_MODE::AXIS);

  }
}