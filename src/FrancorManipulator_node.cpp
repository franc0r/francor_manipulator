
#include "FrancorManipulator_node.h"

FrancorManipulator_node::FrancorManipulator_node()
{
  //rosParam
  ros::NodeHandle privNh("~");
  // std::string string_val;
  double axix0_home;
  double axis1_home;
  double axis2_home;
  double axis0_active;
  double axis1_active;
  double axis2_active;
  // int int_val;
  // bool bool_val;


  // privNh.param(         "string_val" ,    string_val,   std::string("string"));
  privNh.param<double>( "axix0_home" ,      axix0_home,   100.0);
  privNh.param<double>( "axis1_home" ,      axis1_home,   100.0);
  privNh.param<double>( "axis2_home" ,      axis2_home,   100.0);
  privNh.param<double>( "axis0_active" ,    axis0_active,   100.0);
  privNh.param<double>( "axis1_active" ,    axis1_active,   100.0);
  privNh.param<double>( "axis2_active" ,    axis2_active,   100.0);


  // privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);

  _axix0_home     = axix0_home;
  _axis1_home     = axis1_home;
  _axis2_home     = axis2_home;
  _axis0_active   = axis0_active;
  _axis1_active   = axis1_active;
  _axis2_active   = axis2_active;


  // //init publisher
  _pub_pos_axis0       = _nh.advertise<std_msgs::UInt16>( "manipulator/set_pos/axis0", 1);
  _pub_pos_axis1       = _nh.advertise<std_msgs::UInt16>( "manipulator/set_pos/axis1", 1);
  _pub_pos_axis2       = _nh.advertise<std_msgs::UInt16>( "manipulator/set_pos/axis2", 1);
  _pub_head_gripper    = _nh.advertise<std_msgs::UInt16>( "manipulator/set_pos/gripper", 1);
  _pub_pos_head_pan    = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_pan/pos", 1);
  _pub_pos_head_tilt   = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_tilt/pos", 1);
  _pub_ros_head_roll   = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_roll/pos", 1);
  _pub_speed_head_pan  = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_pan/speed", 1);
  _pub_speed_head_tilt = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_tilt/speed", 1);
  _pub_speed_head_roll = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_roll/speed", 1);
  _pub_pos             = _nh.advertise<francor_msgs::ManipulatorCmd>("manipulator/pos", 1);
  _pub_status          = _nh.advertise<std_msgs::String>( "manipulator/status", 1);

  //inti subscriber
  _sub_pos_axis0                 = _nh.subscribe("manipulator/pos/axis0",     1, &FrancorManipulator_node::subPosAxis0_callback, this);
  _sub_pos_axis1                 = _nh.subscribe("manipulator/pos/axis1",     1, &FrancorManipulator_node::subPosAxis1_callback, this);
  _sub_pos_axis2                 = _nh.subscribe("manipulator/pos/axis2",     1, &FrancorManipulator_node::subPosAxis2_callback, this);
  _sub_speed_manipulator_axis    = _nh.subscribe("manipulator/speed/axis",    1, &FrancorManipulator_node::subSpeedManipulatorAxis_callback, this);
  _sub_speed_manipulator_inverse = _nh.subscribe("manipulator/speed/inverse", 1, &FrancorManipulator_node::subSpeedManipulatorInverse_callback, this);
  _sub_pos_manipulator_inverse   = _nh.subscribe("manipulator/pos/inverse",   1, &FrancorManipulator_node::subPosManipulatorInverse_callback, this);

  _srv_set_mode_axis    = _nh.advertiseService("manipulator/set/mode/axis",    &FrancorManipulator_node::srv_set_mode_axis, this);
  _srv_set_mode_inverse = _nh.advertiseService("manipulator/set/mode/inverse", &FrancorManipulator_node::srv_set_mode_inverse, this);
  _srv_set_stand_by     = _nh.advertiseService("manipulator/set/stand_by",     &FrancorManipulator_node::srv_set_stand_by, this);
  _srv_set_active       = _nh.advertiseService("manipulator/set/active",       &FrancorManipulator_node::srv_set_active, this);

}

FrancorManipulator_node::~FrancorManipulator_node()
{
}

void FrancorManipulator_node::start(double duration)
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(duration), &FrancorManipulator_node::loop_callback, this);
  this->run();
}

void FrancorManipulator_node::run()
{
  ros::spin();
}

void FrancorManipulator_node::loop_callback(const ros::TimerEvent& e)
{
  // ROS_ERROR("Got man pos: %s", this->got_manipulator_pos() ? "true" : "false");

  if((ros::Time::now() - _timeLastPosAxix0).toSec() > 1.0)
  {
    _got_pos_axis0 = false;
  }
  if((ros::Time::now() - _timeLastPosAxix1).toSec() > 1.0)
  {
    _got_pos_axis1 = false;
  }
  if((ros::Time::now() - _timeLastPosAxix2).toSec() > 1.0)
  {
    _got_pos_axis2 = false;
  }

  if(this->got_manipulator_pos() && _mode == HOMING)
  {
    return;
  }


  if(this->got_manipulator_pos() && _mode == ACTIVATING)
  {
    return;
  }


  if(this->got_manipulator_pos() && _mode == AXIS)
  {
    //base
    _desired_angle_axis0 += _angle_increment_max * _curr_manip_cmd.joint_0;
    _desired_angle_axis1 += _angle_increment_max * _curr_manip_cmd.joint_1;
    _desired_angle_axis2 += _angle_increment_max * _curr_manip_cmd.joint_2;

    _desired_angle_axis0 = constrain(_desired_angle_axis0, _SERVO_RAD_MIN, _SERVO_RAD_MAX);
    _desired_angle_axis1 = constrain(_desired_angle_axis1, 0, _SERVO_RAD_MAX);
    _desired_angle_axis2 = constrain(_desired_angle_axis2, _SERVO_RAD_MIN, 1.52);

    std_msgs::UInt16 msg;

    msg.data = this->rad_to_servo_ms(_desired_angle_axis0);
    // if(msg.data < 1400) //todo  doooo beeeettteeerrrS
    // {
    //   msg.data = 1400;
    // }
    _pub_pos_axis0.publish(msg);
    msg.data = this->rad_to_servo_ms(_desired_angle_axis1);
    _pub_pos_axis1.publish(msg);
    msg.data = this->rad_to_servo_ms(_desired_angle_axis2);
    _pub_pos_axis2.publish(msg);
    
    
    francor_msgs::ManipulatorCmd pos;
    pos.joint_0 = _desired_angle_axis0;
    pos.joint_1 = _desired_angle_axis1;
    pos.joint_2 = _desired_angle_axis2;

    _pub_pos.publish(pos);
  }

  if(this->got_manipulator_pos() && _mode == INVERSE)
  {

  }


  //head
  //gripper
  _desired_pos_gripper += static_cast<int16_t>(std::round(_ms_gripper_increment_max * _curr_manip_cmd.head_gripper));
  _desired_pos_gripper = static_cast<uint16_t>(constrain(_desired_pos_gripper, 500.0, 2140.0));
  std_msgs::UInt16 msg_gripper;
  msg_gripper.data = _desired_pos_gripper;
  _pub_head_gripper.publish(msg_gripper);
  //pan tilt roll
  std_msgs::Float64 msg_servo;
  msg_servo.data = _curr_manip_cmd.head_pan;
  _pub_speed_head_pan.publish(msg_servo);
  msg_servo.data = _curr_manip_cmd.head_tilt;
  _pub_speed_head_tilt.publish(msg_servo);
  msg_servo.data = _curr_manip_cmd.head_roll;
  _pub_speed_head_roll.publish(msg_servo);


}


void FrancorManipulator_node::subPosAxis0_callback(const std_msgs::UInt16& msg) 
{
  if(msg.data == 65535)
  {//servo has no power
    _got_pos_axis0 = false;
    return;
  }
  _timeLastPosAxix0 = ros::Time::now();

  _current_angle_axis0 = this->servo_ms_to_rad(msg.data);
  if(!_got_pos_axis0)// || (std::abs(_desired_angle_axis0 - _current_angle_axis0) > _SERVO_SAVETY_OFFSET && _mode == AXIS))
  {
    _desired_angle_axis0 = _current_angle_axis0;
  }
  _got_pos_axis0 = true;
}

void FrancorManipulator_node::subPosAxis1_callback(const std_msgs::UInt16& msg) 
{
  if(msg.data == 65535)
  {//servo has no power
    _got_pos_axis1 = false;
    return;
  }
  _timeLastPosAxix1 = ros::Time::now();

  _current_angle_axis1 = this->servo_ms_to_rad(msg.data);
  if(!_got_pos_axis1)// || (std::abs(_desired_angle_axis1 - _current_angle_axis1) > _SERVO_SAVETY_OFFSET && _mode == AXIS))
  {
    _desired_angle_axis1 = _current_angle_axis1;
  }
  _got_pos_axis1 = true;
}

void FrancorManipulator_node::subPosAxis2_callback(const std_msgs::UInt16& msg) 
{
  if(msg.data == 65535)
  {//servo has no power
    _got_pos_axis2 = false;
    return;
  }
  _timeLastPosAxix2 = ros::Time::now();

  _current_angle_axis2 = this->servo_ms_to_rad(msg.data);
  if(!_got_pos_axis2)// || (std::abs(_desired_angle_axis1 - _current_angle_axis1) > _SERVO_SAVETY_OFFSET && _mode == AXIS))
  {
    _desired_angle_axis2 = _current_angle_axis2;
  }
  _got_pos_axis2 = true;
}

void FrancorManipulator_node::subSpeedManipulatorAxis_callback(const francor_msgs::ManipulatorCmd& msg) 
{
  _curr_manip_cmd = msg;
}

void FrancorManipulator_node::subSpeedManipulatorInverse_callback(const geometry_msgs::Vector3& msg) 
{
  
}

void FrancorManipulator_node::subPosManipulatorInverse_callback(const geometry_msgs::Point& msg) 
{
  
}



bool FrancorManipulator_node::srv_set_mode_axis(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) 
{
  _mode = AXIS;
  return true;
}

bool FrancorManipulator_node::srv_set_mode_inverse(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) 
{
  _mode = INVERSE;
  return true;
}

bool FrancorManipulator_node::srv_set_stand_by(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) 
{
  ROS_ERROR("SET MODE STANDBY");
  _mode = HOMING;
  return true;
}

bool FrancorManipulator_node::srv_set_active(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) 
{
  ROS_ERROR("SET MODE ACTIVE");
  _mode = ACTIVATING;
  return true;
}





// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "francor_manipulator_node");
  ros::NodeHandle nh("~");

  FrancorManipulator_node node;
  node.start(0.01);
}
