
#include "FrancorManipulator_node.h"

FrancorManipulator_node::FrancorManipulator_node()
{
  //rosParam
  ros::NodeHandle privNh("~");
  // std::string string_val;
  // double double_val;
  // int int_val;
  // bool bool_val;


  // privNh.param(         "string_val" ,    string_val,   std::string("string"));
  // privNh.param<double>( "double_val" ,    double_val,   100.0);
  // privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);

  // //init publisher
  _pub_pos_axis1       = _nh.advertise<std_msgs::UInt16>( "manipulator/set_pos/axis1", 1);
  _pub_pos_axis2       = _nh.advertise<std_msgs::UInt16>( "manipulator/set_pos/axis2", 1);
  _pub_pos_head_pan    = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_pam/pos", 1);
  _pub_pos_head_tilt   = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_tilt/pos", 1);
  _pub_ros_head_roll   = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_roll/pos", 1);
  _pub_speed_head_pan  = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_pam/speed", 1);
  _pub_speed_head_tilt = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_tilt/speed", 1);
  _pub_speed_head_roll = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_roll/speed", 1);
  _pub_status          = _nh.advertise<std_msgs::String>( "manipulator/status", 1);

  //inti subscriber
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
  if(this->got_manipulator_pos() && _mode == AXIS)
  {
    //base
    _desired_angle_axis1 += _angle_increment_max * _curr_manip_cmd.joint_1;
    _desired_angle_axis2 += _angle_increment_max * _curr_manip_cmd.joint_2;

    _desired_angle_axis1 = constrain(_desired_angle_axis1, _SERVO_RAD_MIN, _SERVO_RAD_MAX);
    _desired_angle_axis2 = constrain(_desired_angle_axis2, _SERVO_RAD_MIN, _SERVO_RAD_MAX);

    std_msgs::UInt16 msg;
    msg.data = this->rad_to_servo_ms(_desired_angle_axis1);
    _pub_pos_axis1.publish(msg);
    msg.data = this->rad_to_servo_ms(_desired_angle_axis2);
    _pub_pos_axis2.publish(msg);
    //head
    std_msgs::Float64 msg_servo;
    msg_servo.data = _curr_manip_cmd.head_pan;
    _pub_speed_head_pan.publish(msg_servo);
    msg_servo.data = _curr_manip_cmd.head_tilt;
    _pub_speed_head_tilt.publish(msg_servo);
    msg_servo.data = _curr_manip_cmd.head_roll;
    _pub_speed_head_roll.publish(msg_servo);
  }

}

void FrancorManipulator_node::subPosAxis1_callback(const std_msgs::UInt16& msg) 
{
  _current_angle_axis1 = this->servo_ms_to_rad(msg.data);
  if(!_got_pos_axis1)
  {
    _desired_angle_axis1 = _current_angle_axis1;
  }
  _got_pos_axis1 = true;
}

void FrancorManipulator_node::subPosAxis2_callback(const std_msgs::UInt16& msg) 
{
  _current_angle_axis2 = this->servo_ms_to_rad(msg.data);
  if(!_got_pos_axis2)
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
  
}

bool FrancorManipulator_node::srv_set_active(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) 
{
  
}





// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "francor_manipulator_node");
  ros::NodeHandle nh("~");

  FrancorManipulator_node node;
  node.start(0.01);
}
