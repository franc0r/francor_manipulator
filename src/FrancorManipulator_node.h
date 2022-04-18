
#ifndef FRANCORMANIPULATOR_NODE_H_
#define FRANCORMANIPULATOR_NODE_H_

#include <iostream>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <francor_base/angle.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <std_srvs/Empty.h>

//francor
#include <francor_msgs/ManipulatorCmd.h>


class FrancorManipulator_node
{

public:
  FrancorManipulator_node();
  virtual ~FrancorManipulator_node();

  /**
     *
     * @brief
     *
     * @return  void
     */
  void start(double duration = 0.01);

  enum ENUM_MODE{
    AXIS = 0,
    INVERSE,
    HOMING,
    ACTIVATING
  };

private: //functions

  francor::base::NormalizedAngleExtended servo_ms_to_rad(const uint16_t servo_us)
  {
    // todo
    return rescale(static_cast<double>(servo_us), 1000.0, 2000.0, _SERVO_RAD_MIN, _SERVO_RAD_MAX);
  }

  uint16_t rad_to_servo_ms(const francor::base::NormalizedAngleExtended rad)
  {
    //todo
    return static_cast<uint16_t>(std::round(rescale(rad, _SERVO_RAD_MIN, _SERVO_RAD_MAX, 1000.0, 2000.0)));
  }

  bool got_manipulator_pos()
  {
    return _got_pos_axis1 && _got_pos_axis2 && _got_pos_axis0;
  }

  /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
  void run();

  void loop_callback(const ros::TimerEvent& e);

  //topics
  void subPosAxis0_callback(const std_msgs::UInt16& msg);
  void subPosAxis1_callback(const std_msgs::UInt16& msg);
  void subPosAxis2_callback(const std_msgs::UInt16& msg);
  void subSpeedManipulatorAxis_callback(const francor_msgs::ManipulatorCmd& msg);
  void subSpeedManipulatorInverse_callback(const geometry_msgs::Vector3& msg);
  void subPosManipulatorInverse_callback(const geometry_msgs::Point& msg);

  //srvs
  bool srv_set_mode_axis(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srv_set_mode_inverse(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srv_set_stand_by(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srv_set_active(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);


  //a la arduino
  static inline double rescale(const double x, const double in_min, const double in_max, const double out_min, const double out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static inline double constrain(const double vel, const double low, const double high)
  {
    return vel < low ? low : (vel > high ? high : vel);
  } 

private: //dataelements
  ros::NodeHandle _nh;

  //base
  ros::Publisher _pub_pos_axis0;
  ros::Publisher _pub_pos_axis1;
  ros::Publisher _pub_pos_axis2;
  //head
  ros::Publisher _pub_head_gripper;
  ros::Publisher _pub_pos_head_pan;
  ros::Publisher _pub_pos_head_tilt;
  ros::Publisher _pub_ros_head_roll;
  ros::Publisher _pub_speed_head_pan;
  ros::Publisher _pub_speed_head_tilt;
  ros::Publisher _pub_speed_head_roll;

  ros::Publisher _pub_pos;

  ros::Publisher _pub_status;

  //return from base
  ros::Subscriber _sub_pos_axis0;
  ros::Subscriber _sub_pos_axis1;
  ros::Subscriber _sub_pos_axis2;
  //sub arm cmd
  ros::Subscriber _sub_speed_manipulator_axis; //each axis individual or head only
  // ros::Subscriber _sub_pos_manipulator_axis; //todo check if needed or only standby or active pos
  ros::Subscriber _sub_speed_manipulator_inverse; //inverse until head
  ros::Subscriber _sub_pos_manipulator_inverse;

  //srv
  ros::ServiceServer _srv_set_mode_axis;
  ros::ServiceServer _srv_set_mode_inverse;
  ros::ServiceServer _srv_set_stand_by;
  ros::ServiceServer _srv_set_active;

  ENUM_MODE _mode = AXIS;

  francor::base::NormalizedAngleExtended _desired_angle_axis0;
  francor::base::NormalizedAngleExtended _desired_angle_axis1;
  francor::base::NormalizedAngleExtended _desired_angle_axis2;

  francor::base::NormalizedAngleExtended _current_angle_axis0;
  francor::base::NormalizedAngleExtended _current_angle_axis1;
  francor::base::NormalizedAngleExtended _current_angle_axis2;

  double _axix0_home;
  double _axis1_home;
  double _axis2_home;
  double _axis0_active;
  double _axis1_active;
  double _axis2_active;

  const double _angle_increment_max = 0.008;

  bool _got_pos_axis0 = false;
  bool _got_pos_axis1 = false;
  bool _got_pos_axis2 = false;

  uint16_t _desired_pos_gripper = 1500;
  const double _ms_gripper_increment_max = 30.0;

  francor_msgs::ManipulatorCmd _curr_manip_cmd;

  ros::Timer _loopTimer;

  ros::Time _timeLastPosAxix0;
  ros::Time _timeLastPosAxix1;
  ros::Time _timeLastPosAxix2;
  

  static constexpr double _SERVO_RAD_MIN = -2.61799;
  static constexpr double _SERVO_RAD_MAX = 2.61799;

  static constexpr double _SERVO_SAVETY_OFFSET = 0.4;

};

#endif  //FRANCORMANIPULATOR_NODE_H_
