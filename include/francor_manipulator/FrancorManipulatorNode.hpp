#ifndef FRANCORMANIPULATORNODE_H_
#define FRANCORMANIPULATORNODE_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <francor_msgs/msg/manipulator_cmd.hpp>
#include <francor_msgs/msg/servo_lx16a.hpp>

#include <std_srvs/srv/empty.hpp>


#include <francor_base/angle.h>

#include "francor_manipulator/ManipulatorSimpleInverse.hpp"


template<typename T>
struct Manipulator_base_axis
{
  T axis_0;
  T axis_1;
  T axis_2;
  Manipulator_base_axis(const T& axis_0, const T& axis_1, const T& axis_2)
  : axis_0(axis_0), axis_1(axis_1), axis_2(axis_2)
  {}
  // T axis_3;
};

template<typename T>
struct Manipulator_head_axis
{
  T pan;
  T tilt;
  T roll;
  Manipulator_head_axis(const T& pan, const T& tilt, const T& roll)
  : pan(pan), tilt(tilt), roll(roll)
  {}
  Manipulator_head_axis(const francor_msgs::msg::ManipulatorCmd& msg)
  { this->from(msg); }
  void from(const francor_msgs::msg::ManipulatorCmd& msg)
  {
    pan = msg.head_pan;
    tilt = msg.head_tilt;
    roll = msg.head_roll;
    //todo gripper
  }
};

enum class ENM_MODE
{
  IDLE = 0,
  STANDBY,
  AXIS,
  INVERSE,
  IS_HOMING,
  IS_ACTIVATING
};


class FrancorManipulatorNode : public rclcpp::Node
{
public:
  FrancorManipulatorNode();
  virtual ~FrancorManipulatorNode() = default;

  void init();

  void timer_heartbeat_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Heartbeat");
    std_msgs::msg::UInt8 msg;
    // msg.data = 1;
    _pub_heartbeat->publish(msg);
  }

  void timer_loop_callback();

private: //functions
  void send_pos_to_base(const Manipulator_base_axis<francor::base::AnglePiToPi>& pos) const
  {
    //todo constrain value!!!

    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(this->rad_to_servo_pos(pos.axis_0, 0));
    msg.data.push_back(this->rad_to_servo_pos(pos.axis_1, 1));
    msg.data.push_back(this->rad_to_servo_pos(pos.axis_2, 2));
    msg.data.push_back(_gripper_pos);
    _pub_set_pos_axis->publish(msg);
  }

  void send_pos_to_head(const Manipulator_head_axis<francor::base::AnglePiToPi>& pos) const
  {
    std_msgs::msg::Float64 msg;
    msg.data = pos.pan;
    _pub_head_pos_pan->publish(msg);
    msg.data = pos.tilt;
    _pub_head_pos_tilt->publish(msg);
    msg.data = pos.roll;
    _pub_head_pos_roll->publish(msg);
  }

  void send_speed_to_head(const Manipulator_head_axis<double>& speed) const
  {
    std_msgs::msg::Float64 msg;
    msg.data = speed.pan;
    _pub_head_speed_pan->publish(msg);
    msg.data = speed.tilt;
    _pub_head_speed_tilt->publish(msg);
    msg.data = speed.roll;
    _pub_head_speed_roll->publish(msg);
  }

  void send_pos_to_sh(const double pitch, const double yaw) const
  {
    std_msgs::msg::Float64 msg;
    msg.data = pitch;
    _pub_sh_pitch->publish(msg);
    msg.data = yaw;
    _pub_sh_yaw->publish(msg);
  }

  /**
   * @brief moves the base to given target pos 
   * 
   * @param pos target pos
   * @param speed speed
   * @param simultaneously if ture all axes are moved at the same tim, else axis 0 is moved first then axis 1 and so on
   * @return true if arrived
   * @return false if still moving
   */
  bool move_base_to(const Manipulator_base_axis<francor::base::AnglePiToPi>& t_pos, const double speed, const bool simultaneously = false)
  {
    double diff_0 = t_pos.axis_0 - _desired_base_axis_pos.axis_0;
    double diff_1 = t_pos.axis_1 - _desired_base_axis_pos.axis_1;
    double diff_2 = t_pos.axis_2 - _desired_base_axis_pos.axis_2;

    if(simultaneously)
    {
      bool rdy = true;
      if(std::abs(diff_0) > _params.angle_increment_speed * 1.1)
      {//axis 0
        _desired_base_axis_pos.axis_0 += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_0);
        rdy = false;
      }
      else
      {
        _desired_base_axis_pos.axis_0 = t_pos.axis_0;
      }

      if(std::abs(diff_1) > _params.angle_increment_speed * 1.1)
      {//home axis1
        _desired_base_axis_pos.axis_1 += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_1);
        rdy = false;
      }
      else
      {
        _desired_base_axis_pos.axis_1 = t_pos.axis_1;
      }
      if(std::abs(diff_2) > _params.angle_increment_speed * 1.1)
      {//home axis2
        _desired_base_axis_pos.axis_2 += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_2);
        rdy = false;
      }
      else
      {
        _desired_base_axis_pos.axis_2 = t_pos.axis_2;
      }

      if(rdy)
      {
        _desired_base_axis_pos.axis_0 = t_pos.axis_0;
        _desired_base_axis_pos.axis_1 = t_pos.axis_1;
        _desired_base_axis_pos.axis_2 = t_pos.axis_2;
        this->send_pos_to_base(_desired_base_axis_pos);
        return true;
      }
    }
    else
    {
      if(std::abs(diff_0) > _params.angle_increment_speed * 1.1)
      {//axis 0
        // RCLCPP_INFO(this->get_logger(), "moving asis0");
        _desired_base_axis_pos.axis_0 += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_0);
      }
      else if(std::abs(diff_1) > _params.angle_increment_speed * 1.1)
      {//home axis1
        // RCLCPP_INFO(this->get_logger(), "moving asis1");
        _desired_base_axis_pos.axis_0 = t_pos.axis_0;
        _desired_base_axis_pos.axis_1 += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_1);
      }
      else if(std::abs(diff_2) > _params.angle_increment_speed * 1.1)
      {//home axis2
        // RCLCPP_INFO(this->get_logger(), "moving asis2");
        _desired_base_axis_pos.axis_0 = t_pos.axis_0;
        _desired_base_axis_pos.axis_1 = t_pos.axis_1;
        _desired_base_axis_pos.axis_2 += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_2);
      }
      else
      {
        _desired_base_axis_pos.axis_0 = t_pos.axis_0;
        _desired_base_axis_pos.axis_1 = t_pos.axis_1;
        _desired_base_axis_pos.axis_2 = t_pos.axis_2;
        this->send_pos_to_base(_desired_base_axis_pos);
        return true;
      }
    }
    this->send_pos_to_base(_desired_base_axis_pos);
    return false;
  }

  bool move_head_to(const Manipulator_head_axis<francor::base::AnglePiToPi>& t_pos, const double speed, const bool simultaneously = false)
  {
    double diff_pan  = t_pos.pan  - _desired_head_axis_pos.pan;
    double diff_tilt = t_pos.tilt - _desired_head_axis_pos.tilt;
    double diff_roll = t_pos.roll - _desired_head_axis_pos.roll;

    if(simultaneously)
    {
      

    }
    else
    {
      if(std::abs(diff_pan) > _params.angle_increment_speed * 2.1)
      {//pan
        _desired_head_axis_pos.pan += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_pan);
      }
      else if(std::abs(diff_tilt) > _params.angle_increment_speed * 2.1)
      {
        _desired_head_axis_pos.pan = t_pos.pan;
        _desired_head_axis_pos.tilt += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_tilt);
      }
      else if(std::abs(diff_roll) > _params.angle_increment_speed * 2.1)
      {
        _desired_head_axis_pos.pan = t_pos.pan;
        _desired_head_axis_pos.tilt = t_pos.tilt;
        _desired_head_axis_pos.roll += _params.angle_increment_speed * speed * FrancorManipulatorNode::sgn(diff_roll);
      }
      else
      {
        _desired_head_axis_pos.pan = t_pos.pan;
        _desired_head_axis_pos.tilt = t_pos.tilt;
        _desired_head_axis_pos.roll = t_pos.roll;
        this->send_pos_to_head(_desired_head_axis_pos);
        // this->send_speed_to_head(_desired_head_axis_pos);
        // RCLCPP_INFO(this->get_logger(), "+++++++++++++++head pos reached");
        return true;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "head pos not reached");
    this->send_pos_to_head(_desired_head_axis_pos);
    return false;
  }


  void set_gripper_speed(const double speed)
  {
    _gripper_pos += static_cast<int32_t>(std::round(100.0 * speed));
    _gripper_pos = static_cast<int32_t>(FrancorManipulatorNode::constrain(_gripper_pos, 500.0, 2300.0)); //todo find min max value!!!
  }

  //subs
  void sub_cmd_axis_speed_callback(const francor_msgs::msg::ManipulatorCmd::SharedPtr msg);
  void sub_cmd_inverse_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  //sub firmware_base
  void sub_pos_axis_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void sub_state_axis_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  void sub_pan_callback(const francor_msgs::msg::ServoLx16a::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "PAN: %f", msg->pos);
    _last_head_pos.pan = msg->pos;
  }

  void sub_tilt_callback(const francor_msgs::msg::ServoLx16a::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "TILT: %f", msg->pos);
    _last_head_pos.tilt = msg->pos;
  }

  void sub_roll_callback(const francor_msgs::msg::ServoLx16a::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "ROLL: %f", msg->pos);
    _last_head_pos.roll = msg->pos;
  }

  //srv callbacks
  void srv_set_mode_inverse_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
  void srv_set_mode_axis_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
  void srv_set_mode_active_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
  void srv_set_mode_standby_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
  void srv_set_init_enable_callback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);

  //a la arduino
  static inline double rescale(const double x, const double in_min, const double in_max, const double out_min, const double out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static inline double constrain(const double vel, const double low, const double high)
  {
    return vel < low ? low : (vel > high ? high : vel);
  } 

  template <typename T> 
  int sgn(T val) 
  {
    return (T(0) < val) - (val < T(0));
  } 

  static std::string enum_mode_to_str(const ENM_MODE mode)
  {
    switch(mode)
    {
      case ENM_MODE::IDLE:
        return "IDLE";
      case ENM_MODE::STANDBY:
        return "STANDBY";
      case ENM_MODE::AXIS:
        return "AXIS";
      case ENM_MODE::INVERSE:
        return "INVERSE";
      case ENM_MODE::IS_HOMING:
        return "IS_HOMING";
      case ENM_MODE::IS_ACTIVATING:
        return "IS_ACTIVATING";
      default:
        return "UNKNOWN";
    }
  }

  void set_mode(const ENM_MODE mode)
  {
    if(mode == _mode)
    {
      return;
    }
    RCLCPP_INFO(_logger, "set mode from %s to  %s", this->enum_mode_to_str(_mode).c_str(), this->enum_mode_to_str(mode).c_str());
    _mode = mode;

    //call fuction for each mode transition
    switch(_mode)
    {
      case ENM_MODE::IDLE:
        break;
      case ENM_MODE::STANDBY:
        break;
      case ENM_MODE::AXIS:
        //set desired pos to curr pos
        _desired_base_axis_pos = _base_axis_pos;
        // RCLCPP_INFO(this->get_logger(), "############## AXIS MODE ##############");
        // RCLCPP_INFO(this->get_logger(), "axis0: %f, axis1: %f, axis2: %f", (float)_desired_base_axis_pos.axis_0, (float)_desired_base_axis_pos.axis_1, (float)_desired_base_axis_pos.axis_2);
        break;
      case ENM_MODE::INVERSE:
        {
          //set desired pos to curr pos
          _desired_base_axis_pos = _base_axis_pos;
          _target_base_axis_pos = _base_axis_pos;
          //init desired pos
          _desired_inverse_pos = ManipulatorSimpleInverse::compute2DForward(_desired_base_axis_pos.axis_1, _desired_base_axis_pos.axis_2);
          break;
        }
      case ENM_MODE::IS_HOMING:
        //set desired pos to curr pos
        _desired_base_axis_pos = _base_axis_pos;
        _desired_head_axis_pos = _last_head_pos;
        break;
      case ENM_MODE::IS_ACTIVATING:
        //set desired pos to curr pos
        _desired_base_axis_pos = _base_axis_pos;
        _desired_head_axis_pos = _last_head_pos;
        break;
      default:
        break;
    }
  }

  /**
   * @brief 
   * 
   * servo pos range: 30 .. 490   //todo for each servo???
   * @param rad 
   * @return int32_t 
   */
  int32_t rad_to_servo_pos(const francor::base::AnglePiToPi rad, const int id) const
  {
    int32_t pos = 0;
    switch(id) {
      case 0:
        pos = static_cast<int32_t>(std::round(FrancorManipulatorNode::rescale(rad, _SERVO_MAX_RAD, _SERVO_MAX_RAD * -1, _SERVO_MIN, _SERVO_MAX)));
        break;
      case 1:
        pos = static_cast<int32_t>(std::round(FrancorManipulatorNode::rescale(rad, _SERVO_MAX_RAD, _SERVO_MAX_RAD * -1, _SERVO_MIN, _SERVO_MAX)));
        break;
      case 2:
        pos = static_cast<int32_t>(std::round(FrancorManipulatorNode::rescale(rad, _SERVO_MAX_RAD * -1, _SERVO_MAX_RAD, _SERVO_MIN, _SERVO_MAX)));
        break;
      default:
        assert(false);
        break;
    }

    return pos;
  }

  francor::base::AnglePiToPi servo_pos_to_rad(const int32_t servo_pos, const int id) const
  {


    francor::base::AnglePiToPi pos = 0;
    switch(id) {
      case 0:
        pos = static_cast<double>(FrancorManipulatorNode::rescale(static_cast<double>(servo_pos), _SERVO_MIN, _SERVO_MAX, _SERVO_MAX_RAD, _SERVO_MAX_RAD * -1));
        break;
      case 1:
        pos = static_cast<double>(FrancorManipulatorNode::rescale(static_cast<double>(servo_pos), _SERVO_MIN, _SERVO_MAX, _SERVO_MAX_RAD, _SERVO_MAX_RAD * -1)); 
        break;
      case 2:
        pos = static_cast<double>(FrancorManipulatorNode::rescale(static_cast<double>(servo_pos), _SERVO_MIN, _SERVO_MAX, _SERVO_MAX_RAD * -1, _SERVO_MAX_RAD));
        break;
      default:
        assert(false);
        break;
    }

    return pos;
  }

  bool got_manipulator_pos() const
  {
    return _base_axis_got_pos.axis_0 && _base_axis_got_pos.axis_1 && _base_axis_got_pos.axis_2;
  }

  bool base_manipulator_active() const
  {
    return _base_axis_active_state.axis_0 && _base_axis_active_state.axis_1 && _base_axis_active_state.axis_2;
  }

  bool base_manipulator_has_pwr() const
  {
    return _base_axis_pwr_state.axis_0 && _base_axis_pwr_state.axis_1 && _base_axis_pwr_state.axis_2;
  }

  bool is_homed() const
  {
    //todo
    return false;
  }
  

  void update_pos_flags()
  {
    if((_clock->now() - _base_axis_time_last_pos.axis_0).seconds() > 0.5)
    {
      // RCLCPP_INFO(this->get_logger(), "base_axis_0: no pos TIMEOUT");
      _base_axis_got_pos.axis_0 = false;
    }
    if((_clock->now() - _base_axis_time_last_pos.axis_1).seconds() > 0.5)
    {
      // RCLCPP_INFO(this->get_logger(), "base_axis_1: no pos TIMEOUT");
      _base_axis_got_pos.axis_1 = false;
    }
    if((_clock->now() - _base_axis_time_last_pos.axis_2).seconds() > 0.5)
    {
      // RCLCPP_INFO(this->get_logger(), "base_axis_2: no pos TIMEOUT");
      _base_axis_got_pos.axis_2 = false;
    }
  }

private: //data
  const double _SERVO_MAX_RAD = 2.61799; //150°

  const double _SERVO_MIN = 30.0;
  const double _SERVO_MAX = 490.0;

  struct FrancorManipulator_params
  {
    double rate_loop = 20;
    double rate_heartbeat = 1;

    Manipulator_base_axis<francor::base::AnglePiToPi> base_pos_standby = {0.0, 2.28, -0.7};
    Manipulator_base_axis<francor::base::AnglePiToPi> base_pos_active  = {0.0, 1.5, 0.25};
    Manipulator_head_axis<francor::base::AnglePiToPi> head_pos_standby = {0, 1.68, -1.64};
    Manipulator_head_axis<francor::base::AnglePiToPi> head_pos_active  = {0, 1.6, 0};

    double sh_safe_yaw = -1.65;
    double sh_safe_pitch = 0.6;

    // double axis_tolerance_rad = 0.03;
    double homing_speed = 0.3;
    double angle_increment_speed = 0.03;
  } _params;


  //clock 
  rclcpp::Clock::SharedPtr _clock;

  //timer
  rclcpp::TimerBase::SharedPtr _timer_loop;
  rclcpp::TimerBase::SharedPtr _timer_heartbeat;

  //publishers

  //arm_head pubs (lx16a)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_head_pos_pan;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_head_pos_tilt;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_head_pos_roll;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_head_speed_pan;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_head_speed_tilt;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_head_speed_roll;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_sh_pitch;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         _pub_sh_yaw;

  //status pub
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             _pub_status;
  rclcpp::Publisher<francor_msgs::msg::ManipulatorCmd>::SharedPtr _pub_status_pos;

  //firmware_pubs
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr _pub_set_pos_axis;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr           _pub_heartbeat;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr           _pub_enable;


  //subscribers
  rclcpp::Subscription<francor_msgs::msg::ManipulatorCmd>::SharedPtr _sub_cmd_axis_speed;
  // rclcpp::Subscription<francor_msgs::msg::ManipulatorCmd>::SharedPtr _sub_cmd_axis_pos;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr       _sub_cmd_inverse_speed;
  //fimware_subs
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr _sub_axis_pos;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr _sub_axis_state;

  //sub servo
  rclcpp::Subscription<francor_msgs::msg::ServoLx16a>::SharedPtr _sub_head_pan;
  rclcpp::Subscription<francor_msgs::msg::ServoLx16a>::SharedPtr _sub_head_tilt;
  rclcpp::Subscription<francor_msgs::msg::ServoLx16a>::SharedPtr _sub_head_roll;

  //services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_set_mode_inverse;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_set_mode_axis;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_set_mode_active;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_set_mode_standby;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_set_init_enable; //after power on

  rclcpp::Logger _logger {rclcpp::get_logger("francor_manipulator_node")};

  
  Manipulator_base_axis<bool> _base_axis_pwr_state    = {false, false, false};
  Manipulator_base_axis<bool> _base_axis_active_state = {false, false, false};
  Manipulator_base_axis<rclcpp::Time> _base_axis_time_last_pos = {rclcpp::Time(0), rclcpp::Time(0), rclcpp::Time(0)};
  Manipulator_base_axis<bool> _base_axis_got_pos   = {false, false, false};
  Manipulator_base_axis<francor::base::AnglePiToPi> _base_axis_pos     = {0.0, 0.0, 0.0};

  //commands
  Manipulator_base_axis<francor::base::AnglePiToPi> _desired_base_axis_pos = {0.0, 0.0, 0.0};
  Manipulator_base_axis<francor::base::AnglePiToPi> _target_base_axis_pos = {0.0, 0.0, 0.0};

  Manipulator_head_axis<francor::base::AnglePiToPi> _desired_head_axis_pos = {0.0, 1.6, 0.0};
  Manipulator_head_axis<francor::base::AnglePiToPi> _last_head_pos = {0.0, 0.0, 0.0};

  Axis _desired_inverse_pos;

  int32_t _gripper_pos = 1500;

  francor_msgs::msg::ManipulatorCmd::SharedPtr _last_cmd_axis_speed;
  geometry_msgs::msg::Vector3::SharedPtr _last_cmd_inverse_speed;

  ENM_MODE _mode  = ENM_MODE::IDLE;             //overall mode
  ENM_MODE _selected_mode = ENM_MODE::AXIS;     //axis or inverse mode
  ENM_MODE _selected_mov_mode = ENM_MODE::IDLE; //moving standby or active

};


#endif  //FRANCORMANIPULATORNODE_H_