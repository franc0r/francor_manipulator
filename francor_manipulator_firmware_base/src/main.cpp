

//begin
#include <Arduino.h>
#include <WString.h>
#include <micro_ros_platformio.h>
#include <cstdint>
#include <Servo.h>

// #include <stdio.h>
// #include <unistd.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>




#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/string.h>


#include "FrancorServo.h"
#include "wiring.h"



//publisher (pos, status)
rcl_publisher_t pub_axis;   //int32 multiarray
rcl_publisher_t pub_status; //uint8 multiarray
rcl_publisher_t pub_info;   //string

std_msgs__msg__Int32MultiArray msg_out_axis;
std_msgs__msg__UInt8MultiArray msg_out_status;
std_msgs__msg__String          msg_out_info;  

rcl_subscription_t sub_axis; //int32 multiarray
rcl_subscription_t sub_heartbeat; //uint8
rcl_subscription_t sub_enable; //uint8

std_msgs__msg__Int32MultiArray msg_in_axis;
std_msgs__msg__UInt8           msg_in_heartbeat; 
std_msgs__msg__UInt8           msg_in_enable;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


uint32_t g_cnt = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Error handle loop
void error_loop() {
  while(1) {
    delay(50);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}


/**
 * @brief todo check if ok ??? may be hack but fuck it
 * 
 * @param msg 
 * @param size 
 */
void allocate_msg_int32_multi_array(std_msgs__msg__Int32MultiArray *msg,  const int32_t size)
{
  msg->data.data = new int32_t[size];
  msg->data.size = size;
  msg->data.capacity = size;
}

void allocate_msg_uint8_multi_array(std_msgs__msg__UInt8MultiArray *msg,  const int32_t size)
{
  msg->data.data = new uint8_t[size];
  msg->data.size = size;
  msg->data.capacity = size;
}

void pub_helper_int32_array(rcl_publisher_t& pub, const int32_t axis0, const int axis1, const int axis2) //todo gripper?
{
  msg_out_axis.data.data[0] = axis0;
  msg_out_axis.data.data[1] = axis1;
  msg_out_axis.data.data[2] = axis2;
  RCSOFTCHECK(rcl_publish(&pub, &msg_out_axis, NULL));
  // rcl_publish(pub, &msg_out_axis);
}

void pub_helper_uint8_array(rcl_publisher_t& pub, const uint8_t axis0, const uint8_t axis1, const uint8_t axis2) //todo gripper?
{
  msg_out_status.data.data[0] = axis0;
  msg_out_status.data.data[1] = axis1;
  msg_out_status.data.data[2] = axis2;
  RCSOFTCHECK(rcl_publish(&pub, &msg_out_status, NULL));
  // rcl_publish(pub, &msg_out_status);
}

void log_info(const String& str)
{
  msg_out_info.data.data = (char*)str.c_str();
  msg_out_info.data.size = str.length() + 1;
  msg_out_info.data.capacity = str.length() + 1;
  RCSOFTCHECK(rcl_publish(&pub_info, &msg_out_info, NULL));
  // rcl_publish(pub_info, &msg_out_info);
}

template<typename T>
struct Axis_t{
  T axis0;
  T axis1;
  T axis2;
};

//Manipulator Servo Holding class
class FrancorManipulatorBase{
public:
  FrancorManipulatorBase() : 
    _servo_axis0(FrancorServo(12, 11, 1, SERVO_0_MAX, SERVO_0_MIN)),
    _servo_axis1(FrancorServo(10, 9, 2, SERVO_1_MAX, SERVO_1_MIN)),
    _servo_axis2(FrancorServo(6, 7, 3, SERVO_2_MAX, SERVO_2_MIN))
  {

  }
  ~FrancorManipulatorBase() = default;
  


  void init()
  {
    _servo_axis0.init();
    _servo_axis1.init();
    _servo_axis2.init();
    _servo_gripper.attach(4);
    _servo_gripper.write(90); //send middle pos as init
  }
  
  bool isEnabled() const
  {
    return _servo_axis0.isEnabled() && _servo_axis1.isEnabled() && _servo_axis2.isEnabled();
  }

  uint8_t enable()
  {
    uint8_t ret0 = _servo_axis0.enable();
    uint8_t ret1 = _servo_axis1.enable();
    uint8_t ret2 = _servo_axis2.enable();

    return max(ret0, max(ret1, ret2));
  }

  void disable()
  {
    _servo_axis0.disable();
    _servo_axis1.disable();
    _servo_axis2.disable();
  }

  bool eachHasPower() const
  {
    return _servo_axis0.hasPower() && _servo_axis1.hasPower() && _servo_axis2.hasPower();
  }

  Axis_t<bool> hasPower() const
  {
    Axis_t<bool> ret;
    ret.axis0 = _servo_axis0.hasPower();
    ret.axis1 = _servo_axis1.hasPower();
    ret.axis2 = _servo_axis2.hasPower();
    return ret;
  }

  void setPos(const int32_t axis0, const int32_t axis1, const int32_t axis2, const int32_t gripper)
  {
    _servo_axis0.setPos(axis0);
    _servo_axis1.setPos(axis1);
    _servo_axis2.setPos(axis2);

    _servo_gripper.writeMicroseconds(gripper);
  }

  void tick()
  {
    _servo_axis0.tick();
    _servo_axis1.tick();
    _servo_axis2.tick();
  }

  Axis_t<int32_t> getPos() const
  {
    Axis_t<int32_t> ret;
    ret.axis0 = _servo_axis0.getPos();
    ret.axis1 = _servo_axis1.getPos();
    ret.axis2 = _servo_axis2.getPos();
    return ret;
  }

  Axis_t<uint8_t> getStatus() const
  {
    Axis_t<uint8_t> ret;
    ret.axis0 = _servo_axis0.getStatus();
    ret.axis1 = _servo_axis1.getStatus();
    ret.axis2 = _servo_axis2.getStatus();
    return ret;
  }

private:
  FrancorServo _servo_axis0;
  FrancorServo _servo_axis1;
  FrancorServo _servo_axis2;

  Servo        _servo_gripper;
  //todo sendMicroseconds to gripper in tick and save last value here and init value??
};



FrancorManipulatorBase g_man_base;







//callback heart beat
void sub_heartbeat_callback(const void* msg)
{
  RCLC_UNUSED(msg);
  //do something with the message
  // pub_helperInt32(pub_axis_0, msg->data);
  //toggling the led
  // log_info(String("Got heartbeat"));
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

//callback sub_axis
void sub_axis_callback(const void* msg_in)
{
  const std_msgs__msg__Int32MultiArray* msg = (const std_msgs__msg__Int32MultiArray*)msg_in;
  if(msg->data.size != 3)
  {
    log_info(String("Got wrong axis message"));
    return;
  }

}

void sub_enable_callback(const void* msg_in)
{
  const std_msgs__msg__UInt8* msg = (const std_msgs__msg__UInt8*)msg_in;

  // String log("SRV -> ");
  // log += String(msg->data);
  // log_info(log);

  if(msg->data)
  {
    auto ret = g_man_base.enable();
    if(ret)
    {
      String msg;
      if(ret == 1)
      {
        msg = "Enabled - TargetPos -> OK";
      }
      else
      {
        msg = "Enabled - TargetPos -> TOO FAR -> CURR POS USED";
      }
      log_info(msg);
    }
  }
  else
  {
    g_man_base.disable();
  }
}




//timer callbacks
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    g_man_base.tick();

    //get pos
    auto pos = g_man_base.getPos();
    pub_helper_int32_array(pub_axis, pos.axis0, pos.axis1, pos.axis2);
    
    //get status:
    auto status = g_man_base.getStatus();
    pub_helper_uint8_array(pub_status, status.axis0, status.axis1, status.axis2);
  }
}






void setup() {
  //set Pin as Output
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "francor_manipulator_base_node", "", &support));

  // //init publishers
  RCCHECK(rclc_publisher_init_default(&pub_axis, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "manipulator/pos/axis"));
  RCCHECK(rclc_publisher_init_default(&pub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "manipulator/pos/status"));
  RCCHECK(rclc_publisher_init_default(&pub_info, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "manipulator/info"));

  //init subscribers
  RCCHECK(rclc_subscription_init_default(&sub_axis, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "manipulator/set_pos/axis"));
  RCCHECK(rclc_subscription_init_default(&sub_heartbeat, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "manipulator/heartbeat"));
  RCCHECK(rclc_subscription_init_default(&sub_enable, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "manipulator/enable"));

  //init services //IMPORTANT NOTE -> Teensy only supports one Service -> second will cause RCCHECK ERROR
  
  //init timer
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default( &timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  //init executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_axis, &msg_in_axis, &sub_axis_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_heartbeat, &msg_in_heartbeat, &sub_heartbeat_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_enable, &msg_in_enable, &sub_enable_callback, ON_NEW_DATA));

  g_man_base.init();

  //allocate out msgs
  allocate_msg_int32_multi_array(&msg_out_axis, 3);
  allocate_msg_uint8_multi_array(&msg_out_status, 3);
}

void loop() 
{
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}