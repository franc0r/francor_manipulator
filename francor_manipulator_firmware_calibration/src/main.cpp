

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
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/string.h>


// #include "FrancorServo.h"
#include "../../francor_manipulator_firmware_base/include/FrancorServo.h"

enum enmStatus {
    OFF           = 0,
    ON_ACTIVE,
    ON_INACTIVE
};

//publisher (pos, status)
rcl_publisher_t pub_axis;   //int32 
rcl_publisher_t pub_status; //uint8 
rcl_publisher_t pub_info;   //string

std_msgs__msg__Int32  msg_out_axis;
std_msgs__msg__UInt8  msg_out_status;
std_msgs__msg__String msg_out_info;  

rcl_subscription_t sub_axis; //int32
rcl_subscription_t sub_heartbeat; //uint8
rcl_subscription_t sub_enable; //uint8

std_msgs__msg__Int32 msg_in_axis;
std_msgs__msg__UInt8 msg_in_heartbeat; 
std_msgs__msg__UInt8 msg_in_enable;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;




uint32_t g_cnt = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// FrancorServo servo(12, 11, 1, 30, 497); //axis0 -> 2000us -> 30, 1000us -> 497
// FrancorServo servo(10, 9, 2, 28, 494); //axis1 -> 2000us -> 28an, 1000us -> 494an
// FrancorServo servo(6, 7, 3, 30, 493); //axis2 -> 2000us -> 30, 1000us -> 493



// Error handle loop
void error_loop() {
  while(1) {
    delay(50);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}


void pub_helper_int32(rcl_publisher_t& pub, const int32_t axis)
{
  msg_out_axis.data = axis;
  RCSOFTCHECK(rcl_publish(&pub, &msg_out_axis, NULL));
}

void pub_helper_uint8(rcl_publisher_t& pub, const uint8_t axis)
{
  msg_out_status.data = axis;
  RCSOFTCHECK(rcl_publish(&pub, &msg_out_status, NULL));
}

void log_info(const String& str)
{
  msg_out_info.data.data = (char*)str.c_str();
  msg_out_info.data.size = str.length() + 1;
  msg_out_info.data.capacity = str.length() + 1;
  RCSOFTCHECK(rcl_publish(&pub_info, &msg_out_info, NULL));
}

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
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msg_in;
  servo.debug_writeServo(msg->data);
  // String dbg_str("posToPwm(");
  // dbg_str += String(msg->data);
  // dbg_str += ")";
  // dbg_str += " -> ";
  // dbg_str += String(servo.posToPwm(msg->data));

  // log_info(dbg_str);
  // servo.setPos(msg->data);
}

void sub_enable_callback(const void* msg_in)
{
  const std_msgs__msg__UInt8* msg = (const std_msgs__msg__UInt8*)msg_in;

  String log("SRV -> ");
  log += String(msg->data);
  log_info(log);
}




//timer callbacks
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    servo.debug_tick();
    // servo.tick();
    //pub state and pos
    pub_helper_int32(pub_axis, servo.getPos());
    pub_helper_uint8(pub_status, (servo.hasPower() ? 1 : 0));

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
  RCCHECK(rclc_publisher_init_default(&pub_axis, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "manipulator/pos/axis"));
  RCCHECK(rclc_publisher_init_default(&pub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "manipulator/pos/status"));
  RCCHECK(rclc_publisher_init_default(&pub_info, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "manipulator/info"));

  //init subscribers
  RCCHECK(rclc_subscription_init_default(&sub_axis, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "manipulator/set_pos/axis"));
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

  servo.init();
  servo.enable();
}

void loop() 
{
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}