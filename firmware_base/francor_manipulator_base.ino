#include "Arduino.h"
#include <ros.h>

#include <Servo.h>
#include <std_msgs/UInt16.h>
// #include <std_msgs/String.h>

constexpr uint16_t SERVO1_US_MAX = 2010;
constexpr uint16_t SERVO1_US_MIN = 1030;
constexpr uint16_t SERVO1_ANALOG_MAX = 50;
constexpr uint16_t SERVO1_ANALOG_MIN = 1023;

constexpr uint16_t SERVO_DIFF_MAX = 500;

ros::NodeHandle  nh;
// std_msgs::String log_msg;

std_msgs::UInt16 pos_axis0;
std_msgs::UInt16 pos_axis1;
std_msgs::UInt16 pos_axis2;

// ros::Publisher pub_log("robotic_arm/log", &log_msg);

ros::Publisher pub_pos_axis0("manipulator/pos/axis0", &pos_axis0);
ros::Publisher pub_pos_axis1("manipulator/pos/axis1", &pos_axis1);
ros::Publisher pub_pos_axis2("manipulator/pos/axis2", &pos_axis2);


Servo servo_axis0;
Servo servo_axis1;
Servo servo_axis2;
Servo servo_gripper;

bool servo_0_ok = true;  //true for now ... servo not used for now
bool servo_1_ok = false;
bool servo_2_ok = false;
// bool servo_3_ok = true;  //true for now ... servo not used for now

bool servo_init_rdy = false;

// // -- functions --
// void log(const char* msg)
// {
//   // log_msg.data = msg;
//   // pub_log.publish(&log_msg);
// }

void sub_axis0_callback(const std_msgs::UInt16& msg)
{
  if(!servo_0_ok)
    return;
  uint16_t val = constrain(msg.data, SERVO1_US_MIN, SERVO1_US_MAX);
  // if(abs(val - pos_axis0.data) < SERVO_DIFF_MAX)
  {
    servo_axis0.writeMicroseconds(val);
  }
}

void sub_axis1_callback(const std_msgs::UInt16& msg)
{
  if(!servo_1_ok)
    return;
  uint16_t val = constrain(msg.data, SERVO1_US_MIN, SERVO1_US_MAX);
  // if(abs(val - pos_axis1.data) < SERVO_DIFF_MAX)
  {
    servo_axis1.writeMicroseconds(val);
  }
  // digitalWrite(13, HIGH-digitalRead(13));
  // log("got pos data");

  // servo.writeMicroseconds(msg.data);
}

void sub_axis2_callback(const std_msgs::UInt16& msg)
{
  if(!servo_2_ok)
    return;
  //todo fit for servo2
  uint16_t val = constrain(msg.data, SERVO1_US_MIN, SERVO1_US_MAX);
  // if(abs(val - pos_axis2.data) < SERVO_DIFF_MAX)
  {
    servo_axis2.writeMicroseconds(val);
  }
}

void sub_gripper_callback(const std_msgs::UInt16& msg)
{
  uint16_t val = constrain(msg.data, 500 , 3000);
  servo_gripper.writeMicroseconds(val);
}


// -- subs --
ros::Subscriber<std_msgs::UInt16> sub_pos_axis0("manipulator/set_pos/axis0", &sub_axis0_callback);
ros::Subscriber<std_msgs::UInt16> sub_pos_axis1("manipulator/set_pos/axis1", &sub_axis1_callback);
ros::Subscriber<std_msgs::UInt16> sub_pos_axis2("manipulator/set_pos/axis2", &sub_axis2_callback);
ros::Subscriber<std_msgs::UInt16> sub_pos_gripper("manipulator/set_pos/gripper", &sub_gripper_callback);


uint16_t servo1_ain_to_ms(const uint16_t ain)
{
  return map(ain, SERVO1_ANALOG_MIN, SERVO1_ANALOG_MAX, SERVO1_US_MIN, SERVO1_US_MAX);
}

/**
 * @brief 
 * 
 * @todo ajust for servo2
 * 
 * @param ain 
 * @return uint16_t 
 */
uint16_t servo2_ain_to_ms(const uint16_t ain)
{
  return map(ain, SERVO1_ANALOG_MIN, SERVO1_ANALOG_MAX, SERVO1_US_MIN, SERVO1_US_MAX);
}

bool initServos()
{
  //init servo ... set to actual pos
  uint16_t pos0 = analogRead(0);
  uint16_t pos1 = analogRead(1);
  uint16_t pos2 = analogRead(2);
  pos0 = servo1_ain_to_ms(pos0);
  servo_axis0.attach(12);
  servo_axis0.writeMicroseconds(pos0);
  pos1 = servo1_ain_to_ms(pos1);
  servo_axis1.attach(9);
  servo_axis1.writeMicroseconds(pos1);
  pos2 = servo2_ain_to_ms(pos2);
  servo_axis2.attach(5);
  servo_axis2.writeMicroseconds(pos2);
}

void setup()
{

  //Servo 0 out    : 12
  //Servo 0 in_pos : A0
  //Servo 0 in_v   : 10
  //Servo 1 out    : 9
  //Servo 1 in_pos : A1
  //Servo 1 in_v   : 8
  //Servo 2 out    : 5
  //Servo 2 in_pos : A2
  //Servo 2 in_v   : 6
  //Servo 3 out    : 2
  //Servo 3 in_pos : A3
  //Servo 3 in_v   : 4

  //Servo Gripper out: 3

  pinMode(10, INPUT);
  pinMode(8, INPUT);
  pinMode(6, INPUT);
  pinMode(4, INPUT);


  servo_gripper.attach(3);

  
  

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  // nh.advertise(pub_log);

  nh.advertise(pub_pos_axis0);
  nh.advertise(pub_pos_axis1);
  nh.advertise(pub_pos_axis2);

  nh.subscribe(sub_pos_axis0);
  nh.subscribe(sub_pos_axis1);
  nh.subscribe(sub_pos_axis2);
  nh.subscribe(sub_pos_gripper);
}

int g_cnt = 0;
bool g_servos_rdy = false;


void loop()
{
  nh.spinOnce();

  //check if servos have power
  servo_0_ok =  (digitalRead(10) == HIGH) ? true : false;
  servo_1_ok =  (digitalRead(8) == HIGH) ? true : false;
  servo_2_ok =  (digitalRead(6) == HIGH) ? true : false;

  if(!servo_2_ok || !servo_1_ok || !servo_2_ok)
  {
    g_servos_rdy = false;
    servo_axis0.detach();
    servo_axis1.detach();
    servo_axis2.detach();

  }

  if(servo_0_ok && servo_1_ok && servo_2_ok && !g_servos_rdy)
  {
    //init servos
    initServos();

    g_servos_rdy = true;
  }

  // servo_3_ok =  (digitalRead(4) == HIGH) ? true : false;
  int pos0 = analogRead(0);
  int pos1 = analogRead(1);
  int pos2 = analogRead(2);

// pub 65535 if servo has no voltage
  pos_axis0.data = servo_0_ok ? servo1_ain_to_ms(pos0) : 65535;
  pos_axis1.data = servo_1_ok ? servo1_ain_to_ms(pos1) : 65535;
  pos_axis2.data = servo_2_ok ? servo2_ain_to_ms(pos2) : 65535;
  

  pub_pos_axis0.publish(&pos_axis0);
  pub_pos_axis1.publish(&pos_axis1);
  pub_pos_axis2.publish(&pos_axis2);

  delay(10);
}
