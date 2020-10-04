#include "Arduino.h"
#include <ros.h>

#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

constexpr uint16_t SERVO1_US_MAX = 2010;
constexpr uint16_t SERVO1_US_MIN = 1030;
constexpr uint16_t SERVO1_ANALOG_MAX = 50;
constexpr uint16_t SERVO1_ANALOG_MIN = 1023;

ros::NodeHandle  nh;
std_msgs::String log_msg;

std_msgs::UInt16 pos_axis1;
std_msgs::UInt16 pos_axis2;

ros::Publisher pub_log("robotic_arm/log", &log_msg);

ros::Publisher pub_pos_axis1("manipulator/pos/axis1", &pos_axis1);
ros::Publisher pub_pos_axis2("manipulator/pos/axis2", &pos_axis2);


Servo servo_axis1;
Servo servo_axis2;

// -- functions --
void log(const char* msg)
{
  log_msg.data = msg;
  pub_log.publish(&log_msg);
}

void sub_axis1_callback(const std_msgs::UInt16& msg)
{
  uint16_t val = constrain(msg.data, SERVO1_US_MIN, SERVO1_US_MAX);
  servo_axis1.writeMicroseconds(val);
  // digitalWrite(13, HIGH-digitalRead(13));
  log("got pos data");

  // servo.writeMicroseconds(msg.data);
}

void sub_axis2_callback(const std_msgs::UInt16& msg)
{
  //todo fit for servo2
  uint16_t val = constrain(msg.data, SERVO1_US_MIN, SERVO1_US_MAX);
  servo_axis2.writeMicroseconds(val);
}


// -- subs --

ros::Subscriber<std_msgs::UInt16> sub_pos_axis1("manipulator/set_pos/axis1", &sub_axis1_callback);
ros::Subscriber<std_msgs::UInt16> sub_pos_axis2("manipulator/set_pos/axis2", &sub_axis2_callback);


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

void setup()
{
  // Serial.begin(9600);

  servo_axis1.attach(9);
  servo_axis2.attach(5);

  //init servo ... set to actual pos
  uint16_t pos1 = analogRead(1);
  uint16_t pos2 = analogRead(2);

  pos1 = servo1_ain_to_ms(pos1);
  servo_axis1.writeMicroseconds(pos1);
  pos2 = servo2_ain_to_ms(pos2);
  servo_axis2.writeMicroseconds(pos2);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_log);

  nh.advertise(pub_pos_axis1);
  nh.advertise(pub_pos_axis2);

  nh.subscribe(sub_pos_axis1);
  nh.subscribe(sub_pos_axis2);
}

int g_cnt = 0;


void loop()
{
  nh.spinOnce();

  int pos1 = analogRead(1);
  int pos2 = analogRead(2);

  pos_axis1.data = servo1_ain_to_ms(pos1);
  pos_axis2.data = servo2_ain_to_ms(pos2);

  pub_pos_axis1.publish(&pos_axis1);
  pub_pos_axis2.publish(&pos_axis2);

  delay(10);
}
