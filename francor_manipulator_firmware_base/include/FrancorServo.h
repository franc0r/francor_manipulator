#include <Arduino.h>
#include <WString.h>
#include <micro_ros_platformio.h>
#include <cstdint>
#include <Servo.h>



//Servo pins:
//servo_axis0; //analogpin 1 | pwm 12 | vin 11
//servo_axis1; //analogpin 2 | pwm 10 | vin 9
//servo_axis2; //analogpin 3 | pwm 6 | vin 7
//   //axis 4 //analogpin 4 | pwm 3 | vin 5
//servo_gripper; //pwm 4 |

#define MAX_PWM_US 2000
#define MIN_PWM_US 1000

#define SERVO_0_MAX 30
#define SERVO_0_MIN 497
#define SERVO_1_MAX 28
#define SERVO_1_MIN 494
#define SERVO_2_MAX 30
#define SERVO_2_MIN 493


enum ServoStatus {
  HAS_POWER = 1,
  IS_ENDABLED = 2,
};

class FrancorServo{
public:
  /**
   * @brief Construct a new Francor Servo object
   * 
   * @param pwm_pin 
   * @param vin_pin 
   * @param ain_pin 
   * @param max_pos -> analog pos from servo at MAX PWM (2000)
   * @param min_pos -> analog pos form servo at MIN PWM (1000)
   */
  FrancorServo(const int32_t pwm_pin, const int32_t vin_pin,  const int32_t ain_pin, const int32_t max_pos, const int32_t min_pos):
    _pwm_pin(pwm_pin),
    _vin_pin(vin_pin),
    _ain_pin(ain_pin),
    _MAX_POS(max_pos),
    _MIN_POS(min_pos)
  { }

  ~FrancorServo() = default;
  
  void init()
  {
    pinMode(_vin_pin, INPUT);
    pinMode(_ain_pin, INPUT);
  }
  /**
   * @brief 
   * 
   * @return 0 if already enabled, or has no power
   * @return 1 if target_pos is ok 
   * @return 2 if target_pos is not ok and curr pos is set
   */
  uint8_t enable()
  {
    if(_enabled || !_has_power)
    {
      return 0;
    }
    uint8_t ret = 1;
    if(!_was_active || abs(_curr_pos - _target_pos) > _MAX_REINIT_DIST)
    {
      _target_pos = constrain(_curr_pos, _MAX_POS, _MIN_POS);
      ret = 2;
    }
    //else curr target_pos is ok

    _servo.attach(_pwm_pin);
    _enabled = true;
    _had_blackout = false;

    return ret;
  }

  void disable()
  {
    if(!_enabled)
    {
      return;
    }

    _servo.detach();
    _enabled = false;
  }

  void tick()
  {
    _has_power = digitalRead(_vin_pin) == HIGH ? true : false;
    _curr_pos  = analogRead(_ain_pin);
    if(_was_active && !_has_power)
    {
      _had_blackout = true;
    }
    if(_enabled)
    {
      _was_active = true;
      _servo.writeMicroseconds(this->posToPwm(_target_pos));
    }
  }

  void debug_tick()
  {
    _has_power = digitalRead(_vin_pin) == HIGH ? true : false;
    _curr_pos  = analogRead(_ain_pin);
  }

  void debug_writeServo(const int32_t pos_us)
  {
    _servo.writeMicroseconds(pos_us);
  }

  void setPos(const int32_t pos)
  {
    if(!_enabled || !_has_power)
    {
      return;
    }


    _target_pos = constrain(pos, _MAX_POS, _MIN_POS);

  }

  int32_t getPos() const
  {
    if(_has_power)
    {
      if(_was_active && !_had_blackout)
      {
        return _target_pos;
      }
      else
      {
        return _curr_pos;
      }
    }
    else
    {
      return -1;
    }
  }

  bool isEnabled() const
  {
    return _enabled;
  }

  bool hasPower() const
  {
    return _has_power;
  }

  uint8_t getStatus() const
  {
    uint8_t status = 0;
    if(_enabled)
    {
      status |= IS_ENDABLED;
    }
    if(_has_power)
    {
      status |= HAS_POWER;
    }
    return status;
  }

  int32_t posToPwm(const int32_t pos) const
  {
    return map(pos, _MIN_POS, _MAX_POS, MIN_PWM_US, MAX_PWM_US);
  }

private:
  const int32_t _pwm_pin;
  const int32_t _vin_pin;
  const int32_t _ain_pin;
  const int32_t _MAX_POS;
  const int32_t _MIN_POS;

  const int32_t _MAX_REINIT_DIST = 25;

  Servo _servo;


  int32_t _target_pos = 0;
  int32_t _curr_pos   = 0;
  bool _enabled       = false;
  bool _has_power     = false;
  bool _was_active    = false;
  bool _had_blackout  = false;
};