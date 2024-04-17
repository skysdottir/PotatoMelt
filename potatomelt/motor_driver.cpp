//this module handles interfacing to the motors
#include "arduino.h"
#include "melty_config.h"
#include "led_driver.h"
#include <DShot.h>
#include "motor_driver.h"

//motor_X_on functions are used for the powered phase of each rotation
//motor_X_coast functions are used for the unpowered phase of each rotation
//motor_X_off functions are used for when the robot is spun-down

DShot motor1 = DShot(DShot::Mode::DSHOT600);
DShot motor2 = DShot(DShot::Mode::DSHOT600);

// In DShot, any throttle value < 48 is a motor configuration command.
// The throttle ranges are then 1000 steps wide, from 48 -> 1047 for direction 1 and 1049-2047 for direction 2
// Yes, this means that we go from full forwards throttle at 1047 to minimum reverse throttle at 1049.
// Also note, 1048 is not 0! We need to send an actual 0 for 0.
// See: https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs
// 
//Todo: fix this to not just truncate the throttle range
int perk2dshot(int throttle_perk) {
  if (throttle_perk == 0) {
    return 0;
  }
  
  if (throttle_perk > 0) {
    return min(throttle_perk, 999) + 48;
  }

  // otherwise, throttle_perk must be less than zero
  return min(throttle_perk*-1, 998) + 1049;
}

void motors_on(int motor_one_throttle_perk, int motor_two_throttle_perk)
{
  motors_on_direct(perk2dshot(motor_one_throttle_perk), perk2dshot(motor_two_throttle_perk));
}

// Just send the input codes directly to the motors
// This gets used in the hot loop, where we're going to be sending the same values over and over
// So we can precompute them once instead of doing all of motors_on() each time
void motors_on_direct(int motor_one_throttle_val, int motor_two_throttle_val)
{
  motor1.setThrottle(motor_one_throttle_val);
  motor2.setThrottle(motor_two_throttle_val);
}

void configure_motors(int config_code) {
  motors_on_direct(config_code, config_code);
  delay(40);
  motors_on_direct(12, 12); // save setting
  delay(40);
  motors_off();
}

void motors_off() {
  motor1.setThrottle(0);
  motor2.setThrottle(0);
}

void init_motors() {
  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motors_off();
}