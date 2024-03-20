//this module handles interfacing to the motors
#include "arduino.h"
#include "melty_config.h"
#include "led_driver.h"
#include "DShot.h"

//motor_X_on functions are used for the powered phase of each rotation
//motor_X_coast functions are used for the unpowered phase of each rotation
//motor_X_off functions are used for when the robot is spun-down

DShot motor1 = DShot(DShot::Mode::DSHOT600);
DShot motor2 = DShot(DShot::Mode::DSHOT600);

// So, in DShot, any throttle value < 48 is a motor configuration command.
// 0, conveniently, is MOTOR_STOP, so 0 is 0.
// This function is only getting called when we want the robot to spin, so, we'll just treat any incoming command code as a 0
void motors_on(int motor_one_throttle_perk, int motor_two_throttle_perk)
{
  if (motor_one_throttle_perk < 48) {
    motor_one_throttle_perk = 0;
  }

  if (motor_two_throttle_perk < 48) {
    motor_two_throttle_perk = 0;
  }

  motor1.setThrottle(motor_one_throttle_perk);
  motor2.setThrottle(motor_two_throttle_perk);
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