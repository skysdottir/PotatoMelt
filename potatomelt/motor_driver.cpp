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

void motors_on(int motor_one_throttle_perk, int motor_two_throttle_perk)
{
  motor1.setThrottle(motor_one_throttle_perk);
  motor2.setThrottle(motor_two_throttle_perk);
}

void motor_coast(int motor_pin) {
  if (THROTTLE_TYPE == FIXED_PWM_THROTTLE || THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
    if (motor_pin = MOTOR_PIN1) {
      motor1.setThrottle(DSHOT_MOTOR_COAST);
    } else {
      motor2.setThrottle(DSHOT_MOTOR_COAST);
    }
  }
  if (THROTTLE_TYPE == BINARY_THROTTLE) {
    digitalWrite(motor_pin, LOW);  //same as "off" for brushed motors
  }
}

void motor_1_coast() {
  motor_coast(MOTOR_PIN1);
}

void motor_2_coast() {
  motor_coast(MOTOR_PIN2);
}

void motor_off(int motor_pin) {
  if (THROTTLE_TYPE == FIXED_PWM_THROTTLE || THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
    if (motor_pin = MOTOR_PIN1) {
      motor1.setThrottle(DSHOT_MOTOR_OFF);
    } else {
      motor2.setThrottle(DSHOT_MOTOR_OFF);
    }
  }
  if (THROTTLE_TYPE == BINARY_THROTTLE) {
    digitalWrite(motor_pin, LOW);  //same as "off" for brushed motors
  }
}

void motor_1_off() {
  motor_off(MOTOR_PIN1);
}

void motor_2_off() {
  motor_off(MOTOR_PIN2);
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