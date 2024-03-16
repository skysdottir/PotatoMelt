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

void motor_on(float throttle_percent, int motor_pin) {

  if (THROTTLE_TYPE == BINARY_THROTTLE) {
    digitalWrite(motor_pin, HIGH);
  }

  if (THROTTLE_TYPE == FIXED_PWM_THROTTLE) {
    if (motor_pin = MOTOR_PIN1) {
      motor1.setThrottle(DSHOT_MOTOR_ON);
    } else {
      motor2.setThrottle(DSHOT_MOTOR_ON);
    }
  }

//If DYNAMIC_PWM_THROTTLE - PWM is scaled between PWM_MOTOR_COAST and PWM_MOTOR_ON
//Applies over range defined by DYNAMIC_PWM_THROTTLE_PERCENT_MAX - maxed at PWM_MOTOR_ON above this
  if (THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
    int throttle_pwm = DSHOT_MOTOR_COAST + ((throttle_percent / DYNAMIC_PWM_THROTTLE_PERCENT_MAX) * (DSHOT_MOTOR_ON - DSHOT_MOTOR_COAST));
    if (throttle_pwm > DSHOT_MOTOR_ON) throttle_pwm = DSHOT_MOTOR_ON;
    if (motor_pin = MOTOR_PIN1) {
      motor1.setThrottle(throttle_pwm);
    } else {
      motor2.setThrottle(throttle_pwm);
    }
  }
}

void motor_1_on(float throttle_percent) {
  motor_on(throttle_percent, MOTOR_PIN1);
}

void motor_2_on(float throttle_percent) {
  motor_on(throttle_percent, MOTOR_PIN2);
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
  motor_1_off();
  motor_2_off();
}

void init_motors() {
  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motors_off();
}