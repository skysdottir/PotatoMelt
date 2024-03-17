//This module handles the RC interface (interrupt driven)
#include <Arduino.h>
#include <IBusBM.h>


#include "rc_handler.h"
#include "melty_config.h"

IBusBM IBus;

//verifies that we are still receiving full power on channel 8 (switched on receiver).
//failsafe behavior on the receiver is to cut to 0 pulses, so this will drop.
//Also, throwing the configured switch will drop.
bool rc_signal_is_healthy() {
  return IBus.readChannel(7) > 2000;
}

//returns at integer from 0 to 1024 based on throttle position
//default values are intended to have "dead zones" at both top
//and bottom of stick for 0 and 1024 percent
int rc_get_throttle_perk() {

  int pulse_length = IBus.readChannel(2);

  if (pulse_length >= FULL_THROTTLE_PULSE_LENGTH) return 1024;
  if (pulse_length <= IDLE_THROTTLE_PULSE_LENGTH) return 0;

  return pulse_length - IDLE_THROTTLE_PULSE_LENGTH;
}

bool rc_get_is_lr_in_config_deadzone() {
  if (abs(rc_get_leftright()) < LR_CONFIG_MODE_DEADZONE_WIDTH) return true;
  return false;
}

bool rc_get_is_lr_in_normal_deadzone() {
  if (abs(rc_get_leftright()) < LR_NORMAL_DEADZONE_WIDTH) return true;
  return false;
}


//returns RC_FORBACK_FORWARD, RC_FORBACK_BACKWARD or RC_FORBACK_NEUTRAL based on stick position
rc_forback rc_get_forback() {

  int pulse_length = IBus.readChannel(1);

  int rc_forback_offset = pulse_length - CENTER_FORBACK_PULSE_LENGTH;
  if (rc_forback_offset > FORBACK_MIN_THRESH_PULSE_LENGTH) return RC_FORBACK_FORWARD;
  if (rc_forback_offset < (FORBACK_MIN_THRESH_PULSE_LENGTH * -1)) return RC_FORBACK_BACKWARD;
  return RC_FORBACK_NEUTRAL;
}

// Returns -512 -> 512 for the forwards-backwards axis
int rc_get_trans() {
  int stick_position = IBus.readChannel(1);
  return stick_position - CENTER_FORBACK_PULSE_LENGTH;
}

//returns offset in microseconds from center value (not converted to percentage)
//0 for hypothetical perfect center (reality is probably +/-50)
//returns negative value for left / positive value for right
int rc_get_leftright() {
  int pulse_length = IBus.readChannel(0);

  return pulse_length - CENTER_LEFTRIGHT_PULSE_LENGTH;
}

//attach interrupts to rc pins
void init_rc(void) {
  IBus.begin(Serial1);
}