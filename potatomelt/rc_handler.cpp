//This module handles the RC interface (interrupt driven)
#include <Arduino.h>
#include "sbus.h"

#include "rc_handler.h"
#include "melty_config.h"

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS data */
bfs::SbusData data;

unsigned long control_checksum;
unsigned long last_changed_at;

float correction_factor = 1024.0 / NOMINAL_PULSE_RANGE;

bool rc_signal_is_healthy() {
  unsigned long new_checksum = compute_checksum();
  unsigned long now = millis();
  bool pass = false;
  if (new_checksum != control_checksum) {
    last_changed_at = now;
    control_checksum = new_checksum;
    pass = true;
  } else {
    pass = ((now - last_changed_at) < CONTROL_MOTION_TIMEOUT_MS);
  }
  return pass;
}

//returns at integer from 0 to 1024 based on throttle position
//default values are intended to have "dead zones" at both top
//and bottom of stick for 0 and 1024 percent
int rc_get_throttle_perk() {

  int pulse_length = get_channel(RC_CHANNEL_THROTTLE);

  if (pulse_length >= FULL_THROTTLE_PULSE_LENGTH) return 1024;
  if (pulse_length <= MIN_RC_PULSE_LENGTH + RC_TRIM_EPSILON) return 0;

  return (int) ((pulse_length - MIN_RC_PULSE_LENGTH) * correction_factor);
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
rc_forback rc_get_forback_bit() {

  int forback = rc_get_forback_trans();

  if (forback > FORBACK_MIN_THRESH_PULSE_LENGTH) return RC_FORBACK_FORWARD;
  if (forback < (FORBACK_MIN_THRESH_PULSE_LENGTH * -1)) return RC_FORBACK_BACKWARD;
  return RC_FORBACK_NEUTRAL;
}

// Returns -512 -> 512 for the forwards-backwards axis
int rc_get_forback_trans() {
  int stick_position = get_channel(RC_CHANNEL_FORBACK);
  return (int) (stick_position - CENTER_FORBACK_PULSE_LENGTH) * correction_factor;
}

//returns offset in microseconds from center value (not converted to percentage)
//0 for hypothetical perfect center (reality is probably +/-50)
//returns negative value for left / positive value for right
int rc_get_leftright() {
  int pulse_length = get_channel(RC_CHANNEL_TURN);

  return pulse_length - CENTER_LEFTRIGHT_PULSE_LENGTH;
}

// returns true if we're in tank mode!
bool rc_get_tank_mode() {
  return get_channel(RC_CHANNEL_TANKMODE) > CENTER_FORBACK_PULSE_LENGTH;
}

// Returns true if the accel correction factor save button is pushed
bool rc_get_accel_save() {
  return get_channel(RC_CHANNEL_ACCEL_OFFSET_SAVE) > CENTER_FORBACK_PULSE_LENGTH;
}

int rc_get_spin_dir() {
  return (get_channel(RC_CHANNEL_SPIN_DIR) > CENTER_FORBACK_PULSE_LENGTH) ? 1 : -1;
}

float rc_get_trans_trim() {
  #ifdef USE_TRANSLATION_TRIM
  int trim_setting = get_channel(RC_CHANNEL_TRANSLATION_TRIM) - MIN_RC_PULSE_LENGTH;
  return trim_setting * 16 / NOMINAL_PULSE_RANGE;
  #else
  return DEFAULT_TRANSLATION_TRIM;
  #endif
}

int get_channel(int channel) {
  return data.ch[channel];
}

bool rc_poll() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    return true;
  }

  return false;
}

//attach interrupts to rc pins
void init_rc(void) {
  sbus_rx.Begin();
}

unsigned long compute_checksum() {
  return data.ch[0]*64 + data.ch[1]*16 + data.ch[2]*4 + data.ch[3];
}