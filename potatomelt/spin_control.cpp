//this module handles calculation and timing loop for translational drift
//direction of rotation assumed to be CLOCKWISE (but should work counter-clockwise)

#include "arduino.h"
#include "melty_config.h"
#include "motor_driver.h"
#include "rc_handler.h"
#include "spin_control.h"
#include "accel_handler.h"
#include "config_storage.h"
#include "led_driver.h"
#include "battery_monitor.h"

#define ACCEL_MOUNT_RADIUS_MINIMUM_CM 0.2                 //Never allow interactive config to set below this value
#define LEFT_RIGHT_CONFIG_RADIUS_ADJUST_DIVISOR 50.0f     //How quick accel. radius is adjusted in config mode (larger values = slower)
#define LEFT_RIGHT_CONFIG_LED_ADJUST_DIVISOR 0.1f         //How quick LED heading is adjusted in config mode (larger values = slower)

#define MAX_TRANSLATION_ROTATION_INTERVAL_US (1.0f / MIN_TRANSLATION_RPM) * 60 * 1000 * 1000
#define MAX_TRACKING_ROTATION_INTERVAL_US MAX_TRANSLATION_ROTATION_INTERVAL_US * 2   //don't track heading if we are this slow (also puts upper limit on time spent in melty loop for safety)

static float accel_mount_radius_cm = DEFAULT_ACCEL_MOUNT_RADIUS_CM;
static float accel_zero_g_offset = DEFAULT_ACCEL_ZERO_G_OFFSET;
static float led_offset_percent = DEFAULT_LED_OFFSET_PERCENT;         //stored in EEPROM as an INT - but handled as a float for configuration purposes

static unsigned int highest_rpm = 0;
static bool config_mode = false;   //1 if we are in config mode

//loads settings from EEPROM
void load_melty_config_settings() {
#ifdef ENABLE_EEPROM_STORAGE 
  accel_mount_radius_cm = load_accel_mount_radius();
  accel_zero_g_offset = load_accel_zero_g_offset();
  led_offset_percent = load_heading_led_offset();
#endif  
}

//saves settings to EEPROM
void save_melty_config_settings() {
#ifdef ENABLE_EEPROM_STORAGE 
  save_settings_to_eeprom(led_offset_percent, accel_mount_radius_cm, accel_zero_g_offset);
#endif  
}

//updated the expected accelerometer reading for 0g 
//assumes robot is not spinning when config mode is entered
//value saved to EEPROM on config mode exit
static void update_accel_zero_g_offset(){
  int offset_samples = 200;
  for (int accel_sample_loop = 0; accel_sample_loop < offset_samples; accel_sample_loop ++) {
    accel_zero_g_offset += get_accel_force_g();
  }
  accel_zero_g_offset = accel_zero_g_offset / offset_samples;
}

void toggle_config_mode() {
  config_mode = !config_mode;

  //on entering config mode - update the zero g offset
  if (config_mode) update_accel_zero_g_offset();
  
  //enterring or exiting config mode also resets highest observed RPM
  highest_rpm = 0;
}

bool get_config_mode() {
  return config_mode;
}

int get_max_rpm() {
  return highest_rpm;
}

//calculates time for this rotation of robot
//robot is steered by increasing / decreasing rotation by factor relative to RC left / right position
//ie - reducing rotation time estimate below actual results in shift of heading opposite the direction of rotation
static float get_rotation_interval_ms(int steering_enabled) {
  
  float radius_adjustment_factor = 0;

  //don't adjust steering if disabled by config mode - or we are in RC deadzone
  if (steering_enabled = 1 && rc_get_is_lr_in_normal_deadzone() == false) {
    radius_adjustment_factor = (float)(rc_get_turn_leftright() / (float)NOMINAL_PULSE_RANGE) / LEFT_RIGHT_HEADING_CONTROL_DIVISOR;
  }
  
  float effective_radius_in_cm = accel_mount_radius_cm;
  
  effective_radius_in_cm = effective_radius_in_cm + (effective_radius_in_cm * radius_adjustment_factor);

  float rpm;
  //use of absolute makes it so we don't need to worry about accel orientation
  //calculate RPM from g's - derived from "G = 0.00001118 * r * RPM^2"
  rpm = fabs(get_accel_force_g() - accel_zero_g_offset) * 89445.0f;
  rpm = rpm / effective_radius_in_cm;
  rpm = sqrt(rpm);

  if (rpm > highest_rpm || highest_rpm == 0) highest_rpm = rpm;

  float rotation_interval = (1.0f / rpm) * 60 * 1000;
  return rotation_interval;
}


//performs changes to melty parameters when in config mode
static struct melty_parameters_t handle_config_mode(melty_parameters_t *melty_parameters) {

  melty_parameters->translate_forback = rc_get_forback();
  //if forback forward - normal drive (for driver testing - no adjustment of melty parameters)

  //if forback neutral - then do radius adjustment
  if (melty_parameters->translate_forback == RC_FORBACK_NEUTRAL) {

    //radius adjustment overrides steering
    melty_parameters->movement_enabled = 0;

    //only adjust if stick is outside deadzone    
    if (rc_get_is_lr_in_config_deadzone() == false) {
      //show that we are changing config
      melty_parameters->led_shimmer = 1;

      float adjustment_factor = (accel_mount_radius_cm * (float)(rc_get_turn_leftright() / (float)NOMINAL_PULSE_RANGE));
      adjustment_factor = adjustment_factor / LEFT_RIGHT_CONFIG_RADIUS_ADJUST_DIVISOR;
      accel_mount_radius_cm = accel_mount_radius_cm + adjustment_factor;

      if (accel_mount_radius_cm < ACCEL_MOUNT_RADIUS_MINIMUM_CM) accel_mount_radius_cm = ACCEL_MOUNT_RADIUS_MINIMUM_CM;
    }    
  }
  
  //if forback backward - do LED heading adjustment (don't translate)
  if (melty_parameters->translate_forback == RC_FORBACK_BACKWARD) {
    //LED heading offset adjustment disables movement
    melty_parameters->movement_enabled = 0;
    
    //only adjust if stick is outside deadzone  
    if (rc_get_is_lr_in_config_deadzone() == false) {

      //show that we are changing config
      melty_parameters->led_shimmer = 1;

      float adjustment_factor =  (float)(rc_get_turn_leftright() / (float)NOMINAL_PULSE_RANGE);
      adjustment_factor = adjustment_factor / LEFT_RIGHT_CONFIG_LED_ADJUST_DIVISOR;
      led_offset_percent = led_offset_percent + adjustment_factor;

      if (led_offset_percent > 99) led_offset_percent = 0;
      if (led_offset_percent < 0) led_offset_percent = 99;

    }
  }
}

//Calculates all parameters need for a single rotation (motor timing, LED timing, etc.)
//This entire section takes ~1300us on an Atmega32u4 (acceptable - fast enough to not have major impact on tracking accuracy)
static void get_melty_parameters(melty_parameters_t *melty_parameters) {
  // set some of the defaults
  melty_parameters->led_shimmer = 0;
  melty_parameters->movement_enabled = 1;

  float led_offset_portion = led_offset_percent / 100.0f;

  melty_parameters->throttle_perk = rc_get_throttle_perk();

  float led_on_portion = melty_parameters->throttle_perk / 1024.0f;  //LED width changes with throttle percent
  if (led_on_portion < 0.10f) led_on_portion = 0.10f;
  if (led_on_portion > 0.90f) led_on_portion = 0.90f;

  //if we are in config mode - handle it (and disable steering if needed)
  if (get_config_mode() == true) {
    handle_config_mode(melty_parameters);
  }

  melty_parameters->rotation_interval_us = get_rotation_interval_ms(melty_parameters->movement_enabled) * 1000;

  //if we are too slow - don't even try to track heading
  if (melty_parameters->rotation_interval_us > MAX_TRACKING_ROTATION_INTERVAL_US) {
    melty_parameters->rotation_interval_us = MAX_TRACKING_ROTATION_INTERVAL_US;
  }

  unsigned long led_on_us = led_on_portion * melty_parameters->rotation_interval_us;
  unsigned long led_offset_us = led_offset_portion * melty_parameters->rotation_interval_us;

  //starts LED on time at point in rotation so it's "centered" on led offset
  melty_parameters->led_start = led_offset_us - (led_on_us / 2);
  if (melty_parameters->led_start < 0) {
    melty_parameters->led_start += melty_parameters->rotation_interval_us;
  }
  
  melty_parameters->led_stop = melty_parameters->led_start + led_on_us;
  if (melty_parameters->led_stop > melty_parameters->rotation_interval_us)
  {
    melty_parameters->led_stop -= melty_parameters->rotation_interval_us;
  }

  // phase transition timing: Brace yourself
  // One phase is always half of the rotation interval
  melty_parameters->phase_length_us = melty_parameters->rotation_interval_us / 2;

  // And now we need to talk about the difference between "forwards" as in "0", the start of each rotation and straight up on the control stick,
  // And "forwards" as in "heading", the direction we want the robot to travel in.
  // We're in motor phase 1 when wheel 1 is advancing towards the heading- it's when we run the throttle at higher power
  // And when wheel 2 is retreating from the heading, we're in phase 2, and run the throttle at lower power
  // So. Let's work out what the heading is.
  // We're going to represent this a little interestingly, for the sake of the hot loop later: Rather than record the heading angle as 0-360,
  // We're going to record what phase we're in at 0, and when, from 0 to half a rotation interval of microseconds, we will enter the next phase.
  // So, let's work out what that angle is first
  int fwds = rc_get_trans_forback();
  int lr = rc_get_trans_leftright();

  // if lr is 0 (so, the stick is straight forwards or straight back), the phase transition happens at exactly 0.
  double dir_rads = 0;

  // Otherwise, it's trig time
  if (lr != 0) {
    // This will give us dir_rads between -pi/2 and pi/2.
    dir_rads = atan((double) fwds/lr);
  }

  // convert to degrees
  double dir_degs = dir_rads * 180/PI;

  // and, if it's negative, add 180 degrees to find the first positive phase transition's angular offset
  if (dir_degs < 0) {
    dir_degs += 180;
  }

  // Which we can convert to micros easily:
  melty_parameters->phase_offset_us = dir_degs * melty_parameters->phase_length_us / 180;
 
  // That's great, but now we need to calculate what phase we're in at 0.
  // You might think there are two options here, but there's actually four!
  // If we're rotating clockwise, and the heading is to the right of 0, we start in phase 1, because wheel 1 is advancing as we pass 0.
  // If we're rotating clockwise and the heading is to the left of 0, we start in phase 2.
  // If we're rotating counter-clockwise, and the heading is to the right of 0, we're starting in phase 2 - because at 0, wheel 1 is retreating towards the phase transition opposite the heading
  // And if we're rotating counter-clockwise and the heading is to the left of 0, we start in phase 1
  // drawing the truth table for this crazy boolean is left as an exercise

  melty_parameters->start_in_phase_one = ((lr > 0) == ROTATION_IS_CLOCKWISE);

  // compared to all that, how much the throttle settings vary for each phase is easy to calculate.
  // 256 is a magic number - if we used 512, at exactly half throttle and stick hard over, the advancing wheel would be running at 1023 and the withdrawing wheel at 0
  // Which would be super neat and all, but it just doesn't feel responsive enough
  // So we doubled the influence that stick angle has
  int translate_disp = max(abs(fwds), abs(lr));
  melty_parameters->throttle_high_perk = min(melty_parameters->throttle_perk + (melty_parameters->movement_enabled * translate_disp * melty_parameters->throttle_perk / 256), 1023);
  melty_parameters->throttle_low_perk = max(melty_parameters->throttle_perk - (melty_parameters->movement_enabled * translate_disp * melty_parameters->throttle_perk / 256 ), 0);

  //if the battery voltage is low - shimmer the LED to let user know
#ifdef BATTERY_ALERT_ENABLED
  if (battery_voltage_low() == true) melty_parameters->led_shimmer = 1;
#endif

}

//turns on heading LED at appropriate timing
static void update_heading_led(struct melty_parameters_t melty_parameters, unsigned long time_spent_this_rotation_us) {
  if (melty_parameters.led_start > melty_parameters.led_stop) {
    if (time_spent_this_rotation_us >= melty_parameters.led_start || time_spent_this_rotation_us <= melty_parameters.led_stop) {
      heading_led_on(melty_parameters.led_shimmer);
    } else {
      heading_led_off();
    }
  } else {
    if (time_spent_this_rotation_us >= melty_parameters.led_start && time_spent_this_rotation_us <= melty_parameters.led_stop) {
      heading_led_on(melty_parameters.led_shimmer);
    } else {
      heading_led_off();
    }
  }
}

//rotates the robot once + handles translational drift
//(repeat as needed)
void spin_one_rotation(void) {

  //-initial- assignment of melty parameters
  static struct melty_parameters_t melty_parameters;
  get_melty_parameters(&melty_parameters);

  //capture initial time stamp before rotation start (time performing accel sampling / floating point math is included)
  unsigned long start_time = micros();
  unsigned long time_spent_this_rotation_us = 0;

  //tracking cycle count is needed to alternate cycles for non-translation (overflow is non-issue)
  static unsigned long cycle_count = 0;
  cycle_count++;

  //the melty parameters are updated either at the beginning of the rotation - or the middle of the rotation (alternating each time)
  //this is done so that any errors due to the ~1ms accel read / math cycle cancel out any effect on tracking / translational drift
  int melty_parameter_update_time_offset_us = 0;
  if (cycle_count % 2 == 1) melty_parameter_update_time_offset_us = melty_parameters.rotation_interval_us / 2;  
  bool melty_parameters_updated_this_rotation = false;

  //loop for one rotation of robot
  while (time_spent_this_rotation_us < melty_parameters.rotation_interval_us) {

    //update melty parameters if we haven't / update time has elapsed
    if (melty_parameters_updated_this_rotation == false && time_spent_this_rotation_us > melty_parameter_update_time_offset_us) { 
      get_melty_parameters(&melty_parameters);
      melty_parameters_updated_this_rotation = true;
    }

    //translate
    // So. What phase are we in?
    // This can be answered by determining how many phase transitions have passed since we started this rotation
    // We're adding 1 because the complicated math starts at -1, for annoying integer reasons.
    int phase_transitions_passed = 1 + (time_spent_this_rotation_us - melty_parameters.phase_offset_us) / melty_parameters.phase_length_us;

    // Now, if we've passed one phase transition, we're in the opposite of the phase we passed at time 0
    // If we've passed 0 or 2 phase transitions, we're in the same phase as time 0
    // And we know which phase time 0 is in, thanks to earlier math!
    // once again, we're in an xor situation- we're in phase one either if time 0 is in phase one and we've passed 0 or 2 phase transitions, or if time 0 is in phase 2 and we've passed one phase transition.
    if (melty_parameters.start_in_phase_one == (phase_transitions_passed != 1)) {
      // We're in phase one! Spin motor one faster
      motors_on(melty_parameters.throttle_high_perk, melty_parameters.throttle_low_perk);
    } else {
      // We're in phase two! Spin motor two faster
      motors_on(melty_parameters.throttle_low_perk, melty_parameters.throttle_high_perk);
    }
   
    //displays heading LED at correct location
    update_heading_led(melty_parameters, time_spent_this_rotation_us);

    time_spent_this_rotation_us = micros() - start_time;

  }

}