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
#define LEFT_RIGHT_CONFIG_LED_ADJUST_DIVISOR 0.2f         //How quick LED heading is adjusted in config mode (larger values = slower)

#define MAX_TRANSLATION_ROTATION_INTERVAL_US (1.0f / MIN_TRANSLATION_RPM) * 60 * 1000 * 1000
#define MAX_TRACKING_ROTATION_INTERVAL_US MAX_TRANSLATION_ROTATION_INTERVAL_US * 2   //don't track heading if we are this slow (also puts upper limit on time spent in melty loop for safety)

static float accel_mount_radius_cm = DEFAULT_ACCEL_MOUNT_RADIUS_CM;
static float accel_zero_g_offset = DEFAULT_ACCEL_ZERO_G_OFFSET;
static float led_offset_percent = DEFAULT_LED_OFFSET_PERCENT;         //stored in EEPROM as an INT - but handled as a float for configuration purposes

static unsigned int highest_rpm = 0;
static bool config_mode = false;   //1 if we are in config mode

//-initial- assignment of melty parameters
melty_parameters_t melty_parameters;

// Globals for spin state - when did we start spinning and how long have we been spinning for?
unsigned long start_time;
unsigned long time_spent_this_rotation_us = 0;

// Global for acceleration/deceleration estimation
unsigned long last_rotation_instant_time = 0;

void init_spin_timer() {
    // TIMER 3 for interrupt frequency 2000 Hz:
  cli(); // stop interrupts
  TCCR3A = 0; // set entire TCCR1A register to 0
  TCCR3B = 0; // same for TCCR1B
  TCNT3  = 0; // initialize counter value to 0
  // set compare match register for 2000 Hz increments
  OCR3A = 7999; // = 16000000 / (1 * 2000) - 1 (must be <65536)
  // turn on CTC mode
  TCCR3B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR3B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);
  sei(); // allow interrupts
}

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
static unsigned long get_rotation_interval_us(int steering_enabled) {
  
  float radius_adjustment_factor = 0;

  //don't adjust steering if disabled by config mode - or we are in RC deadzone
  if (steering_enabled == 1 && rc_get_is_lr_in_normal_deadzone() == false) {
    radius_adjustment_factor = (float)(rc_get_leftright() / (float)NOMINAL_PULSE_RANGE) / LEFT_RIGHT_HEADING_CONTROL_DIVISOR;
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

  // How fast it'll take us to spin if we don't accelerate or decelerate
  unsigned long instant_rotation_interval = (1.0f / rpm) * 60 * 1000 * 1000;

  if (last_rotation_instant_time == 0) {
    last_rotation_instant_time = instant_rotation_interval;
  }

  // And now some linear estimation. We know our previous instantaneous rotation rate, we know our current instantaneous rotation rate,
  // which means we can gueess our next instantaneous rotation rate linearily, with next = current + (current-previous)
  // But we want average rotation rate instead, which is to say, (current+next)/2
  // Algebra them together and you get...
  unsigned long predicted_rotation_rate = (3*instant_rotation_interval - last_rotation_instant_time)/2;

  last_rotation_instant_time = instant_rotation_interval;
  return predicted_rotation_rate;
}


//performs changes to melty parameters when in config mode
static struct melty_parameters_t handle_config_mode(melty_parameters_t *melty_parameters) {

  int forback = rc_get_forback_bit();
  //if forback forward - normal drive (for driver testing - no adjustment of melty parameters)

  //if forback neutral - then do radius adjustment
  if (forback == RC_FORBACK_NEUTRAL) {

    //radius adjustment overrides steering
    melty_parameters->translation_enabled = 0;

    //only adjust if stick is outside deadzone    
    if (rc_get_is_lr_in_config_deadzone() == false) {
      //show that we are changing config
      melty_parameters->led_shimmer = 1;

      float adjustment_factor = (accel_mount_radius_cm * (float)(rc_get_leftright() / (float)NOMINAL_PULSE_RANGE));
      adjustment_factor = adjustment_factor / LEFT_RIGHT_CONFIG_RADIUS_ADJUST_DIVISOR;
      accel_mount_radius_cm = accel_mount_radius_cm + adjustment_factor;

      if (accel_mount_radius_cm < ACCEL_MOUNT_RADIUS_MINIMUM_CM) accel_mount_radius_cm = ACCEL_MOUNT_RADIUS_MINIMUM_CM;
    }    
  }
  
  //if forback backward - do LED heading adjustment (don't translate)
  if (forback == RC_FORBACK_BACKWARD) {
    //LED heading offset adjustment disables movement
    melty_parameters->translation_enabled = 0;
    
    //only adjust if stick is outside deadzone  
    if (rc_get_is_lr_in_config_deadzone() == false) {

      //show that we are changing config
      melty_parameters->led_shimmer = 1;

      float adjustment_factor =  (float)(rc_get_leftright() / (float)NOMINAL_PULSE_RANGE);
      adjustment_factor = adjustment_factor / LEFT_RIGHT_CONFIG_LED_ADJUST_DIVISOR;
      led_offset_percent = led_offset_percent + adjustment_factor;

      if (led_offset_percent > 99) led_offset_percent = 0;
      if (led_offset_percent < 0) led_offset_percent = 99;

    }
  }
}

//Calculates all parameters need for a single rotation (motor timing, LED timing, etc.)
//This entire section takes ~1300us on an Atmega32u4
//Which means it will be interrupted by the hot loop multiple times
//Fortunately, once the bot is spinning, it spins fast enough that no human is going to move the controls significantly in a single loop
//So, partially stale data for one update isn't going to break anything
//Todo: Rethink this assumption when we get into omnidirectional maneuvering, because then there'll be a pole right in the middle of the deadzone
static void get_melty_parameters(melty_parameters_t *melty_parameters) {
  // set some of the defaults
  melty_parameters->led_shimmer = 0;
  melty_parameters->translation_enabled = 1;

  float led_offset_portion = led_offset_percent / 100.0f;

  int throttle_perk = rc_get_throttle_perk();

  //if we are in config mode - handle it (and disable steering if needed)
  if (get_config_mode() == true) {
    handle_config_mode(melty_parameters);
  }

  melty_parameters->rotation_interval_us = get_rotation_interval_us(melty_parameters->translation_enabled);

  //if we are too slow - don't even try to track heading
  if (melty_parameters->rotation_interval_us > MAX_TRACKING_ROTATION_INTERVAL_US) {
    melty_parameters->rotation_interval_us = MAX_TRACKING_ROTATION_INTERVAL_US;
  }

  //LED width changes with RPM - Totally spitballing ratios here
  float led_on_portion = 16000.0f / melty_parameters->rotation_interval_us;  
  if (led_on_portion < 0.10f) led_on_portion = 0.10f;
  if (led_on_portion > 0.90f) led_on_portion = 0.90f;

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

  // phase transition timing: Currently, only forwards/backwards
  melty_parameters->motor_start_phase_1 = 0;
  melty_parameters->motor_start_phase_2 = melty_parameters->rotation_interval_us / 2;

  int translate_disp = rc_get_forback_trans();
  // translation control!
  // Because there's a lot of math here, we're going to compute the actual dshot commands once
  // So then in the hot loop we can just spam the known codes
  int throttle_high_perk = min(throttle_perk + (melty_parameters->translation_enabled * translate_disp * throttle_perk / 128), 1023);
  int throttle_low_perk = max(throttle_perk - (melty_parameters->translation_enabled * translate_disp * throttle_perk / 128), 0);

  int motor_dir = rc_get_spin_dir();

  throttle_high_perk *= motor_dir;
  throttle_low_perk *= motor_dir;

  melty_parameters->throttle_high_dshot = perk2dshot(throttle_high_perk);
  melty_parameters->throttle_low_dshot = perk2dshot(throttle_low_perk);

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

// Cut the motors!
void disable_spin() {
  motors_off();
  melty_parameters.spin_enabled = false;
}

//rotates the robot once + handles translational drift
//(repeat as needed)
void spin_one_rotation(void) {
  get_melty_parameters(&melty_parameters);

  if (!melty_parameters.spin_enabled) {
    start_time = micros();
    melty_parameters.spin_enabled = true;
  }

  delayMicroseconds(melty_parameters.rotation_interval_us - 2048);
}

// The hot loop
ISR(TIMER3_COMPA_vect) {
  // fast bail if we aren't supposed to be spinning.
  // disable_spin() already stopped the motors, so we can just return
  if (!melty_parameters.spin_enabled) {
    return;
  }

  time_spent_this_rotation_us = micros() - start_time;
  
  if (time_spent_this_rotation_us > melty_parameters.rotation_interval_us) {
    time_spent_this_rotation_us -= melty_parameters.rotation_interval_us;
    start_time += melty_parameters.rotation_interval_us;
  }

  //translate
  if (time_spent_this_rotation_us >= melty_parameters.motor_start_phase_1 && time_spent_this_rotation_us <= melty_parameters.motor_start_phase_2) {
    motors_on_direct(melty_parameters.throttle_high_dshot, melty_parameters.throttle_low_dshot);
  } else {
    motors_on_direct(melty_parameters.throttle_low_dshot, melty_parameters.throttle_high_dshot);
  }
   
    //displays heading LED at correct location
    update_heading_led(melty_parameters, time_spent_this_rotation_us);
}