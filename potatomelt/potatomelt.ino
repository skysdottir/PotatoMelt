#include <SparkFun_LIS331.h>
#include <Arduino.h>

#define ENABLE_WATCHDOG

// See melty_config.h for configuration parameters

#include "rc_handler.h"
#include "melty_config.h"
#include "motor_driver.h"
#include "accel_handler.h"
#include "spin_control.h"
#include "config_storage.h"
#include "battery_monitor.h"

#ifdef ENABLE_TANK_MODE
#include "tank_control.h"
#endif

#ifdef ENABLE_WATCHDOG
#include <Adafruit_SleepyDog.h>
#endif

void service_watchdog() {
#ifdef ENABLE_WATCHDOG
    Watchdog.reset();
#endif
} 

// Arduino initial setup function
void setup() {
  
  Serial.begin(115200);

  // get motor drivers setup (and off!) first thing
  init_motors();
  init_led();

#ifdef ENABLE_WATCHDOG
  // returns actual watchdog timeout MS
  int watchdog_ms = Watchdog.enable(WATCH_DOG_TIMEOUT_MS);
#endif

  init_rc();
  init_accel();   // accelerometer uses i2c - which can fail blocking (so only initializing it -after- the watchdog is running)
  
// load settings on boot
#ifdef ENABLE_EEPROM_STORAGE  
  load_melty_config_settings();
#endif

init_pid();

// start the interrupt clock!
init_spin_timer();

}

// dumps out diagnostics info
static void echo_diagnostics() {
  Serial.print("Raw Accel G: "); Serial.print(get_accel_force_g());
  Serial.print("  RC Health: "); Serial.print(rc_signal_is_healthy());
  Serial.print("  RC Throttle: "); Serial.print(rc_get_throttle_perk());
  Serial.print("  RC L/R: "); Serial.print(rc_get_leftright());
  Serial.print("  RC F/B: "); Serial.print(rc_get_forback_trans());

#ifdef BATTERY_ALERT_ENABLED
  Serial.print("  Battery Voltage: "); Serial.print(get_battery_voltage());
#endif 
  
#ifdef ENABLE_EEPROM_STORAGE  
  Serial.print("  Heading Offset: "); Serial.print(load_heading_led_offset());
  Serial.print("  Zero G Offset: "); Serial.print(load_accel_zero_g_offset());
#endif  
  Serial.println("");

}

// checks if user has requested to enter / exit config mode
static void check_config_mode() {
  // if user pulls control stick back for 750ms - enters (or exits) interactive configuration mode
  if (rc_get_forback_bit() == RC_FORBACK_BACKWARD) {
    delay(750);
    if (rc_get_forback_bit() == RC_FORBACK_BACKWARD) {
      Serial.println("Entering config mode");
      toggle_config_mode(); 
      if (get_config_mode() == false) save_melty_config_settings();    // save melty settings on config mode exit
      
      // wait for user to release stick - so we don't re-toggle modes
      while (rc_get_forback_bit() == RC_FORBACK_BACKWARD) {
        rc_poll();
        service_watchdog();
      }
    }
  }    
}

static void check_accel_config_clear()
{
  if (get_config_mode() && rc_get_accel_save()) {
    delay(750);
    rc_poll();
    if (rc_get_accel_save()) {
      Serial.println("Clearing accelerometer correction table");
      clear_correction_table();
    }
  }
}

// handles the bot when not spinning (with RC good)
static void handle_bot_idle() {

    disable_spin();              // assure motors are off
    
    if (get_config_mode() == true) {
      set_led_pattern(CONFIG);
    } else {
      set_led_pattern(READY);
    }

    check_config_mode();          // check if user requests we enter / exit config mode
    check_accel_config_clear();

    echo_diagnostics();           // echo diagnostics if bot is idle
}

static void handle_battery_crit() {
  disable_spin();
    
  set_led_pattern(BATTERY);

  rc_poll();
  service_watchdog();
  echo_diagnostics();
}

// main control loop
void loop() {

  // keep the watchdog happy
  service_watchdog();

  // if JUST_DO_DIAGNOSTIC_LOOP - then we just loop and display debug info via USB (good for testing)
#ifdef JUST_DO_DIAGNOSTIC_LOOP
    rc_poll();
    disable_spin();
    echo_diagnostics();
    delay(250);   //delay prevents serial from getting flooded (can cause issues programming)
    return;
#endif

  #ifdef BATTERY_CRIT_HALT_ENABLED
  if(battery_voltage_crit())
  {
    handle_battery_crit();
    return;
  }
  #endif

  // fast-fail if there's no new RC data to work from
  if (!rc_poll()) {
    return;
  }

  // if the rc signal isn't good - assure motors off - and "slow flash" LED
  // this will interrupt a spun-up bot if the signal becomes bad
  if (rc_signal_is_healthy() == false) {
    disable_spin();
    
    set_led_pattern(LOS);
    
    // echo diagnostics while we are waiting for RC signal
    echo_diagnostics();

    // And then bail
    return;
  }

  #ifdef ENABLE_TANK_MODE
  // If we're in tank mode, go drive like a tank!
  if (rc_get_tank_mode()) {
    disable_spin();
    handle_tank_mode();
    return;
  }
  #endif

  // if RC is good - and throtte is above 0 - spin a single rotation
 if (rc_get_throttle_perk() > 0) {
    // this is where all the motor control happens!  (see spin_control.cpp)
    enable_spin();
    spin_one_iteration();  
  } else {    
    handle_bot_idle();
  }
}