#include "led_driver.h"

//does translational drift rotation (robot spins and computes updated parameters)
void spin_one_iteration(void);

// Stop spinning!
void disable_spin();

// Allow spinning! If the throttle's at 0, the bot still won't spin.
void enable_spin();

//returns maximum rotation speed since last entering config mode
int get_max_rpm();

//toggles configuration mode
void toggle_config_mode();

//returns true if in configuration mode
bool get_config_mode();

//triggers load of melty parameters
void load_melty_config_settings();

//saves melty parameters
void save_melty_config_settings();

// Update the blinky pattern for when we aren't spinning
void set_led_pattern(LED_Pattern pattern);

//sets up the timer interrupt for melty drive hot loop
void init_spin_timer();

//sets up the PID controller - most notably, sets the output limits to (0, 1023);
void init_pid();

//holds melty parameters used to determine timing for current spin cycle
//all time offsets are in microseconds

typedef struct melty_parameters_t {
  LED_Pattern led_pattern;               // Current status LED state 
  bool spin_enabled;                  // Authorization for the hot loop to spin
  int translation_enabled;            // Authorization for the spinning bot to translate
  int throttle_perk;                  //stores throttle out of 0-1024
  int max_throttle_offset;            //In a rotation, the furthest from the base throttle setting that each motor might be spun
	unsigned long rotation_interval_us; //time for 1 rotation of robot
	unsigned long led_start;            //offset for beginning of LED beacon
	unsigned long led_stop;             //offset for end of LED beacon
	unsigned long motor_start_phase_1;  //time offset for when motor 1 begins translating forwards
	unsigned long motor_start_phase_2;  //time offset for when motor 2 begins translating forwards

  int led_shimmer;                    //LED is shimmering to indicate something to the user
};