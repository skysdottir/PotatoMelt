
//does translational drift rotation (robot spins 360 degrees)
void spin_one_rotation(void);

// Stop spinning!
void disable_spin();

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

//sets up the timer interrupt for melty drive hot loop
void init_spin_timer();

//holds melty parameters used to determine timing for current spin cycle
//all time offsets are in microseconds

typedef struct melty_parameters_t {
  bool spin_enabled;                  // Authorization for the hot loop to spin
  int translation_enabled;            // Authorization for the spinning bot to translate
  int throttle_perk;                  //stores throttle out of 0-1024
  int throttle_high_dshot;            // for translation, the approaching wheel power, as dshot expects it
  int throttle_low_dshot;             // for translation, the receeding wheel power, as dshot expects it
	unsigned long rotation_interval_us; //time for 1 rotation of robot
	unsigned long led_start;            //offset for beginning of LED beacon
	unsigned long led_stop;             //offset for end of LED beacon
	unsigned long motor_start_phase_1;  //time offset for when motor 1 begins translating forwards
	unsigned long motor_start_phase_2;  //time offset for when motor 2 begins translating forwards

  int led_shimmer;                    //LED is shimmering to indicate something to the user
};