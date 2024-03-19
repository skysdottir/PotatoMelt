
//does translational drift rotation (robot spins 360 degrees)
void spin_one_rotation(void);

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

//holds melty parameters used to determine timing for current spin cycle
//all time offsets are in microseconds

typedef struct melty_parameters_t {
  int translate_forback;              //RC_FORBACK_FORWARD, RC_FORBACK_BACKWARD, RC_FORBACK_NETURAL
  int throttle_perk;                  //stores throttle out of 0-1024
  int throttle_high_perk;             // for translation, the approaching wheel power
  int throttle_low_perk;              // for translation, the receeding wheel power
	unsigned long rotation_interval_us; //time for 1 rotation of robot
  unsigned long phase_length_us;      // time for one phase of the rotation, 1/2 of rotation_interval_us
  unsigned long phase_offset_us;      // time from 0 to the first phase transition
  bool start_in_phase_one;            // do we start with wheel one advancing or retreating?
	unsigned long led_start;            //offset for beginning of LED beacon
	unsigned long led_stop;             //offset for end of LED beacon
  int movement_enabled;              //Prevents adjustment of left / right heading adjustment (used for configuration mode)
  int led_shimmer;                    //LED is shimmering to indicate something to the user
};