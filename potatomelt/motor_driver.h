//intitialize motors
void init_motors();

// Motors on!
// This is a polite function, you give it the throttle you want from -1024 to +1024 and it just works
void motors_on(int motor_one_throttle_perk, int motor_two_throttle_perk);

// Motors on, without any niceties: just put the input values straight into dshot
void motors_on_direct(int motor_one_throttle_val, int motor_two_throttle_val);

// Send a config command to both motors
void configure_motors(int config_code);

// And a conversion function to go from perK to the dshot code
int perk2dshot(int throttle_perk);

//motors shut-down (robot not translating)
void motors_off();