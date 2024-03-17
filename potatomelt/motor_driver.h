//intitialize motors
void init_motors();


// Motors on!
void motors_on(int motor_one_throttle_perk, int motor_two_throttle_perk);

//motors shut-down (robot not translating)
void motor_1_off();
void motor_2_off();
void motors_off();

//motors coasting (unpowered part of rotation when translating)
void motor_1_coast();
void motor_2_coast();