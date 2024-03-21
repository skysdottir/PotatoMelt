// Tank controls!

#define TANK_FORBACK_POWER_SCALE 0.02f; // We're just going to scale inputs down, because we're sitting on a pair of ungeared brushless motors
#define TANK_TURNING_POWER_SCALE 0.005f; // And we're doubly going to turn down rotation, because this bot was built to spin

void handle_tank_mode();

