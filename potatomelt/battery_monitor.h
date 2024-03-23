//check for low battery - but only alarm after certain number of low reads in a row (prevents ADC noise from alarming)
//(settings in melty_config.h)
bool battery_voltage_low();

//check for critically low battery- the stop-the-robot level bad
bool battery_voltage_crit();

//returns battery voltage
float get_battery_voltage();