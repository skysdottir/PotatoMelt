#include "melty_config.h"

#ifdef ENABLE_EEPROM_STORAGE

//retrieves the accelerometer correction table from EEPROM
int load_correction_table(float *table);

void save_correction_table(float *table, int);

//retrieves zero G offset from EEPROM
float load_accel_zero_g_offset();

//save zero G offset to EEPROM
void save_accel_zero_g_offset(float offset);

//retrieves LED offset from EEPROM
int load_heading_led_offset();

//saves LED offset to EEPROM
void save_heading_led_offset(int offset);

void eeprom_write_sentinel();

#endif