//handles persistent storage of melty brain config (accelerometer radius, LED offset and accelerometer 0g offset)
//this code (unfortunately) depends on the AVR-specific EEPROM library:
//https://docs.arduino.cc/learn/built-in-libraries/eeprom

//if using on an ARM-based arduino - either just disable this (and hard code your settings in melty_config.h)
//...or you can try porting this functionality to ARM using the FlashStorage library:
//https://reference.arduino.cc/reference/en/libraries/flashstorage/

#include "melty_config.h"

#ifdef ENABLE_EEPROM_STORAGE

#include <EEPROM.h>
#include <arduino.h>
#include "config_storage.h"

#define EEPROM_WRITTEN_SENTINEL1_VALUE EEPROM_WRITTEN_SENTINEL_VALUE
#define EEPROM_WRITTEN_SENTINEL2_VALUE EEPROM_WRITTEN_SENTINEL_VALUE

#define EEPROM_WRITTEN_SENTINEL_BYTE1_LOC 0
#define EEPROM_WRITTEN_SENTINEL_BYTE2_LOC 1

#define EEPROM_HEADING_LED_LOC 2

//bytes for float
#define EEPROM_ACCEL_RADIUS_BYTE1_LOC 3
#define EEPROM_ACCEL_RADIUS_BYTE2_LOC 4
#define EEPROM_ACCEL_RADIUS_BYTE3_LOC 5
#define EEPROM_ACCEL_RADIUS_BYTE4_LOC 6

//bytes for float
#define EEPROM_ACCEL_OFFSET_BYTE1_LOC 7
#define EEPROM_ACCEL_OFFSET_BYTE2_LOC 8
#define EEPROM_ACCEL_OFFSET_BYTE3_LOC 9
#define EEPROM_ACCEL_OFFSET_BYTE4_LOC 10

#define EEPROM_RPM_CORRECTION_LENGTH_LOC 11

//start bytes for the lookup table
#define EEPROM_RPM_CORRECTION_BYTE1_LOC 12

//indicates values saved to EEPROM (doing 2x)
static void eeprom_write_sentinel() {
  EEPROM.write(EEPROM_WRITTEN_SENTINEL_BYTE1_LOC, EEPROM_WRITTEN_SENTINEL1_VALUE);
  EEPROM.write(EEPROM_WRITTEN_SENTINEL_BYTE2_LOC, EEPROM_WRITTEN_SENTINEL2_VALUE);
}

//make sure we actually wrote values before trying to read... (doing 2x)
//if EEPROM_WRITTEN_SENTINEL_VALUE is changed - will cause EEPROM values to invalidate / use default values 
static int check_sentinel() {
  if (EEPROM.read(EEPROM_WRITTEN_SENTINEL_BYTE1_LOC) != EEPROM_WRITTEN_SENTINEL1_VALUE) return 0;
  if (EEPROM.read(EEPROM_WRITTEN_SENTINEL_BYTE2_LOC) != EEPROM_WRITTEN_SENTINEL2_VALUE) return 0;
  return 1;
}

int load_correction_table(float *table) {
  if (check_sentinel() != 1) {
    for (int i = 0; i < 16; i++) {
      table[i] = 0;
    }
    return 14;
  }

  int len = EEPROM.read(EEPROM_RPM_CORRECTION_LENGTH_LOC);

  for (int i = 0; i < 16; i++) {
    int addr = EEPROM_RPM_CORRECTION_BYTE1_LOC + i*4;
    table[i] = EEPROM.get(addr, table[i]);
  }
}

void save_correction_table(float *table, int length) {

  for (int i = 0; i < 16; i++) {
    int addr = EEPROM_RPM_CORRECTION_BYTE1_LOC + i*4;
    EEPROM.write(addr, table[i]);
  }
}

int load_heading_led_offset() {
  //if value hasn't been saved previously - return the default
  if (check_sentinel() != 1) return DEFAULT_LED_OFFSET_PERCENT;
  return EEPROM.read(EEPROM_HEADING_LED_LOC);
}

void save_heading_led_offset(int offset) {
  EEPROM.write(EEPROM_HEADING_LED_LOC, offset);
}

float load_accel_zero_g_offset() {
  //if value hasn't been saved previously - return the default
  if (check_sentinel() != 1) return DEFAULT_ACCEL_ZERO_G_OFFSET;
  float accel_zero_g_offset;
  accel_zero_g_offset = EEPROM.get(EEPROM_ACCEL_OFFSET_BYTE1_LOC, accel_zero_g_offset);
  return accel_zero_g_offset;
}

void save_accel_zero_g_offset(float offset) {
  EEPROM.write(EEPROM_ACCEL_OFFSET_BYTE1_LOC, offset);
}

float load_accel_mount_radius() {
  //if value hasn't been saved previously - return the default
  if (check_sentinel() != 1) return DEFAULT_ACCEL_MOUNT_RADIUS_CM;
  float accel_radius;
  accel_radius = EEPROM.get(EEPROM_ACCEL_RADIUS_BYTE1_LOC, accel_radius);
  return accel_radius;
}

#endif