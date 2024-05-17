//this module interfaces with the accelerometer to provide current G-force level

//Two breakout boards using the H3LIS331DL accelerometer have been tested / verified to work

//Sparkfun's H3LIS331DL breakout: https://www.sparkfun.com/products/14480 (3v part)
//Requires using  to interface with 5v arduino

//Adafruit's HSLIS311 breakout includes a level converter - is a single component solution:
//https://www.adafruit.com/product/4627

//Update ACCEL_I2C_ADDRESS in accel_handler.h with I2c address (Adafruit and Sparkfun boards are different!)

//Sparkfun's LIS311 library is used (also works for Adafruit part)

#include <arduino.h>
#include "melty_config.h"
#include "accel_handler.h"
#include "config_storage.h"
#include <Wire.h>
#include <SparkFun_LIS331.h>

LIS331 xl;

static float accel_zero_g_offset = DEFAULT_ACCEL_ZERO_G_OFFSET;
static float assume_radius_in_cm = DEFAULT_ACCEL_MOUNT_RADIUS_CM;

// 8 pairs of (raw RPM, correction factor)
static float correction_lookup_table[16];

// Stores the length of the correction table so we don't have to iterate through it every time
// Note that this is going to be used as an index, so it goes 0-16 by 2s
static int correction_table_length = 0;

// If we're overwriting an existing lookup table,
// we'll evict the existing records one by one
// starting with the first
static int ring_buffer_position = 0;

void init_accel() { 
  #ifdef ENABLE_EEPROM_STORAGE 
    correction_table_length = load_correction_table(correction_lookup_table);
  #endif

  Wire.begin();
  
  Wire.setClock(400000);  //increase I2C speed to reduce read times a bit
                          //value of 400000 allows accel read in ~1ms and
                          //is verfied to work with Sparkfun level converter
                          //(some level converters have issues at higher speeds)
  
  xl.setI2CAddr(ACCEL_I2C_ADDRESS);
  xl.begin(LIS331::USE_I2C);

  //sets accelerometer to specified scale (100, 200, 400g)
  xl.setFullScale(ACCEL_RANGE);
}

//reads accel and converts to G's
//ACCEL_MAX_SCALE needs to match ACCEL_RANGE value
float get_accel_force_g() {
  int16_t x, y, z;
  xl.readAxes(x, y, z);
  return xl.convertToG(ACCEL_MAX_SCALE,y);
}

// Returns the raw RPM, uncorrected- used for setting the correction factor
float get_uncorrected_rpm() {
  float rpm;
  //use of absolute makes it so we don't need to worry about accel orientation
  //calculate RPM from g's - derived from "G = 0.00001118 * r * RPM^2"
  rpm = fabs(get_accel_force_g() - accel_zero_g_offset) * 89445.0f;
  rpm = rpm / assume_radius_in_cm;
  rpm = sqrt(rpm);

  return rpm;
}

// Get the correction factor for a specified RPM
float get_correction_factor(float rpm) {
  
  // no table, no correction
  if(correction_table_length == 0) {
    return 0;
  }

  // Find the first lookup table record higher than RPM,
  // Stop on the final record
  int i = 0;
  while (i < correction_table_length && correction_lookup_table[i] < rpm) {
    i += 2;
  }

  if (i == 0) {
    return correction_lookup_table[1];
  }

  // we're at the far end of the lookup table, reply with the end entry
  if (i == correction_table_length) {
    return correction_lookup_table[i-1];
  }

  // otherwise, lirp. i points at the first value higher than rpm, so we're lirping between [i-2] and [i]

  // If we happen to have two points at exactly the same value, don't div/0
  if(correction_lookup_table[i-2]==correction_lookup_table[i]) {
    return correction_lookup_table[i-1];
  }

  float fac = (rpm - correction_lookup_table[i-2])/(correction_lookup_table[i]-correction_lookup_table[i-2]);
  return correction_lookup_table[i-1] + (fac * (correction_lookup_table[i+1]-correction_lookup_table[i-1]));
}

// Set the correction factor for a specified RPM to the lookup table
void set_correction_factor(float rpm, float correction_factor) {
  if (correction_table_length == 16) {
    evict_record();
  }
  
  // Sorted insert
  // Yes, yes, it's o(n^2)
  // The table is 8 entries long, it's fine
  
  // pos is always pointing at a blank (or duplicate) entry at the start of a loop
  int pos = correction_table_length;

  while(pos > 0 && rpm < correction_lookup_table[pos-2]) {
    correction_lookup_table[pos] = correction_lookup_table[pos-2];
    correction_lookup_table[pos+1] = correction_lookup_table[pos-1];
    pos -= 2;
  }

  correction_lookup_table[pos] = rpm;
  correction_lookup_table[pos+1] = correction_factor;

  correction_table_length += 2;
}

void evict_record() {
  if (ring_buffer_position >= correction_table_length) {
    ring_buffer_position = 0;
  }
  // fill down
  for (int i = ring_buffer_position; i < correction_table_length; i++) {
    correction_lookup_table[i] = correction_lookup_table[i+2];
    correction_lookup_table[i+1] = correction_lookup_table[i+3];
  }

  // and clear the final entry
  correction_lookup_table[correction_table_length] = 0;
  correction_lookup_table[correction_table_length + 1] = 0;

  // and adjust parameters
  correction_table_length -= 2;
  ring_buffer_position += 2;
}

void clear_correction_table() {
  for(int i = 0; i < 16; i++) {
    correction_lookup_table[i] = 0;
  }

  correction_table_length=0;
  ring_buffer_position = 0;
}

// Sort the lookup table and store it in EEPROM for future use
void save_parameters() {
  save_accel_zero_g_offset(accel_zero_g_offset);
  save_correction_table(correction_lookup_table, correction_table_length);
  eeprom_write_sentinel();
}

void set_accel_zero_offset() {
  int offset_samples = 200;
  for (int accel_sample_loop = 0; accel_sample_loop < offset_samples; accel_sample_loop ++) {
    accel_zero_g_offset += get_accel_force_g();
  }
  accel_zero_g_offset = accel_zero_g_offset / offset_samples;
}