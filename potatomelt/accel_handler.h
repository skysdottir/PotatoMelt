

//Set high enough to allow for G forces at top RPM
//LOW_RANGE - +/-100g for the H3LIS331DH
//MED_RANGE - +/-200g for the H3LIS331DH
//HIGH_RANGE - +/-400g for the H3LIS331DH
#define ACCEL_RANGE LIS331::HIGH_RANGE   

//Set to correspond to ACCEL_RANGE
#define ACCEL_MAX_SCALE 400

//Change as needed as needed
//(Adafuit breakout default is 0x18, Sparkfun default is 0x19)
#define ACCEL_I2C_ADDRESS 0x18

void init_accel();

// Returns the raw G force we're experiencing
float get_accel_force_g();

// Returns the raw RPM, uncorrected- used for setting the correction factor
float get_uncorrected_rpm();

// Get the correction factor for a specified RPM
float get_correction_factor(float rpm);

// Set the correction factor for a specified RPM to the lookup table
void set_correction_factor(float rpm, float correction_factor);

// Blank the correction table for reconfiguration
void clear_correction_table();

// If the lookup table is full, evict existing entries one by one
void evict_record();

// Save accelerometer and correction params for future use
void save_parameters();

// Call when NOT SPINNING to set the zero offset in the accelerometer
void set_accel_zero_offset();