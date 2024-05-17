
//used to return forward / back control stick position
typedef enum {
    RC_FORBACK_FORWARD = 1,     //control stick pushed forward
    RC_FORBACK_NEUTRAL = 0,     //control stick neutral
    RC_FORBACK_BACKWARD = -1     //control stick held back
} rc_forback;

void init_rc();

//return true if RC signal looks good
bool rc_signal_is_healthy();         

// Compute a checksum for the current position of the sticks, to use in the previous
unsigned long compute_checksum();

//returns (0,1023) value indicating throttle level
int rc_get_throttle_perk();

//returns (-512,512) value indicating displacement of drive stick forback
int rc_get_forback_trans();

//returns RC_FORBACK_FORWARD, RC_FORBACK_NEUTRAL or RC_FORBACK_BACKWARD depending on stick position
rc_forback rc_get_forback_bit();

//returns (-512,512) from center value (not converted to percentage)
int rc_get_leftright();

// Returns spin direction multiplier
int rc_get_spin_dir();

//these functions return true if L/R stick movement is below defined thresholds
bool rc_get_is_lr_in_config_deadzone();  
bool rc_get_is_lr_in_normal_deadzone();

// Returns true if the switch is active to put the bot in tank mode
bool rc_get_tank_mode();

// returns true if the accelerometer correction save button is pushed
bool rc_get_accel_save();

// Returns (1, 16) for translation trim - adjusts how hard the bot tries to translate
float rc_get_trans_trim();

// Channel assignments
// Note that the first channel is 0 in Ibus.cpp, while most controllers start at channel 1
// So channel 0 here is channel 1 on the controller, and so on
#define RC_CHANNEL_TURN 0                   // Logical turning - usually left-right on the right stick
#define RC_CHANNEL_FORBACK 1                // Translate forwards or backwards - usually forwards-backwards on the right stick
#define RC_CHANNEL_THROTTLE 2               // Spin RPM - usually forwards-backwards on the left stick
// reserved                                 // RC channel 3 is reserved for future sideways translation
#define RC_CHANNEL_TANKMODE 4               // Tank mode switch - usually a switch, <0 for melty mode, >0 for tank mode
#define RC_CHANNEL_ACCEL_OFFSET_SAVE 5      // Button for saving an accelerometer offset in config mode - <0 normally, >0 when pressed
#define RC_CHANNEL_SPIN_DIR 6               // Spin direction switch - runs motors forwards when >0, backwards when <0. Does not adjust heading LED offsets - useful when ESCs are wired backwards
#define RC_CHANNEL_TRANSLATION_TRIM 7       // Translate power trim - Usually a knob, more trim = robot will try harder to translate while spinning.

#define DEFAULT_TRANSLATION_TRIM 2.0;       // The default translation trim, if the feature is disabled in melty_config.h

// All pulse lengths in microseconds
// it's accepted that a TX with fully centered trims may produce values somewhat off these numbers

// RC pulses outside this range are considered invalid (indicate a bad RC signal)
#define MAX_RC_PULSE_LENGTH 2012
#define MIN_RC_PULSE_LENGTH 988

// This value reflects nominal range of possible RC pulse values (maximum - minimum)
// This value is used to help scale left / right adjustment of heading
// (does not need to be perfect)
#define NOMINAL_PULSE_RANGE (MAX_RC_PULSE_LENGTH - MIN_RC_PULSE_LENGTH)

#define IDLE_THROTTLE_PULSE_LENGTH 988            // pulses below this value are considered 0% throttle
#define FULL_THROTTLE_PULSE_LENGTH 2012           // pulses above this value are considered 100%
#define CENTER_LEFTRIGHT_PULSE_LENGTH 1500        // center value for left / right
#define CENTER_FORBACK_PULSE_LENGTH 1500          // center value for for / back

#define FORBACK_MIN_THRESH_PULSE_LENGTH 100       // pulse length must differ by this much from CENTER_FORBACK_PULSE_LENGTH to be considered going forward or back

#define LR_CONFIG_MODE_DEADZONE_WIDTH 100         // deadzone for LR when in config mode (in US) - prevents unintended tracking adjustments
#define LR_NORMAL_DEADZONE_WIDTH 25               // deadzone for normal drive - can help with unintentional drift when moving forward / back