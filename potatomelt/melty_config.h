//This file is intended to include all commonly modified settings for Open Melt

#ifndef MELTY_CONFIG_GUARD  //header guard
#define MELTY_CONFIG_GUARD

//----------AVR SPECIFIC FUNCTIONALITY----------
//This code has not been tested on ARM / non-AVR Arduinos (but may work)
//Doesn't currently support persistent config storage for non-AVR Arduinos (see config_storage.cpp for details)
//490Hz PWM-throttle behavior is specific to Atmega32u4 (see below)

//----------DIAGNOSTICS----------
//#define JUST_DO_DIAGNOSTIC_LOOP                 //Disables the robot / just displays config / battery voltage / RC info via serial

//----------EEPROM----------
#define ENABLE_EEPROM_STORAGE                     //Comment out this to disable EEPROM (for ARM)
#define EEPROM_WRITTEN_SENTINEL_VALUE 42          //Changing this value will cause existing EEPROM values to be invalidated (revert to defaults)

//----------TRANSLATIONAL DRIFT SETTINGS----------
//"DEFAULT" values are overriden by interactive config / stored in EEPROM (interactive config will be easier if they are about correct)
//To force these values to take effect after interactive config - increment EEPROM_WRITTEN_SENTINEL_VALUE
#define DEFAULT_ACCEL_MOUNT_RADIUS_CM 3.17        //Radius of accelerometer from center of robot
#define DEFAULT_LED_OFFSET_PERCENT 7              //Adjust to make heading LED line up with direction robot travels 0-99 (increasing moves beacon clockwise)
                                                   
#define DEFAULT_ACCEL_ZERO_G_OFFSET 0.0f          //Value accelerometer returns with robot at rest (in G) - adjusts for any offset
                                                  //H3LIS331 claims +/-1g DC offset - typical - but +/-2.5 has been observed at +/-400g setting (enough to cause tracking error)
                                                  //Just enterring and exiting config mode will automatically set this value / save to EEPROM (based on current accel reading reflecting 0g)
                                                  //For small-radius bots - try changing to H3LIS331 to +/-200g range for improved accuracy (accel_handler.h)

#define LEFT_RIGHT_HEADING_CONTROL_DIVISOR 1.0f   //How quick steering is (larger values = slower)

#define MIN_TRANSLATION_RPM 400                   //full power spin in below this number (increasing can reduce spin-up time)


//----------PIN MAPPINGS----------
//RC pins must be Arduino interrupt pins
//we need 3 interrupt pins - which requires an Arduino with Atmega32u4 or better (Atmega328 only support 2 interrupts)
//Common RC receiver setup LEFTRIGHT = CH1, FORBACK = CH2, THROTTLE = CH3
//Note: Accelerometer is connected with default Arduino SDA / SCL pins

#define LEFTRIGHT_RC_CHANNEL_PIN 7                //To Left / Right on RC receiver
#define FORBACK_RC_CHANNEL_PIN 1                  //To Forward / Back on RC receiver (Pin 1 on Arduino Micro labelled as "TX" - https://docs.arduino.cc/hacking/hardware/PinMapping32u4)
#define THROTTLE_RC_CHANNEL_PIN 0                 //To Throttle on RC receiver (Pin 0 on Arduino Micro labelled as "RX" - https://docs.arduino.cc/hacking/hardware/PinMapping32u4)

#define HEADING_LED_PIN 13                        //To heading LED (pin 13 is on-board Arduino LED)

//no configuration changes are needed if only 1 motor is used!
#define MOTOR_PIN1 9                              //Pin for Motor 1 driver
#define MOTOR_PIN2 10                             //Pin for Motor 2 driver

#define BATTERY_ADC_PIN A0                        //Pin for battery monitor (if enabled)

//----------PWM MOTOR SETTINGS---------- 
//(only used if a PWM throttle mode is chosen)
//PWM values are 0-255 duty cycle
#define DSHOT_MOTOR_ON 2048                       //Motor PWM ON duty cycle (Simonk: 140 seems barely on / 230 seems a good near-full-throttle value)
#define DSHOT_MOTOR_COAST 0                       //Motor PWM COAST duty cycle - set to same as PWM_ESC_MOTOR_OFF for fully unpowered (best translation?)
#define DSHOT_MOTOR_OFF 0                         //Motor PWM OFF duty cycle (Simonk: 100 worked well in testing - if this is too low - ESC may not init)


//----------BATTERY MONITOR----------
#define BATTERY_ALERT_ENABLED                     //if enabled - heading LED will flicker when battery voltage is low
#define VOLTAGE_DIVIDER 11                        //(~10:1 works well - 10kohm to GND, 100kohm to Bat+).  Resistors have tolerances!  Adjust as needed...
#define BATTERY_ADC_WARN_VOLTAGE_THRESHOLD 7.0f  //If voltage drops below this value - then alert is triggered
#define ARDUINIO_VOLTAGE 5.0f                     //Needed for ADC maths for battery monitor
#define LOW_BAT_REPEAT_READS_BEFORE_ALARM 20      //Requires this many ADC reads below threshold before alarming


//----------SAFETY----------
#define ENABLE_WATCHDOG                           //Uses Adafruit's sleepdog to enable watchdog / reset (tested on AVR - should work for ARM https://github.com/adafruit/Adafruit_SleepyDog)
#define WATCH_DOG_TIMEOUT_MS 2000                 //Timeout value for watchdog (not all values are supported - 2000ms verified with Arudino Micro)
#define CONTROL_MOTION_TIMEOUT_MS 2000            //Timeout value for stick motion / loss of signal
#define VERIFY_RC_THROTTLE_ZERO_AT_BOOT           //Requires RC throttle be 0% at boot to allow spin-up for duration of MAX_MS_BETWEEN_RC_UPDATES (about 1 second)
                                                  //Intended as safety feature to prevent bot from spinning up at power-on if RC was inadvertently left on.
                                                  //Downside is if unexpected reboot occurs during a fight - driver will need to set throttle to zero before power 

#endif