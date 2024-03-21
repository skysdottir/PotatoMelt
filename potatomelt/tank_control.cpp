#include <Arduino.h>

#include "tank_control.h"
#include "motor_driver.h"
#include "rc_handler.h"
#include "led_driver.h"

void handle_tank_mode() {
    // One loop of tank drive!

    int forback = rc_get_trans() * TANK_FORBACK_POWER_SCALE;
    int leftright = rc_get_leftright() * TANK_TURNING_POWER_SCALE;

    motors_on((forback + leftright), -1 * (forback - leftright));

    // and long-blink. Morse code 'T' for tank
    heading_led_on(0); delay(50);
    heading_led_off(); delay(10);
}