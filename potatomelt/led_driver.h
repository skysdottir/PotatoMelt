enum LED_Pattern {
    READY = 0xAAAA,     // # # # # # # # # 
    LOS = 0xFFFF,       // ################
    CONFIG = 0xA0A0,    // # #     # #     
    TANK = 0x7777,      //  ### ### ### ###
    BATTERY = 0x7D50    //  ##### # # #    
};

void init_led(void);

//turns heading LED on (with quick flicker effect if "shimmering")
void heading_led_on(int led_shimmering);
 
//turns heading LED off
void heading_led_off();