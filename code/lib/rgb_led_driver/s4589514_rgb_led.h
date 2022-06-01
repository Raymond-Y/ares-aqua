/*
 * RGB LED Driver 
 * - Initialises RGB GPIO expander pin
 * - Configures GPIO pins
 * - Allows individual control and toggling of 
 * 		RGB LEDS
 * 
*/

#ifndef S4589514_RGB_LED_H
#define S4589514_RGB_LED_H

#include <stdbool.h>

void init_rgb_led(void);
void deinit_rgb_led(void);
void change_rgb_state(bool redState, bool greenState, bool blueState);
void toggle_rgb_pin(int16_t pin);
void rgb_led_off(void);


#endif