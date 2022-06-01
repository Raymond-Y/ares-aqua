/*
 * RGB LED Driver 
 * - Initialises RGB GPIO expander pin
 * - Configures GPIO pins
 * - Allows individual control and toggling of 
 * 		RGB LEDS
 * 
*/

#include <zephyr.h>
#include <drivers/gpio.h>

#include "s4589514_rgb_led.h"

#define SW1_NODE	DT_ALIAS(sw1)
#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
    #define PARTICLE_ARGON      true
    #define THINGY_52           false
#else
    #define PARTICLE_ARGON      false
    #define THINGY_52           true
#endif

#if THINGY_52 == true

#define LED0_NODE DT_ALIAS(redled)
#define LED1_NODE DT_ALIAS(greenled)
#define LED2_NODE DT_ALIAS(blueled)

static const struct gpio_dt_spec ledRed = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec ledGreen = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec ledBlue = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

static bool ledStates[3];

void init_rgb_led(void) {
	if (!device_is_ready(ledRed.port) || !device_is_ready(ledBlue.port) || 
				!device_is_ready(ledGreen.port)) {
		return;
	}
	gpio_pin_configure_dt(&ledRed, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&ledGreen, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&ledBlue, GPIO_OUTPUT_ACTIVE);

	rgb_led_off();
} 

void deinit_rgb_led(void) {
	gpio_pin_configure_dt(&ledRed, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledGreen, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledBlue, GPIO_OUTPUT_INACTIVE);
}

void change_rgb_state(bool redState, bool greenState, bool blueState) {
	gpio_pin_set_dt(&ledRed, redState);
	gpio_pin_set_dt(&ledGreen, greenState);
	gpio_pin_set_dt(&ledBlue, blueState);
	ledStates[0] = redState;
	ledStates[1] = greenState;
	ledStates[2] = blueState;
}

// r = 0, g = 1, b = 2
void toggle_rgb_pin(int16_t pin) {
	if (pin == 1) {
		gpio_pin_toggle_dt(&ledRed);
	} else if (pin == 2) {
		gpio_pin_toggle_dt(&ledGreen);
	} else if (pin == 3) {
		gpio_pin_toggle_dt(&ledBlue);
	}
}

void rgb_led_off(void) {
	gpio_pin_set_dt(&ledRed, 0);
	gpio_pin_set_dt(&ledGreen, 0);
	gpio_pin_set_dt(&ledBlue, 0);
	ledStates[0] = 0;
	ledStates[1] = 0;
	ledStates[2] = 0;

}

#endif
