/*
 * Pushbutton driver
 * - Initialises pushbutton gpio pins 
 * - Creates push button callback * 
 * 
*/


#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

#include "s4589514_pb.h"

#define SLEEP_TIME_MS 1

#define SW0_NODE    DT_ALIAS(sw0)

bool pbState;

static const struct gpio_dt_spec button = 
            GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});

static struct gpio_callback button_cb_data;

static void button_pressed(const struct device *dev, 
            struct gpio_callback *cb, uint32_t pins)
{
    pbState = !pbState;
	if (pbState) {
        printk("button ON\n");
    } else {
        printk("button OFF\n");
    }
}

void init_pb() {

    int ret;

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, 
                GPIO_INT_EDGE_TO_ACTIVE);
    pbState = 0;

    gpio_init_callback(&button_cb_data, button_pressed, 
                    BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

}

bool get_pb_state(void) {

    return pbState;
}