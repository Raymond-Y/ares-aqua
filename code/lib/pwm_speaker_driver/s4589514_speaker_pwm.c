/*
 * Speaker and PWM driver
 * - Initialises GPIO pin power control
 * - Initialises GPIO pin PWM control
 * - Functions allow buzzer freq to change
 * - Plays mario theme song
 * 
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "s4589514_speaker_pwm.h"
// #include "s4589514_rgb_led.h"

#define PWM_BUZZER_NODE     DT_ALIAS(pwmbuzzer)
#define DT_ALIAS_PWM_1_LABEL    "PWM_1"

#if DT_NODE_HAS_STATUS(PWM_BUZZER_NODE, okay)
#define PWM_CTLR	DT_PWMS_CTLR(PWM_BUZZER_NODE)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_BUZZER_NODE)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_BUZZER_NODE)
#endif

#define GPIO0       DT_NODELABEL(gpio0)
#define GPIO0_29    0x1D    //P0.29 speaker power
#define GPIO0_27    0x1B    //P0.27 speaker PWM

static const struct device *pwm;
static const struct device *devSpeakerPower;

const double melody[] = {
  2637, 2637, 0, 2637,
  0, 2093, 2637, 0,
  3136, 0, 0,  0,
  1568, 0, 0, 0,

  2093, 0, 0, 1568,
  0, 0, 1318.5, 0,
  0, 1760, 0, 1979.5,
  0, 1864.7, 1760, 0,

  1568, 2637, 3136,
  3520, 0, 2793.8, 3136,
  0, 2637, 0, 2093,
  2349.3, 1979.5, 0, 0,

  2093, 0, 0, 1568,
  0, 0, 1318.5, 0,
  0, 1760, 0, 1979.5,
  0, 1864.7, 1760, 0,

  1568, 2637, 3136,
  3520, 0, 2793.8, 3136,
  0, 2637, 0, 2093,
  2349.3, 1979.5, 0, 0
};

const int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};


void init_speaker_pwm(void) {
    pwm = device_get_binding(DT_ALIAS_PWM_1_LABEL);

    devSpeakerPower = device_get_binding(DT_LABEL(GPIO0));
    gpio_pin_configure(devSpeakerPower, GPIO0_29, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set(devSpeakerPower, GPIO0_29, 1); // turn speaker on

    // if (!pwm)
	// {   
    //     change_rgb_state(1, 0, 0);
	// 	printk("Cannot find PWM device!\n");
	// 	return;
	// }
	// else
	// {
    //     change_rgb_state(0, 1, 0);
	// 	printk("PWM device find\n");
	// 	printk("%s\n", DT_ALIAS_PWM_1_LABEL);
	// }
}

void buzzer_frequency(int freq) {
    uint32_t periodMicro = (uint32_t) ((1.0 / freq) * 1000000);
    uint32_t pulse = (uint32_t) (periodMicro / 2.0);

    pwm_pin_set_usec(pwm, GPIO0_27, periodMicro, pulse, 0);
}

void play_mario_tune(void) {
    uint32_t periodMicro;
    uint32_t pulse;
    // 94 is the total num of notes
    for (int i = 0; i < 78; i++) {
        periodMicro = (uint32_t) ((1.0 / melody[i]) * 1000000);
        pulse = (uint32_t) (periodMicro / 2.0);
        pwm_pin_set_usec(pwm, GPIO0_27, periodMicro, pulse, 0);
        k_msleep(1000 / tempo[i]); // duration to hold note
        pwm_pin_set_usec(pwm, GPIO0_27, 0, 0, 0); // turn off pwm
        // wait 1.3x the duration of the note before starting next note
        k_msleep((1000 / tempo[i]));
    }
}