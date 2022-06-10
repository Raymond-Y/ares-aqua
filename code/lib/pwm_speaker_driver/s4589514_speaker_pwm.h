/*
 * Speaker and PWM driver
 * - Initialises GPIO pin power control
 * - Initialises GPIO pin PWM control
 * - Functions allow buzzer freq to change
 * - Plays mario theme song
 * 
*/

#ifndef S4589514_SPEAKER_PWM_H
#define S4589514_SPEAKER_PWM_H

void init_speaker_pwm(void);
void buzzer_frequency(int freq);
void play_mario_tune(void);

#endif