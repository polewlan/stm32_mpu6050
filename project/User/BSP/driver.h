#ifndef __DRIVER_H_
#define __DRIVER_H_
#include "main.h"
#include "tim.h"
#include "gpio.h"

void motor_control(int16_t pwm);
int16_t get_encoder(void);
int16_t motor_pi(int16_t encoder, int16_t target);
float velocity_control(int16_t encoder, int16_t target);
int16_t angle_control(float angle, float target_angle);
int16_t palstance_control(int16_t palstance, int16_t target_palstance);

#endif
