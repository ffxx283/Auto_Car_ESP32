#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdio.h>
#include "esp_err.h"
#include "driver/gpio.h"

#define MOTOR_A_PWM_PIN 0
#define MOTOR_A_IN1_PIN 15
#define MOTOR_A_IN2_PIN 2
#define MOTOR_A_PIN_SEL  ((1ULL<<MOTOR_A_PWM_PIN) | (1ULL<<MOTOR_A_IN1_PIN) | (1ULL<<MOTOR_A_IN2_PIN) )

#define MOTOR_B_PWM_PIN 12
#define MOTOR_B_IN1_PIN 26
#define MOTOR_B_IN2_PIN 16
#define MOTOR_B_PIN_SEL  ((1ULL<<MOTOR_B_PWM_PIN) | (1ULL<<MOTOR_B_IN1_PIN) | (1ULL<<MOTOR_B_IN2_PIN) )

#define MOTOR_FLAG_RIGHT 	1
#define MOTOR_FLAG_MIDDLE 	0
#define MOTOR_FLAG_LEFT 	-1

#define MOTOR_RUN_FRONT 	1
#define MOTOR_RUN_STOP 		0
#define MOTOR_RUN_BACK 		-1

esp_err_t Motor_IO_Init(void);

esp_err_t Motor_TURN_CONROL(int flag);
esp_err_t Motor_RUN(int flag);





#endif // !_MOTOR_H_
