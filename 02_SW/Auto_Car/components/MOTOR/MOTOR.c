
#include "MOTOR.h"


/**
 * @brief Motor_IO_Init
 */
esp_err_t Motor_IO_Init(void)
{
    int ret;

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins GPIO17
    io_conf.pin_bit_mask = MOTOR_A_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    io_conf.pin_bit_mask = MOTOR_B_PIN_SEL;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;


    return ret;
}

/**
 * @brief Motor_A_Go
 */
esp_err_t Motor_TURN_CONROL(int flag)
{
	int ret;

	if(flag == MOTOR_FLAG_RIGHT)
	{
		ret = gpio_set_level(MOTOR_A_PWM_PIN, 1);
		ret = gpio_set_level(MOTOR_A_IN1_PIN, 1);
		ret = gpio_set_level(MOTOR_A_IN2_PIN, 0);
	}
	else if(flag == MOTOR_FLAG_LEFT)
	{
		ret = gpio_set_level(MOTOR_A_PWM_PIN, 1);
		ret = gpio_set_level(MOTOR_A_IN1_PIN, 0);
		ret = gpio_set_level(MOTOR_A_IN2_PIN, 1);
	}
	else if(flag == MOTOR_FLAG_MIDDLE)
	{
		ret = gpio_set_level(MOTOR_A_PWM_PIN, 0);
		ret = gpio_set_level(MOTOR_A_IN1_PIN, 0);
		ret = gpio_set_level(MOTOR_A_IN2_PIN, 0);
	}
	else
		ret = 0;

	return ret;
}

/**
 * @brief Motor_B_Go
 */
esp_err_t Motor_RUN(int flag)
{
	int ret;

	if(flag == MOTOR_RUN_FRONT)
	{
		ret = gpio_set_level(MOTOR_B_PWM_PIN, 1);
		ret = gpio_set_level(MOTOR_B_IN1_PIN, 0);
		ret = gpio_set_level(MOTOR_B_IN2_PIN, 1);
	}
	else if(flag == MOTOR_RUN_BACK)
	{
		ret = gpio_set_level(MOTOR_B_PWM_PIN, 1);
		ret = gpio_set_level(MOTOR_B_IN1_PIN, 1);
		ret = gpio_set_level(MOTOR_B_IN2_PIN, 0);
	}
	else if(flag == MOTOR_RUN_STOP)
	{
		ret = gpio_set_level(MOTOR_B_PWM_PIN, 0);
		ret = gpio_set_level(MOTOR_B_IN1_PIN, 0);
		ret = gpio_set_level(MOTOR_B_IN2_PIN, 0);
	}
	else
		ret = 0;

	return ret;
}



