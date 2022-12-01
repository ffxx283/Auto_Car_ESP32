
#include "TCA9539.h"

static const char *TAG = "TCA9539";

tca_config_t tca_config = {
        .LED_MODE 				= TCA_MODE_OUTPUT,
		.KEY_MODE 				= TCA_MODE_INPUT,

		.MOTOR_PWR_MODE 		= TCA_MODE_OUTPUT,

		.OLED_RESET_MODE 		= TCA_MODE_OUTPUT,
		.IP5306_KEY_MODE 		= TCA_MODE_OUTPUT,
		.SYS_PWR_MODE 			= TCA_MODE_OUTPUT,
		.RADAR_PWR_MODE 		= TCA_MODE_OUTPUT,
		.SENSOR_PWR_MODE 		= TCA_MODE_OUTPUT,
		.OV2640_RESET_MODE 		= TCA_MODE_OUTPUT
    };

tca_output_t tca_output = {
		.LED_OUT_STA 			= TCA_IO_HIGH,

		.MOTOR_PWR_OUT_STA 		= TCA_IO_HIGH,
		.OLED_RESET_OUT_STA 	= TCA_IO_HIGH,
		.IP5306_KEY_OUT_STA 	= TCA_IO_HIGH,
		.SYS_PWR_OUT_STA 		= TCA_IO_LOW,
		.RADAR_PWR_OUT_STA 		= TCA_IO_HIGH,
		.SENSOR_PWR_OUT_STA 	= TCA_IO_HIGH,
		.OV2640_RESET_OUT_STA 	= TCA_IO_HIGH
};


/**
 * @brief i2c master initialization
 */
esp_err_t TCA9539_init(void)
{
    int i2c_master_port = TCA_I2C_MASTER_NUM;

    TCA9539_Reset();

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TCA_I2C_MASTER_SDA_IO,
        .scl_io_num = TCA_I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = TCA_I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


/**
 * @brief i2c master initialization
 */
esp_err_t TCA9539_Reset(void)
{
    int ret;

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins GPIO17
    io_conf.pin_bit_mask = TCA_RESET_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    ret = gpio_set_level(TCA_RESET_PIN, 0);
    vTaskDelay(300 / portTICK_RATE_MS);
    ret = gpio_set_level(TCA_RESET_PIN, 1);

    return ret;
}

/*
    write a data to register
*/
esp_err_t TCA_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(TCA_I2C_MASTER_NUM, TCA9539_SLAVE_ADDR, write_buf, sizeof(write_buf), TCA_I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/*
    read a data from register
*/
esp_err_t TCA_register_read_byte(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(TCA_I2C_MASTER_NUM, TCA9539_SLAVE_ADDR, &reg_addr, 1, data, len, TCA_I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/*
    set port0 and port1 as input and output ports
    can be a bit operation
*/
esp_err_t TCA_Set_Pin_Mode(void)
{
    int ret;
    uint8_t cfg_port0 = 0;
    uint8_t cfg_port1 = 0;
    uint8_t default_sta_port0 = 0;
    uint8_t default_sta_port1 = 0;

    /* configure the default output state of port0 */
    default_sta_port0 = (\
    		(tca_output.MOTOR_PWR_OUT_STA<<6) | \
			(tca_output.OLED_RESET_OUT_STA<<5) | \
			(tca_output.IP5306_KEY_OUT_STA<<4) | \
			(tca_output.SYS_PWR_OUT_STA<<3) | \
			(tca_output.RADAR_PWR_OUT_STA<<2) | \
			(tca_output.SENSOR_PWR_OUT_STA<<1) | \
			(tca_output.OV2640_RESET_OUT_STA)   \
			);

    /* configure the default output state of port1 */
    if( tca_output.LED_OUT_STA == TCA_IO_LOW )
    	default_sta_port1 &= 0x0F;
	else if( tca_output.LED_OUT_STA == TCA_IO_HIGH )
		default_sta_port1 |= 0xF0;

    /* configure the in/out mode register port0 */
    cfg_port0 = (\
    		(tca_config.MOTOR_PWR_MODE<<6) | \
			(tca_config.OLED_RESET_MODE<<5) | \
			(tca_config.IP5306_KEY_MODE<<4) | \
			(tca_config.SYS_PWR_MODE<<3) | \
			(tca_config.RADAR_PWR_MODE<<2) | \
			(tca_config.SENSOR_PWR_MODE<<1) | \
			(tca_config.OV2640_RESET_MODE)   \
			);

    /* configure the in/out mode register port1 */
    if( tca_config.LED_MODE == TCA_MODE_OUTPUT )
    	cfg_port1 &= 0x0F;
    else if( tca_config.LED_MODE == TCA_MODE_INPUT )
    	cfg_port1 |= 0xF0;

    if( tca_config.KEY_MODE == TCA_MODE_OUTPUT )
        	cfg_port1 &= 0xF0;
	else if( tca_config.KEY_MODE == TCA_MODE_INPUT )
		cfg_port1 |= 0x0F;

    /* write data to register */
    ret = TCA_register_write_byte(TCA9539_OUTPUT_PORT0,default_sta_port0);
    if (ret != ESP_OK) return ret;
    ret = TCA_register_write_byte(TCA9539_OUTPUT_PORT1,default_sta_port1);
    if (ret != ESP_OK) return ret;

    /* write data to register */
    ret = TCA_register_write_byte(TCA9539_CFGMODE_PORT0,cfg_port0);
    if (ret != ESP_OK) return ret;

    return TCA_register_write_byte(TCA9539_CFGMODE_PORT1,cfg_port1);
}

/*
    get port0 and port1 's input and output cfg
*/
esp_err_t TCA_Get_Pin_Mode(uint8_t *data)
{
    int ret;

    ret = TCA_register_read_byte( TCA9539_CFGMODE_PORT0, data, 1);
    if (ret != ESP_OK) return ret;

    data++;
    return TCA_register_read_byte( TCA9539_CFGMODE_PORT1, data, 1);

}

/*
    Set Output_Pin
    PORT:       0 or 1
    Pin_Num :   0~7
    sta:        0 or 1
*/
esp_err_t TCA_Set_Output_Pin(uint8_t PORT, uint8_t Pin_Num, uint8_t sta)
{
    int ret;
    uint8_t read_buf[2];
    uint8_t data;

    ret = TCA_Get_Pin_Mode(read_buf);
    if (ret != ESP_OK) return ret;
    if( PORT == 0 ){
        if ( ~read_buf[0] & (1<<Pin_Num) )
        {
            ret = TCA_register_read_byte( TCA9539_OUTPUT_PORT0, &data, 1);
            if (ret != ESP_OK) return ret;
            if (sta)
                data = data | (1<<Pin_Num);
            else
                data = data & (~(1<<Pin_Num));
            ret = TCA_register_write_byte(TCA9539_OUTPUT_PORT0,data);
            if (ret != ESP_OK) return ret;
        }
        else{
        	ESP_LOGI(TAG, "PORT0 pin do not configure");
        	return ESP_FAIL;
        }

    }
    else
    {
        if ( ~read_buf[1] & (1<<Pin_Num) )
        {
            ret = TCA_register_read_byte( TCA9539_OUTPUT_PORT1, &data, 1);
            if (ret != ESP_OK) return ret;
            if (sta)
                data = data | (1<<Pin_Num);
            else
                data = data & (~(1<<Pin_Num));
            ret = TCA_register_write_byte(TCA9539_OUTPUT_PORT1,data);
            if (ret != ESP_OK) return ret;
        }
        else{
        	ESP_LOGI(TAG, "PORT1 pin do not configure");
        	return ESP_FAIL;
        }
    }
    return ret;
}

/*
    Set Output_Port
    PORT:       0 or 1
    sta:        0x00~0xff
*/
esp_err_t TCA_Set_Output_PORT(uint8_t PORT, uint8_t sta)
{
    int ret;

    if (!PORT)
    {
        ret = TCA_register_write_byte(TCA9539_OUTPUT_PORT0,sta);
        if (ret != ESP_OK) return ret;
    }
    else
    {
        ret = TCA_register_write_byte(TCA9539_OUTPUT_PORT1,sta);
        if (ret != ESP_OK) return ret;
    }

    return ret;
}

/*
    Set Output_Pin
    PORT:       0 or 1
    Pin_Num :   0~7
    sta:        0 or 1
*/
esp_err_t TCA_Get_input_Pin(uint8_t PORT, uint8_t Pin_Num, uint8_t *sta)
{
    int ret;

    if (!PORT)
    {
        ret = TCA_register_read_byte(TCA9539_INPUT_PORT0,sta,1);
        if (ret != ESP_OK) return ret;
    }
    else
    {
        ret = TCA_register_read_byte(TCA9539_INPUT_PORT1,sta,1);
        if (ret != ESP_OK) return ret;
    }

    if ( *sta & (1<<Pin_Num))
    {
        *sta = 1;
    }
    else
    {
        *sta = 0;
    }


    return ret;
}

/*
    Set Output_Port
    PORT:       0 or 1
    sta:        0x00~0xff
*/
esp_err_t TCA_Get_input_PORT(uint8_t PORT , uint8_t *sta)
{
    int ret;

    if (!PORT)
    {
        ret = TCA_register_read_byte(TCA9539_INPUT_PORT0,sta,1);
        if (ret != ESP_OK) return ret;
    }
    else
    {
        ret = TCA_register_read_byte(TCA9539_INPUT_PORT1,sta,1);
        if (ret != ESP_OK) return ret;
    }

    return ret;
}

/*
    Set Sensor PWR
    sta:        0 or 1
*/
esp_err_t TCA_Set_Sensor_PWR(uint8_t sta)
{
	int ret=0;

	ESP_ERROR_CHECK(TCA_Set_Output_Pin( SENSOR_PWR_PORT , SENSOR_PWR_PIN , sta));

	return ret;
}


/*
    Set Motor PWR
    sta:        0 or 1
*/
esp_err_t TCA_Set_Motor_PWR(uint8_t sta)
{
	int ret=0;

	ESP_ERROR_CHECK(TCA_Set_Output_Pin( MOTOR_PWR_PORT , MOTOR_PWR_PIN , sta));

	return ret;
}


/*
    Set IP5306_KEY IO
    sta:        0 or 1
*/
esp_err_t TCA_Set_IP5306_KEY(uint8_t sta)
{
	int ret=0;

	ESP_ERROR_CHECK(TCA_Set_Output_Pin( IP5306_KEY_PORT , IP5306_KEY_PIN , sta));

	return ret;
}


/*
    Set  SYS_PWR_SHUTDOWN IO
    sta:        0 or 1
*/
esp_err_t TCA_Set_SYS_PWR_SHUTDOWN(uint8_t sta)
{
	int ret=0;

	ESP_ERROR_CHECK(TCA_Set_Output_Pin( SYS_PWR_SHUTDOWN_PORT , SYS_PWR_SHUTDOWN_PIN , sta));

	return ret;
}


/*
    Set  OLED_RESET IO
    sta:        0 or 1
*/
esp_err_t TCA_Set_OLED_RESET(uint8_t sta)
{
	int ret=0;

	ESP_ERROR_CHECK(TCA_Set_Output_Pin( SYS_PWR_SHUTDOWN_PORT , SYS_PWR_SHUTDOWN_PIN , sta));

	return ret;
}



/*
    Set  OV2640_RESET IO
    sta:        0 or 1
*/
esp_err_t TCA_Set_OV2640_RESET(uint8_t sta)
{
	int ret=0;

	ESP_ERROR_CHECK(TCA_Set_Output_Pin( OV_RESET_PORT , OV_RESET_PIN , sta));

	return ret;
}


/*
    Get Key Input
    ret:        0x00~0x0f
*/
uint8_t TCA_Get_KEY_IN( void)
{
	int ret = TCA_KEY_NO_PRESS;
    uint8_t key=0;

    ESP_ERROR_CHECK( TCA_Get_input_PORT(1, &key ) );
    if( (key&0x0F) != TCA_KEY_NO_PRESS)
    {
    	vTaskDelay(50 / portTICK_RATE_MS);
    	ESP_ERROR_CHECK( TCA_Get_input_PORT(1, &key ) );
    	if( (key&0x0F) != TCA_KEY_NO_PRESS)
    		ret = key&0x0F;
    }

    return ret;
}

























