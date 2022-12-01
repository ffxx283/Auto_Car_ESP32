
#include "SMI130.h"

static const char *TAG = "SMI130";
/**
 * @brief i2c master initialization
 */
esp_err_t SMI130_I2C_init(void)
{
	int ret;
    int i2c_master_port = SMI130_I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SMI130_I2C_MASTER_SDA_IO,
        .scl_io_num = SMI130_I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = SMI130_I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(i2c_master_port, &conf);

    ret = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    if(ret == ESP_OK)
    	ESP_LOGI(TAG, "SMI130_I2C_init OK");
    else
    	ESP_LOGI(TAG, "SMI130_I2C_init Fail");

    return ret;
}


/*
    write a data to register
*/
esp_err_t SMI130_ACC_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(SMI130_I2C_MASTER_NUM, SMI130_ACC_SLAVE_ADDR, write_buf, sizeof(write_buf), SMI130_I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/*
    read a data from register
*/
esp_err_t SMI130_ACC_register_read_byte(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(SMI130_I2C_MASTER_NUM, SMI130_ACC_SLAVE_ADDR, &reg_addr, 1, data, len, SMI130_I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/*
    write a data to register
*/
esp_err_t SMI130_GYR_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(SMI130_I2C_MASTER_NUM, SMI130_GYR_SLAVE_ADDR, write_buf, sizeof(write_buf), SMI130_I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/*
    read a data from register
*/
esp_err_t SMI130_GYR_register_read_byte(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(SMI130_I2C_MASTER_NUM, SMI130_GYR_SLAVE_ADDR, &reg_addr, 1, data, len, SMI130_I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}





