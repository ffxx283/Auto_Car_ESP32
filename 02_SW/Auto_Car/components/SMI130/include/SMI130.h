
#ifndef _SMI130_H_
#define _SMI130_H_

#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c.h"
#include "driver/gpio.h"


#define SMI130_I2C_MASTER_NUM              1           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define SMI130_I2C_MASTER_SCL_IO           23          /*!< GPIO number used for I2C master clock */
#define SMI130_I2C_MASTER_SDA_IO           18          /*!< GPIO number used for I2C master data  */
#define SMI130_I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */
#define SMI130_I2C_MASTER_TIMEOUT_MS       1000

#define SMI130_ACC_SLAVE_ADDR              0x18        /*!< Slave address of the TCA9539 */
#define SMI130_GYR_SLAVE_ADDR              0x68        /*!< Slave address of the TCA9539 */


esp_err_t SMI130_I2C_init(void);

esp_err_t SMI130_ACC_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t SMI130_ACC_register_read_byte(uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t SMI130_GYR_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t SMI130_GYR_register_read_byte(uint8_t reg_addr, uint8_t *data, size_t len);



#endif // !_SMI130_H_
