
#ifndef _TCA9539_H_
#define _TCA9539_H_

#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c.h"
#include "driver/gpio.h"


#define TCA_RESET_PIN    				17
#define TCA_RESET_PIN_SEL  				((1ULL<<TCA_RESET_PIN))
#define TCA_RESET_TIME    				100                       //ms

#define TCA_I2C_MASTER_NUM              0           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define TCA_I2C_MASTER_SCL_IO           19          /*!< GPIO number used for I2C master clock */
#define TCA_I2C_MASTER_SDA_IO           22          /*!< GPIO number used for I2C master data  */
#define TCA_I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */
#define TCA_I2C_MASTER_TIMEOUT_MS       1000

#define TCA9539_SLAVE_ADDR              0x77        /*!< Slave address of the TCA9539 */

#define TCA9539_INPUT_PORT0             0x00        /*!< Register addresses"*/
#define TCA9539_INPUT_PORT1             0x01        /*!< Register addresses"*/
#define TCA9539_OUTPUT_PORT0            0x02        /*!< Register addresses"*/
#define TCA9539_OUTPUT_PORT1            0x03        /*!< Register addresses"*/
#define TCA9539_POLAR_PORT0             0x04        /*!< Register addresses"*/
#define TCA9539_POLAR_PORT1             0x05        /*!< Register addresses"*/
#define TCA9539_CFGMODE_PORT0           0x06        /*!< Register addresses"*/
#define TCA9539_CFGMODE_PORT1           0x07        /*!< Register addresses"*/

#define TCA_MODE_INPUT					1
#define TCA_MODE_OUTPUT					0
#define TCA_IO_HIGH						1
#define TCA_IO_LOW						0

#define OV_RESET_PORT          			0        	/*!< Register addresses"*/
#define OV_RESET_PIN           			0        	/*!< Register addresses"*/

#define SENSOR_PWR_PORT          		0        	/*!< Register addresses"*/
#define SENSOR_PWR_PIN           		1        	/*!< Register addresses"*/

#define SYS_PWR_SHUTDOWN_PORT          	0        	/*!< Register addresses"*/
#define SYS_PWR_SHUTDOWN_PIN           	3        	/*!< Register addresses"*/

#define IP5306_KEY_PORT          		0        	/*!< Register addresses"*/
#define IP5306_KEY_PIN           		4        	/*!< Register addresses"*/

#define OLED_RESET_PORT          		0        	/*!< Register addresses"*/
#define OLED_RESET_PIN           		5        	/*!< Register addresses"*/

#define MOTOR_PWR_PORT          		0        	/*!< Register addresses"*/
#define MOTOR_PWR_PIN           		6        	/*!< Register addresses"*/

#define TCA_KEY_NO_PRESS				0x0F
#define TCA_KEY_PRESS_UP				0x0D
#define TCA_KEY_PRESS_DOWN				0x0B
#define TCA_KEY_PRESS_LEFT				0x07
#define TCA_KEY_PRESS_RIGHT				0x0E

/**
 * @brief Configuration structure for GPIO initialization
 */
typedef struct {
    uint8_t LED_MODE;
    uint8_t KEY_MODE;
    uint8_t MOTOR_PWR_MODE;
    uint8_t OLED_RESET_MODE;
    uint8_t IP5306_KEY_MODE;
    uint8_t SYS_PWR_MODE;
    uint8_t RADAR_PWR_MODE;
    uint8_t SENSOR_PWR_MODE;
    uint8_t OV2640_RESET_MODE;
} tca_config_t;

typedef struct {
    uint8_t LED_OUT_STA;
    uint8_t MOTOR_PWR_OUT_STA;
    uint8_t OLED_RESET_OUT_STA;
    uint8_t IP5306_KEY_OUT_STA;
    uint8_t SYS_PWR_OUT_STA;
    uint8_t RADAR_PWR_OUT_STA;
    uint8_t SENSOR_PWR_OUT_STA;
    uint8_t OV2640_RESET_OUT_STA;
} tca_output_t;



esp_err_t TCA9539_init(void);
esp_err_t TCA9539_Reset(void);

esp_err_t TCA_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t TCA_register_read_byte(uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t TCA_Set_Pin_Mode(void);
esp_err_t TCA_Get_Pin_Mode(uint8_t *data);

esp_err_t TCA_Set_Output_Pin(uint8_t PORT, uint8_t Pin_Num, uint8_t sta);
esp_err_t TCA_Set_Output_PORT(uint8_t PORT, uint8_t sta);

esp_err_t TCA_Get_input_Pin(uint8_t PORT, uint8_t Pin_Num, uint8_t *sta);
esp_err_t TCA_Get_input_PORT(uint8_t PORT , uint8_t *sta);

esp_err_t TCA_Set_Sensor_PWR(uint8_t sta);
esp_err_t TCA_Set_Motor_PWR(uint8_t sta);
esp_err_t TCA_Set_IP5306_KEY(uint8_t sta);
esp_err_t TCA_Set_SYS_PWR_SHUTDOWN(uint8_t sta);
esp_err_t TCA_Set_OLED_RESET(uint8_t sta);
esp_err_t TCA_Set_OV2640_RESET(uint8_t sta);
uint8_t	  TCA_Get_KEY_IN( void );


#endif // !_TCA9539_H_
