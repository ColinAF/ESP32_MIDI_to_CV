/**
 * @file dac.c
 * @brief A driver for the MCP4725 Digital to
 */

#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_CLK_HZ 400000 // 400KHz
#define MCP4725_ADDR 0x60


/**
 * @brief Configures the ESP32's I2C interface
 */
void configure_i2c(void)
{
    i2c_config_t conf = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,               // GPIO 21 is SDA
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22,               // GPIO 22 is SCL
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK_HZ,          
        .clk_flags = 0,                          // you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0); 

    uint8_t out[2] = {0x0F, 0xFF};
    
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, MCP4725_ADDR, out, 2, 100 / portTICK_PERIOD_MS);
    
}