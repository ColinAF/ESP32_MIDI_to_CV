#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLINK_GPIO GPIO_NUM_2
#define I2C_CLK_HZ 400000 // 400KHz
#define MCP4726_ADDR 0x60

static uint8_t s_led_state = 0;

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_i2c(void)
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
    
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();

    // Configure the I2C driver 
    configure_i2c();

    uint8_t out[2] = {0x0F, 0xFF};
    
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, MCP4726_ADDR, out, 2, 100 / portTICK_PERIOD_MS);

    if(ret == ESP_OK)
    {
        while (1) 
        {
            blink_led();
            /* Toggle the LED state */
            s_led_state = !s_led_state;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

}