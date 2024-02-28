#include <stdio.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <display.h>
#include <esp_err.h>
#include <ultrasonic.h>
#include <stdbool.h>

#define FLOW_SENSOR_PIN GPIO_NUM_15 
#define LCD_NUM_ROWS 2
#define LCD_NUM_COLS 16
#define LCD_ADDR 0x27 //! check with i2c_tools   
#define LCD_SDA GPIO_NUM_21
#define LCD_SCL GPIO_NUM_22
#define DISTANCIA_MAXIMA_CM 500 //! 5m máximo
#define PINO_TRIGGER GPIO_NUM_5
#define PINO_ECHO GPIO_NUM_18

lcd_t lcd;

void configura_lcd(){
    lcd_t lcd = {
        .rows = LCD_NUM_ROWS,
        .cols = LCD_NUM_COLS,
        .addr = LCD_ADDR,
        .sda = LCD_SDA,
        .scl = LCD_SCL,
        .backlightval = LCD_BACKLIGHT,
        .charsize = LCD_5x8DOTS,
    };

    i2c_init(&lcd);
    lcd_begin(&lcd);
    lcd_backlight(&lcd);
}

void teste_ultrasom (void *pvParameters)
{
    ultrasonic_sensor_t sensor  = {
        .trigger_pin = PINO_TRIGGER,
        .echo_pin = PINO_ECHO,
    };

    ultrasonic_init(&sensor);
    char buffer[32];

    while (true)
    {
        float distancia;
        esp_err_t resultado = ultrasonic_measure(&sensor, DISTANCIA_MAXIMA_CM,  &distancia);
        if (resultado != ESP_OK) 
        {
            printf("Error %d: ", resultado);
            lcd_clear(&lcd);
            lcd_set_cursor(&lcd, 0, 0);
            switch (resultado)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    lcd_print(&lcd, "Ping error");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    lcd_print(&lcd, "Ping timeout");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    lcd_print(&lcd, "Echo timeout");
                    break;
                default:
                    lcd_print(&lcd, "Sensor error");
            }
        }
        else
        {
            snprintf(buffer, sizeof(buffer), "Distance: %.2f cm", distancia);
            lcd_clear(&lcd);
            lcd_set_cursor(&lcd, 0, 0); 
            lcd_print(&lcd, buffer);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    configura_lcd(); 
    xTaskCreate(teste_ultrasom, "Teste Ultrasonic", 2048, NULL, 5, NULL);
}
