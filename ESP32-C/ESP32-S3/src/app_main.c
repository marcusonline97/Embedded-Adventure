#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

/* ---------- Pin plan ---------- */
#define PIN_TRIG   42
#define PIN_ECHO   14          /* through 1k + 2k (two 1k in series) divider! */
#define PIN_SDA    21
#define PIN_SCL    47
#define PIN_JOY_SW 2           /* button, active LOW */
#define JOY_X_CH   ADC_CHANNEL_0   /* GPIO 1 = ADC1 channel 0 on ESP32-S3 */

/* ---------- Settings ---------- */
#define LOOP_MS        100     /* time between measurements (sonar needs >60 ms) */
#define JOY_CENTER     2048
#define JOY_DEADZONE   500
#define ALERT_MIN_CM   5
#define ALERT_MAX_CM   300
#define ALERT_STEP_CM  5
#define ALERT_START_CM 50

#define LCD_ADDR  0x27         /* try 0x3F if the screen stays blank */
#define I2C_PORT  I2C_NUM_0

/* ---------- LCD (HD44780 behind a PCF8574 I2C backpack) ---------- */
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BL 0x08

static void pcf_write(uint8_t d)
{
    i2c_master_write_to_device(I2C_PORT, LCD_ADDR, &d, 1, pdMS_TO_TICKS(50));
}

static void lcd_nibble(uint8_t nib, uint8_t rs)
{
    uint8_t d = (uint8_t)((nib << 4) | rs | LCD_BL);
    pcf_write(d | LCD_EN);
    esp_rom_delay_us(2);
    pcf_write(d);
    esp_rom_delay_us(60);
}

static void lcd_send(uint8_t v, uint8_t rs)
{
    lcd_nibble(v >> 4, rs);
    lcd_nibble(v & 0x0F, rs);
}

static void lcd_cmd(uint8_t c)
{
    lcd_send(c, 0);
    if (c <= 0x02) esp_rom_delay_us(2000);   /* clear/home are slow */
}

static void lcd_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_nibble(0x03, 0); esp_rom_delay_us(5000);
    lcd_nibble(0x03, 0); esp_rom_delay_us(200);
    lcd_nibble(0x03, 0); esp_rom_delay_us(200);
    lcd_nibble(0x02, 0);             /* switch to 4-bit mode */
    lcd_cmd(0x28);                   /* 2 lines, 5x8 font */
    lcd_cmd(0x0C);                   /* display on, no cursor */
    lcd_cmd(0x06);                   /* cursor moves right */
    lcd_cmd(0x01);                   /* clear */
}

/* Prints at most 16 characters on the given row */
static void lcd_print_at(uint8_t row, const char *s)
{
    lcd_cmd(0x80 | (row ? 0x40 : 0x00));
    for (int i = 0; i < 16 && s[i]; i++) lcd_send((uint8_t)s[i], LCD_RS);
}

/* ---------- Sonar (HC-SR04) ---------- */
/* Returns distance in cm, or -1 if nothing was detected */
static float read_distance_cm(void)
{
    gpio_set_level(PIN_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(PIN_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_TRIG, 0);

    int64_t t0 = esp_timer_get_time();
    while (!gpio_get_level(PIN_ECHO)) {
        if (esp_timer_get_time() - t0 > 30000) return -1;
    }
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(PIN_ECHO)) {
        if (esp_timer_get_time() - start > 25000) return -1;   /* ~4 m max */
    }
    int64_t us = esp_timer_get_time() - start;
    return (float)us * 0.0343f / 2.0f;
}

/* ---------- Joystick button (interrupt + debounce) ---------- */
static volatile bool    btn_flag    = false;
static volatile int64_t last_btn_us = 0;

static void IRAM_ATTR btn_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - last_btn_us > 300000) {
        last_btn_us = now;
        btn_flag = true;
    }
}

/* ---------- Display + serial output ---------- */
static void show(float cm, int alert_cm, bool inches)
{
    char l0[32], l1[32];
    const char *unit = inches ? "in" : "cm";
    float k = inches ? (1.0f / 2.54f) : 1.0f;
    bool close = (cm >= 0 && cm < alert_cm);

    if (cm < 0) snprintf(l0, sizeof(l0), "Dist: ---       ");
    else        snprintf(l0, sizeof(l0), "Dist:%5.1f %s   ", cm * k, unit);
    snprintf(l1, sizeof(l1), "Alert:%4d %s %s", (int)(alert_cm * k), unit, close ? "!!" : "  ");

    lcd_print_at(0, l0);
    lcd_print_at(1, l1);
    printf("%.1f\n", cm);   /* one value per line, handy for a plotter later */
}

void app_main(void)
{
    /* Sonar trigger output */
    gpio_config_t trig_cfg = {
        .pin_bit_mask = 1ULL << PIN_TRIG,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trig_cfg);

    /* Sonar echo input */
    gpio_config_t echo_cfg = {
        .pin_bit_mask = 1ULL << PIN_ECHO,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&echo_cfg);

    /* Joystick button with interrupt */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << PIN_JOY_SW,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_JOY_SW, btn_isr, NULL);

    /* Joystick X axis (ADC) */
    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &adc);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc, JOY_X_CH, &chan_cfg);

    lcd_init();
    lcd_print_at(0, "Sonar meter");
    vTaskDelay(pdMS_TO_TICKS(800));
    lcd_cmd(0x01);

    bool inches = false;
    int  alert_cm = ALERT_START_CM;

    while (1) {
        /* Button: switch between cm and inches */
        if (btn_flag) {
            btn_flag = false;
            inches = !inches;
        }

        /* Joystick left/right: lower/raise the alert distance */
        int x = 0;
        adc_oneshot_read(adc, JOY_X_CH, &x);
        if (x > JOY_CENTER + JOY_DEADZONE) alert_cm += ALERT_STEP_CM;
        if (x < JOY_CENTER - JOY_DEADZONE) alert_cm -= ALERT_STEP_CM;
        if (alert_cm < ALERT_MIN_CM) alert_cm = ALERT_MIN_CM;
        if (alert_cm > ALERT_MAX_CM) alert_cm = ALERT_MAX_CM;

        show(read_distance_cm(), alert_cm, inches);
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}