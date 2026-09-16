// SPDX-License-Identifier: GPL-3.0-only

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "app_config.h"
#include <stdint.h>

#define TAG "actuator"

static const int pwm_gpio[4] = {
    PIN_SERVO_TURBINE_G, PIN_SERVO_TURBINE_D,
    PIN_SERVO_REVERSE_G, PIN_SERVO_REVERSE_D
};
static const uint8_t ads_channel[4] = {
    ADS_CH_TURBINE_G, ADS_CH_TURBINE_D,
    ADS_CH_REVERSE_G, ADS_CH_REVERSE_D
};
static const int16_t angle_min[4] = {
    TURBINE_G_ANGLE_MIN_DEG, TURBINE_D_ANGLE_MIN_DEG,
    REVERSE_G_ANGLE_MIN_DEG, REVERSE_D_ANGLE_MIN_DEG
};
static const int16_t angle_max[4] = {
    TURBINE_G_ANGLE_MAX_DEG, TURBINE_D_ANGLE_MAX_DEG,
    REVERSE_G_ANGLE_MAX_DEG, REVERSE_D_ANGLE_MAX_DEG
};
static uint16_t pwm_value[4];

static void pwm_write(int ch, uint16_t value) {
    if (value > SERVO_COMMAND_MAX) value = SERVO_COMMAND_MAX;
    pwm_value[ch] = value;
    /* Servo RC : période 20 ms (50 Hz), impulsion 1..2 ms. */
    uint32_t pulse_us = SERVO_PULSE_MIN_US + (uint32_t)value;
    uint32_t duty = pulse_us * 65535U / 20000U;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

static esp_err_t ads_read(uint8_t channel, int16_t *raw) {
    uint16_t cfg = 0x4000 | ((uint16_t)(channel + 4) << 12) | 0x0200 | 0x0100 | 0x0083;
    uint8_t w[3] = {0x01, (uint8_t)(cfg >> 8), (uint8_t)cfg};
    esp_err_t e = i2c_master_write_to_device(I2C_NUM_0, ADS1115_ADDRESS, w, sizeof(w), pdMS_TO_TICKS(20));
    if (e != ESP_OK) return e;
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t reg = 0x00, r[2];
    e = i2c_master_write_read_device(I2C_NUM_0, ADS1115_ADDRESS, &reg, 1, r, 2, pdMS_TO_TICKS(20));
    if (e == ESP_OK) *raw = (int16_t)(((uint16_t)r[0] << 8) | r[1]);
    return e;
}

static int16_t raw_to_angle(int16_t raw, int i) {
    if (raw < 0) raw = 0;
    return angle_min[i] + (int32_t)(raw - ADS_RAW_MIN) *
           (angle_max[i] - angle_min[i]) / (ADS_RAW_MAX - ADS_RAW_MIN);
}

static void send_status(void) {
    twai_message_t m = {.identifier = CAN_ID_ACTUATOR_STATUS, .data_length_code = 8};
    for (int i = 0; i < 4; i++) {
        int16_t raw = 0;
        ads_read(ads_channel[i], &raw);
        int16_t angle10 = (int16_t)(raw_to_angle(raw, i) * 10);
        m.data[i * 2] = (uint8_t)(angle10 >> 8);
        m.data[i * 2 + 1] = (uint8_t)angle10;
    }
    twai_transmit(&m, pdMS_TO_TICKS(10));
}

void app_main(void) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    ledc_timer_config_t timer = {.speed_mode=LEDC_LOW_SPEED_MODE,.timer_num=LEDC_TIMER_0,.duty_resolution=LEDC_TIMER_16_BIT,.freq_hz=SERVO_FREQUENCY_HZ,.clk_cfg=LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    for (int i=0;i<4;i++) { ledc_channel_config_t c={.gpio_num=pwm_gpio[i],.speed_mode=LEDC_LOW_SPEED_MODE,.channel=i,.timer_sel=LEDC_TIMER_0,.duty=0}; ESP_ERROR_CHECK(ledc_channel_config(&c)); }

    i2c_config_t ic={.mode=I2C_MODE_MASTER,.sda_io_num=PIN_I2C_SDA,.scl_io_num=PIN_I2C_SCL,.sda_pullup_en=GPIO_PULLUP_ENABLE,.scl_pullup_en=GPIO_PULLUP_ENABLE,.master.clk_speed=400000};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0,&ic));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0,ic.mode,0,0,0));

    int64_t last_cmd = esp_timer_get_time();
    for (;;) {
        twai_message_t m;
        if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK && m.identifier == CAN_ID_ACTUATOR_CMD && m.data_length_code >= 8) {
            for (int i=0;i<4;i++) pwm_write(i, ((uint16_t)m.data[i*2] << 8) | m.data[i*2+1]);
            last_cmd = esp_timer_get_time();
        }
        if (esp_timer_get_time() - last_cmd > CAN_TIMEOUT_MS * 1000LL) for (int i=0;i<4;i++) pwm_write(i, 0);
        static int tick = 0; if (++tick >= 10) { send_status(); tick = 0; }
    }
    (void)TAG;
}