// SPDX-License-Identifier: GPL-3.0-only

#include "bme280.h"

#include "freertos/FreeRTOS.h"

#include <stddef.h>

/* Coefficients de calibration stockés dans la mémoire du BME280. */
typedef struct {
    uint16_t t1;
    int16_t t2;
    int16_t t3;

    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;

    uint8_t h1;
    int16_t h2;
    uint8_t h3;
    int16_t h4;
    int16_t h5;
    int8_t h6;

    int32_t t_fine;
} calibration_t;

/* État interne du BME280. */
static calibration_t calibration;
static i2c_master_dev_handle_t device;

/* Lire plusieurs registres avec le pilote I2C moderne. */
static esp_err_t read_registers(
    uint8_t reg,
    uint8_t *data,
    size_t length
)
{
    return i2c_master_transmit_receive(
        device,
        &reg,
        1,
        data,
        length,
        20
    );
}

/* Écrire un registre avec le pilote I2C moderne. */
static esp_err_t write_register(
    uint8_t reg,
    uint8_t value
)
{
    const uint8_t data[2] = {
        reg,
        value
    };

    return i2c_master_transmit(
        device,
        data,
        sizeof(data),
        20
    );
}

/* Convertir deux octets little-endian en uint16_t. */
static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] |
           ((uint16_t)data[1] << 8);
}

/* Convertir deux octets little-endian en int16_t. */
static int16_t read_s16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

/* Étendre le signe d'une valeur codée sur 12 bits. */
static int16_t sign_extend_12(uint16_t value)
{
    if ((value & 0x0800U) != 0U) {
        value |= 0xF000U;
    }

    return (int16_t)value;
}

esp_err_t bme280_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t i2c_address,
    uint32_t frequency_hz
)
{
    if (bus_handle == NULL || frequency_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Ajouter le BME280 sur le bus I2C moderne. */
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = frequency_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(
        bus_handle,
        &device_config,
        &device
    );

    if (err != ESP_OK) {
        return err;
    }

    /* Vérifier l'identifiant du BME280. */
    uint8_t chip_id = 0;

    err = read_registers(
        0xD0,
        &chip_id,
        1
    );

    if (err != ESP_OK) {
        return err;
    }

    if (chip_id != 0x60) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Lire les coefficients température et pression. */
    uint8_t temperature_pressure[26];

    err = read_registers(
        0x88,
        temperature_pressure,
        sizeof(temperature_pressure)
    );

    if (err != ESP_OK) {
        return err;
    }

    /* Lire les coefficients d'humidité. */
    uint8_t humidity[7];

    err = read_registers(
        0xE1,
        humidity,
        sizeof(humidity)
    );

    if (err != ESP_OK) {
        return err;
    }

    /* Décoder les coefficients de calibration. */
    calibration.t1 = read_u16_le(temperature_pressure);
    calibration.t2 = read_s16_le(temperature_pressure + 2);
    calibration.t3 = read_s16_le(temperature_pressure + 4);

    calibration.p1 = read_u16_le(temperature_pressure + 6);
    calibration.p2 = read_s16_le(temperature_pressure + 8);
    calibration.p3 = read_s16_le(temperature_pressure + 10);
    calibration.p4 = read_s16_le(temperature_pressure + 12);
    calibration.p5 = read_s16_le(temperature_pressure + 14);
    calibration.p6 = read_s16_le(temperature_pressure + 16);
    calibration.p7 = read_s16_le(temperature_pressure + 18);
    calibration.p8 = read_s16_le(temperature_pressure + 20);
    calibration.p9 = read_s16_le(temperature_pressure + 22);

    calibration.h1 = temperature_pressure[25];
    calibration.h2 = read_s16_le(humidity);
    calibration.h3 = humidity[2];

    calibration.h4 = sign_extend_12(
        ((uint16_t)humidity[3] << 4) |
        (humidity[4] & 0x0FU)
    );

    calibration.h5 = sign_extend_12(
        ((uint16_t)humidity[5] << 4) |
        (humidity[4] >> 4)
    );

    calibration.h6 = (int8_t)humidity[6];

    /* Configurer le BME280. */
    err = write_register(0xF2, 0x01);

    if (err != ESP_OK) {
        return err;
    }

    err = write_register(0xF4, 0x27);

    if (err != ESP_OK) {
        return err;
    }

    return write_register(0xF5, 0xA0);
}

esp_err_t bme280_read(bme280_data_t *data)
{
    if (data == NULL || device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Lire pression, température et humidité brutes. */
    uint8_t raw[8];

    esp_err_t err = read_registers(
        0xF7,
        raw,
        sizeof(raw)
    );

    if (err != ESP_OK) {
        return err;
    }

    const int32_t raw_pressure =
        ((int32_t)raw[0] << 12) |
        ((int32_t)raw[1] << 4) |
        (raw[2] >> 4);

    const int32_t raw_temperature =
        ((int32_t)raw[3] << 12) |
        ((int32_t)raw[4] << 4) |
        (raw[5] >> 4);

    const int32_t raw_humidity =
        ((int32_t)raw[6] << 8) |
        raw[7];

    /* Compensation de la température selon Bosch. */
    const int32_t temperature_v1 =
        (((raw_temperature >> 3) -
          ((int32_t)calibration.t1 << 1)) *
         calibration.t2) >> 11;

    const int32_t temperature_v2 =
        (((((raw_temperature >> 4) -
            (int32_t)calibration.t1) *
           ((raw_temperature >> 4) -
            (int32_t)calibration.t1)) >> 12) *
         calibration.t3) >> 14;

    calibration.t_fine =
        temperature_v1 +
        temperature_v2;

    data->temperature_c =
        ((calibration.t_fine * 5 + 128) >> 8) /
        100.0f;

    /* Compensation de la pression avec des entiers 64 bits. */
    int64_t pressure_v1 =
        (int64_t)calibration.t_fine - 128000;

    int64_t pressure_v2 =
        pressure_v1 *
        pressure_v1 *
        calibration.p6;

    pressure_v2 +=
        (pressure_v1 * calibration.p5) << 17;

    pressure_v2 +=
        ((int64_t)calibration.p4) << 35;

    pressure_v1 =
        ((pressure_v1 *
          pressure_v1 *
          calibration.p3) >> 8) +
        ((pressure_v1 *
          calibration.p2) << 12);

    pressure_v1 =
        ((((int64_t)1 << 47) +
          pressure_v1) *
         calibration.p1) >> 33;

    if (pressure_v1 == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int64_t pressure =
        1048576 -
        raw_pressure;

    pressure =
        (((pressure << 31) -
          pressure_v2) *
         3125) /
        pressure_v1;

    pressure_v1 =
        (calibration.p9 *
         (pressure >> 13) *
         (pressure >> 13)) >> 25;

    pressure_v2 =
        (calibration.p8 * pressure) >> 19;

    pressure =
        ((pressure +
          pressure_v1 +
          pressure_v2) >> 8) +
        ((int64_t)calibration.p7 << 4);

    data->pressure_hpa =
        (float)pressure /
        25600.0f;

    /* Compensation de l'humidité selon Bosch. */
    int32_t humidity_value =
        calibration.t_fine - 76800;

    humidity_value =
        (((((raw_humidity << 14) -
            ((int32_t)calibration.h4 << 20) -
            ((int32_t)calibration.h5 *
             humidity_value)) +
           16384) >> 15) *
         (((((((humidity_value *
                calibration.h6) >> 10) *
              (((humidity_value *
                 calibration.h3) >> 11) +
               32768)) >> 10) +
            2097152) *
           calibration.h2 +
           8192) >> 14));

    humidity_value -=
        (((((humidity_value >> 15) *
            (humidity_value >> 15)) >> 7) *
          calibration.h1) >> 4);

    /* Limiter l'humidité entre 0 et 100 %. */
    if (humidity_value < 0) {
        humidity_value = 0;
    }

    if (humidity_value > 419430400) {
        humidity_value = 419430400;
    }

    data->humidity_percent =
        (humidity_value >> 12) /
        1024.0f;

    return ESP_OK;
}