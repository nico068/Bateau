// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#include <stdint.h>

/* Mesures physiques fournies par le BME280. */
typedef struct {
    float temperature_c;
    float pressure_hpa;
    float humidity_percent;
} bme280_data_t;

/*
 * Initialise le capteur et charge ses coefficients de calibration.
 * Le bus I2C doit avoir été créé avec i2c_new_master_bus().
 */
esp_err_t bme280_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t i2c_address,
    uint32_t frequency_hz
);

/* Lit la température, la pression et l'humidité courantes. */
esp_err_t bme280_read(bme280_data_t *data);
