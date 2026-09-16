// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Mesures produites par le MPU9250 et son magnétomètre AK8963.
 *
 * Unités :
 * - accélération : g ;
 * - vitesse angulaire : degrés par seconde ;
 * - champ magnétique : microteslas.
 */
typedef struct {
    /* Accélération suivant les trois axes, exprimée en g. */
    float ax_g;
    float ay_g;
    float az_g;

    /* Vitesse angulaire suivant les trois axes, exprimée en °/s. */
    float gx_dps;
    float gy_dps;
    float gz_dps;

    /* Champ magnétique suivant les trois axes, exprimé en µT. */
    float mx_ut;
    float my_ut;
    float mz_ut;

    /*
     * true  : une nouvelle mesure magnétique valide a été lue ;
     * false : aucune nouvelle mesure ou saturation du magnétomètre.
     */
    bool magnetic_valid;
} mpu9250_data_t;

/*
 * Initialise le MPU9250 et le magnétomètre AK8963.
 *
 * Paramètres :
 * - bus_handle            : bus I2C maître moderne déjà initialisé ;
 * - mpu_address          : adresse I2C du MPU9250, généralement 0x68 ;
 * - magnetometer_address : adresse I2C de l'AK8963, normalement 0x0C ;
 * - frequency_hz         : fréquence des périphériques sur le bus.
 *
 * Valeurs de retour :
 * - ESP_OK               : initialisation réussie ;
 * - ESP_ERR_NOT_FOUND    : identifiant MPU9250 incompatible ;
 * - autre code ESP_ERR_* : erreur de communication I2C.
 *
 * Le bus I2C moderne doit être initialisé avant cet appel.
 */
esp_err_t mpu9250_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t mpu_address,
    uint8_t magnetometer_address,
    uint32_t frequency_hz
);

/*
 * Lit l'accéléromètre, le gyroscope et, si disponible, le magnétomètre.
 *
 * Paramètre :
 * - data : structure de destination des mesures converties.
 *
 * Valeurs de retour :
 * - ESP_OK              : lecture MPU9250 réussie ;
 * - ESP_ERR_INVALID_ARG : pointeur data nul ;
 * - autre code ESP_ERR_*: erreur de communication I2C.
 *
 * L'absence d'une nouvelle mesure magnétique ne fait pas échouer la
 * fonction. Dans ce cas, data->magnetic_valid vaut false.
 */
esp_err_t mpu9250_read(
    mpu9250_data_t *data
);