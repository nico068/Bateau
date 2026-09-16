// SPDX-License-Identifier: GPL-3.0-only

#include "sensor_manager.h"
#include "app_config.h"
#include "bme280.h"
#include "mpu9250.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

/* Étiquette utilisée par le système de journalisation ESP-IDF. */
static const char *TAG = "sensors";

/* Dernier état valide connu pour l'ensemble des capteurs. */
static sensor_state_t state;

/* Protège la structure state contre les accès concurrents des tâches. */
static SemaphoreHandle_t lock;

/* Disponibilité déterminée une seule fois pendant l'initialisation. */
static bool bme_available;
static bool mpu_available;
static i2c_master_bus_handle_t i2c_bus;

/*
* Scanner toutes les adresses I2C valides.
*
* Plage réservée aux périphériques :
* - 0x03 à 0x77.
*
* Une adresse détectée est affichée dans le moniteur série.
*/
static void i2c_scan_bus(void)
{
    ESP_LOGI(TAG, "Debut du scan I2C");
    
    static const uint8_t addresses[] = {
        0x68,
        0x69,
        0x76,
        0x77
    };

    for (size_t i = 0; i < sizeof(addresses); i++) {
        uint8_t address = addresses[i];

        if (i2c_master_probe(i2c_bus, address, 10) == ESP_OK) {
            ESP_LOGI(
                TAG,
                "Peripherique I2C detecte : 0x%02X",
                address
            );
        }
    }

    ESP_LOGI(TAG, "Fin du scan I2C");
}

esp_err_t sensor_manager_init(
    int sda,
    int scl,
    uint32_t frequency_hz
)
{
    /* Toutes les valeurs et tous les indicateurs démarrent à zéro/false. */
    memset(&state, 0, sizeof(state));

    /* Le mutex permet aux tâches IMU, environnement et télémétrie de coexister. */
    lock = xSemaphoreCreateMutex();
    if (lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Configuration du bus I2C moderne en mode maître. */
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus);

    if (err != ESP_OK) {
        vSemaphoreDelete(lock);
        lock = NULL;
        return err;
    }

    /* Scanner le bus avant d'initialiser les capteurs. */
    i2c_scan_bus();

    /*
     * L'absence d'un capteur secondaire ne doit pas empêcher le contrôleur
     * maître de démarrer ni d'envoyer les commandes CAN.
     */
    err = bme280_init(
        i2c_bus,
        BME280_I2C_ADDRESS,
        frequency_hz
    );
    bme_available = err == ESP_OK;

    if (!bme_available) {
        ESP_LOGW(
            TAG,
            "BME280 indisponible: %s",
            esp_err_to_name(err)
        );
    }

    err = mpu9250_init(
        i2c_bus,
        MPU9250_I2C_ADDRESS,
        AK8963_I2C_ADDRESS,
        frequency_hz
    );
    mpu_available = err == ESP_OK;

    if (!mpu_available) {
        ESP_LOGW(
            TAG,
            "MPU9250 indisponible: %s",
            esp_err_to_name(err)
        );
    }

    return ESP_OK;
}

esp_err_t sensor_manager_read_environment(void)
{
    /* Le pilote n'est pas interrogé si le BME280 était absent au démarrage. */
    if (!bme_available) {
        return ESP_ERR_NOT_FOUND;
    }

    bme280_data_t data;
    esp_err_t err = bme280_read(&data);

    /* Mettre à jour l'état partagé dans une section critique protégée. */
    xSemaphoreTake(lock, portMAX_DELAY);
    state.environment_valid = err == ESP_OK;

    if (err == ESP_OK) {
        state.temperature_c = data.temperature_c;
        state.pressure_hpa = data.pressure_hpa;
        state.humidity_percent = data.humidity_percent;

        /* Horodatage de la dernière mesure environnementale valide. */
        state.environment_timestamp_us = esp_timer_get_time();
    }

    xSemaphoreGive(lock);
    return err;
}

esp_err_t sensor_manager_read_imu(void)
{
    /* Le pilote n'est pas interrogé si le MPU9250 était absent au démarrage. */
    if (!mpu_available) {
        return ESP_ERR_NOT_FOUND;
    }

    mpu9250_data_t data;
    esp_err_t err = mpu9250_read(&data);

    /* Mettre à jour l'état partagé dans une section critique protégée. */
    xSemaphoreTake(lock, portMAX_DELAY);
    state.imu_valid = err == ESP_OK;

    if (err == ESP_OK) {
        /* Accélérations déjà converties en g par le pilote MPU9250. */
        state.ax_g = data.ax_g;
        state.ay_g = data.ay_g;
        state.az_g = data.az_g;

        /* Retirer les biais statiques configurés pour le gyroscope. */
        state.gx_dps = data.gx_dps - GYRO_BIAS_X_DPS;
        state.gy_dps = data.gy_dps - GYRO_BIAS_Y_DPS;
        state.gz_dps = data.gz_dps - GYRO_BIAS_Z_DPS;

        /*
         * Une lecture MPU9250 valide ne garantit pas qu'une nouvelle mesure
         * magnétique soit disponible. Le pilote fournit un drapeau distinct.
         */
        state.magnetic_valid = data.magnetic_valid;

        if (data.magnetic_valid) {
            /* Appliquer les corrections hard-iron et soft-iron configurées. */
            state.mx_ut = (data.mx_ut - MAG_OFFSET_X_UT) * MAG_SCALE_X;
            state.my_ut = (data.my_ut - MAG_OFFSET_Y_UT) * MAG_SCALE_Y;
            state.mz_ut = (data.mz_ut - MAG_OFFSET_Z_UT) * MAG_SCALE_Z;
        }

        /* Horodatage de la dernière mesure inertielle valide. */
        state.imu_timestamp_us = esp_timer_get_time();
    } else {
        /* Interdire l'utilisation d'une ancienne mesure magnétique. */
        state.magnetic_valid = false;
    }

    xSemaphoreGive(lock);
    return err;
}

esp_err_t sensor_manager_get_state(sensor_state_t *out)
{
    /* Refuser un pointeur de destination invalide. */
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Le gestionnaire doit avoir été initialisé avant toute copie. */
    if (lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Copier l'état complet sous mutex pour obtenir un instantané cohérent. */
    xSemaphoreTake(lock, portMAX_DELAY);
    *out = state;
    xSemaphoreGive(lock);

    return ESP_OK;
}
