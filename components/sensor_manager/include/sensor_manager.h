#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * État consolidé des capteurs du contrôleur maître.
 *
 * Cette structure rassemble les mesures du BME280, du MPU9250 et de
 * l'AK8963. Les indicateurs de validité doivent être contrôlés avant
 * d'utiliser les valeurs associées.
 */
typedef struct {
    /* Mesures environnementales fournies par le BME280. */
    float temperature_c;       /* Température en degrés Celsius. */
    float pressure_hpa;        /* Pression atmosphérique en hPa. */
    float humidity_percent;    /* Humidité relative en pourcentage. */

    /* Accélération mesurée par le MPU9250, exprimée en g. */
    float ax_g;
    float ay_g;
    float az_g;

    /* Vitesse angulaire mesurée par le MPU9250, exprimée en °/s. */
    float gx_dps;
    float gy_dps;
    float gz_dps;

    /* Champ magnétique mesuré par l'AK8963, exprimé en µT. */
    float mx_ut;
    float my_ut;
    float mz_ut;

    /* Validité du dernier groupe de mesures environnementales. */
    bool environment_valid;

    /* Validité de la dernière mesure accéléromètre/gyroscope. */
    bool imu_valid;

    /* Validité de la dernière mesure du magnétomètre. */
    bool magnetic_valid;

    /*
     * Horodatages en microsecondes depuis le démarrage de l'ESP32.
     * Ils sont produits par esp_timer_get_time().
     */
    int64_t environment_timestamp_us;
    int64_t imu_timestamp_us;
} sensor_state_t;

/*
 * Initialise le bus I2C numéro 0 ainsi que le BME280 et le MPU9250.
 *
 * Paramètres :
 * - sda          : numéro GPIO utilisé pour la ligne I2C SDA ;
 * - scl          : numéro GPIO utilisé pour la ligne I2C SCL ;
 * - frequency_hz : fréquence du bus I2C en hertz.
 *
 * La fonction retourne ESP_OK si le bus I2C a été initialisé, même si
 * un capteur secondaire est absent. L'absence du capteur sera signalée
 * par son indicateur de validité et par un message dans le journal.
 */
esp_err_t sensor_manager_init(
    int sda,
    int scl,
    uint32_t frequency_hz
);

/*
 * Lit le BME280 et met à jour les mesures environnementales partagées.
 *
 * Valeurs de retour principales :
 * - ESP_OK            : lecture réussie ;
 * - ESP_ERR_NOT_FOUND : BME280 absent lors de l'initialisation ;
 * - autre ESP_ERR_*   : erreur de communication ou de conversion.
 */
esp_err_t sensor_manager_read_environment(void);

/*
 * Lit le MPU9250 et l'AK8963, applique les calibrations configurées,
 * puis met à jour les mesures inertielles partagées.
 *
 * Valeurs de retour principales :
 * - ESP_OK            : lecture du MPU9250 réussie ;
 * - ESP_ERR_NOT_FOUND : MPU9250 absent lors de l'initialisation ;
 * - autre ESP_ERR_*   : erreur de communication I2C.
 */
esp_err_t sensor_manager_read_imu(void);

/*
 * Copie de manière atomique le dernier état connu des capteurs.
 *
 * Le mutex interne protège la structure pendant la copie. L'appelant
 * reçoit une copie indépendante qu'il peut lire sans verrou supplémentaire.
 *
 * Paramètre :
 * - state : destination de la copie.
 *
 * Valeurs de retour :
 * - ESP_OK              : copie réussie ;
 * - ESP_ERR_INVALID_ARG : pointeur state nul.
 */
esp_err_t sensor_manager_get_state(
    sensor_state_t *state
);