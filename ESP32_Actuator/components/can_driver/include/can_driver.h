// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include "esp_err.h"
#include "driver/twai.h"

#include <stdint.h>

/* Longueur maximale de la charge utile d'une trame CAN classique. */
#define CAN_CLASSIC_MAX_DATA_LENGTH  8

/* Identifiant maximal d'une trame CAN standard sur 11 bits. */
#define CAN_STANDARD_ID_MAX          0x7FF

/*
 * Représentation générique d'une trame CAN standard reçue.
 *
 * Le projet utilise uniquement :
 * - des trames CAN classiques ;
 * - des identifiants standards sur 11 bits ;
 * - des trames de données, pas des trames RTR.
 */
typedef struct {
    /* Identifiant standard compris entre 0x000 et 0x7FF. */
    uint32_t identifier;

    /* Nombre d'octets valides dans data, compris entre 0 et 8. */
    uint8_t data_length;

    /* Charge utile de la trame CAN classique. */
    uint8_t data[CAN_CLASSIC_MAX_DATA_LENGTH];
} can_frame_t;

/*
 * Installe et démarre le contrôleur TWAI interne de l'ESP32.
 *
 * Paramètres :
 * - tx_gpio : GPIO relié à l'entrée TXD du transceiver CAN ;
 * - rx_gpio : GPIO relié à la sortie RXD du transceiver CAN ;
 * - bitrate : débit demandé en bits par seconde.
 *
 * Débits acceptés :
 * - 125000 bit/s ;
 * - 250000 bit/s ;
 * - 500000 bit/s ;
 * - 1000000 bit/s.
 */
esp_err_t can_driver_init(
    int tx_gpio,
    int rx_gpio,
    uint32_t bitrate
);

/*
 * Envoie une trame CAN standard.
 *
 * Paramètres :
 * - identifier : identifiant standard 11 bits ;
 * - data       : charge utile, ou NULL si length vaut zéro ;
 * - length     : nombre d'octets, entre 0 et 8 ;
 * - timeout_ms : attente maximale dans la file d'émission TX.
 */
esp_err_t can_driver_send(
    uint32_t identifier,
    const uint8_t *data,
    uint8_t length,
    uint32_t timeout_ms
);

/*
 * Attend et récupère une trame CAN standard depuis la file RX.
 *
 * Les trames étendues et RTR sont consommées mais rejetées avec :
 *
 *     ESP_ERR_NOT_SUPPORTED
 */
esp_err_t can_driver_receive(
    can_frame_t *frame,
    uint32_t timeout_ms
);

/*
 * Copie les informations de diagnostic du contrôleur TWAI :
 * - état du bus ;
 * - compteurs d'erreurs ;
 * - nombre de trames reçues ;
 * - nombre de trames transmises ;
 * - occupation des files TX/RX.
 */
esp_err_t can_driver_get_status(
    twai_status_info_t *status
);

/*
 * Lance la séquence de récupération après un état bus-off.
 *
 * Cette opération est asynchrone :
 * 1. can_driver_recover() démarre la récupération ;
 * 2. le contrôleur émet une alerte TWAI_ALERT_BUS_RECOVERED ;
 * 3. twai_start() doit être appelé pour reprendre les échanges.
 */
esp_err_t can_driver_recover(void);