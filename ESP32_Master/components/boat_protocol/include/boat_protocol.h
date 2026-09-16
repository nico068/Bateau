#pragma once

#include "esp_err.h"

#include <stdint.h>

/* Identifiants CAN standards du bus interne. */
#define CAN_ID_ACTUATOR_CMD       0x210
#define CAN_ID_ACTUATOR_STATUS    0x290

/* Paramètres de codage des consignes et positions. */
#define CAN_ACTUATOR_DATA_LENGTH  8
#define SERVO_COMMAND_MAX         1000
#define ANGLE_SCALE               10
#define CAN_COMMAND_TIMEOUT_MS    5

/*
 * Consignes maître -> ESP32 n°2.
 *
 * Chaque champ représente une commande servo RC normalisée :
 * - 0    : impulsion minimale, environ 1000 µs ;
 * - 500  : impulsion centrale, environ 1500 µs ;
 * - 1000 : impulsion maximale, environ 2000 µs.
 */
typedef struct {
    uint16_t turbine_g;
    uint16_t turbine_d;
    uint16_t reverse_g;
    uint16_t reverse_d;
} boat_actuator_command_t;

/*
 * Positions ESP32 n°2 -> maître.
 *
 * Les angles sont transmis en degrés multipliés par ANGLE_SCALE.
 * Exemple : 12,3 degrés est transmis sous la forme 123.
 */
typedef struct {
    int16_t turbine_g_angle10;
    int16_t turbine_d_angle10;
    int16_t reverse_g_angle10;
    int16_t reverse_d_angle10;
} boat_actuator_status_t;

/* Encode et envoie la trame CAN standard 0x210. */
esp_err_t boat_protocol_send_actuator_command(
    const boat_actuator_command_t *command
);

/* Reçoit et décode la trame CAN standard 0x290. */
esp_err_t boat_protocol_receive_actuator_status(
    boat_actuator_status_t *status,
    uint32_t timeout_ms
);