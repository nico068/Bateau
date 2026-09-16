// SPDX-License-Identifier: GPL-3.0-only

#include "boat_protocol.h"
#include "can_driver.h"

#include <stddef.h>

/*
 * Limiter une consigne avant de l'insérer
 * dans la trame CAN.
 */
static uint16_t clamp_servo_command(
    uint16_t value
)
{
    if (value > SERVO_COMMAND_MAX) {
        return SERVO_COMMAND_MAX;
    }

    return value;
}

/*
 * Écrire un entier 16 bits dans l'ordre réseau :
 * octet fort en premier, puis octet faible.
 */
static void put_u16_be(
    uint8_t *data,
    uint16_t value
)
{
    data[0] =
        (uint8_t)(value >> 8);

    data[1] =
        (uint8_t)value;
}

/*
 * Lire un entier signé 16 bits codé en big-endian.
 */
static int16_t get_s16_be(
    const uint8_t *data
)
{
    return (int16_t)(
        ((uint16_t)data[0] << 8) |
        data[1]
    );
}

esp_err_t boat_protocol_send_actuator_command(
    const boat_actuator_command_t *command
)
{
    /*
     * Ne jamais déréférencer un pointeur nul
     * fourni par l'appelant.
     */
    if (command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * La trame 0x210 contient quatre consignes
     * de 16 bits :
     *
     * - octets 0-1 : turbine gauche ;
     * - octets 2-3 : turbine droite ;
     * - octets 4-5 : reverse gauche ;
     * - octets 6-7 : reverse droite.
     */
    uint8_t data[CAN_ACTUATOR_DATA_LENGTH];

    put_u16_be(
        data,
        clamp_servo_command(
            command->turbine_g
        )
    );

    put_u16_be(
        data + 2,
        clamp_servo_command(
            command->turbine_d
        )
    );

    put_u16_be(
        data + 4,
        clamp_servo_command(
            command->reverse_g
        )
    );

    put_u16_be(
        data + 6,
        clamp_servo_command(
            command->reverse_d
        )
    );

    /*
     * Envoyer la trame au pilote CAN générique.
     */
    return can_driver_send(
        CAN_ID_ACTUATOR_CMD,
        data,
        sizeof(data),
        CAN_COMMAND_TIMEOUT_MS
    );
}

esp_err_t boat_protocol_receive_actuator_status(
    boat_actuator_status_t *status,
    uint32_t timeout_ms
)
{
    /*
     * Ne jamais écrire dans une adresse nulle.
     */
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    can_frame_t frame;

    /*
     * Attendre une trame depuis la file
     * de réception du pilote CAN.
     */
    esp_err_t err =
        can_driver_receive(
            &frame,
            timeout_ms
        );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Vérifier l'identifiant et la longueur
     * avant tout décodage.
     */
    if (
        frame.identifier != CAN_ID_ACTUATOR_STATUS ||
        frame.data_length != CAN_ACTUATOR_DATA_LENGTH
    ) {
        return ESP_ERR_NOT_FOUND;
    }

    /*
     * La trame 0x290 contient quatre angles signés
     * de 16 bits exprimés en degrés x10 :
     *
     * - octets 0-1 : turbine gauche ;
     * - octets 2-3 : turbine droite ;
     * - octets 4-5 : reverse gauche ;
     * - octets 6-7 : reverse droite.
     */
    status->turbine_g_angle10 =
        get_s16_be(
            frame.data
        );

    status->turbine_d_angle10 =
        get_s16_be(
            frame.data + 2
        );

    status->reverse_g_angle10 =
        get_s16_be(
            frame.data + 4
        );

    status->reverse_d_angle10 =
        get_s16_be(
            frame.data + 6
        );

    return ESP_OK;
}