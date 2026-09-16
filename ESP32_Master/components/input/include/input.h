#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * État normalisé de toutes les commandes physiques du poste maître.
 *
 * Plages :
 * - joystick et volant : -1000 à +1000 ;
 * - manettes des gaz   : 0 à 1000 ;
 * - bouton             : true lorsqu'il est physiquement appuyé.
 */
typedef struct {
    /* Axe horizontal du joystick : gauche=-1000, centre=0, droite=+1000. */
    int16_t joystick_x;

    /* Axe vertical du joystick : arrière=-1000, centre=0, avant=+1000. */
    int16_t joystick_y;

    /* Position normalisée de la manette des gaz gauche. */
    uint16_t throttle_g;

    /* Position normalisée de la manette des gaz droite. */
    uint16_t throttle_d;

    /* Position normalisée du volant : gauche=-1000, centre=0, droite=+1000. */
    int16_t wheel;

    /* État logique du bouton du joystick, actif à l'état bas. */
    bool joystick_button;
} input_state_t;

/*
 * Initialise :
 * - l'ADC1 pour les deux axes du joystick ;
 * - l'entrée GPIO du bouton ;
 * - trois unités PCNT pour les encodeurs incrémentaux.
 *
 * Retourne ESP_OK ou le premier code d'erreur reçu d'un pilote ESP-IDF.
 */
esp_err_t input_init(void);

/*
 * Lit toutes les commandes et remplit la structure fournie.
 *
 * Retourne :
 * - ESP_OK              : lecture réussie ;
 * - ESP_ERR_INVALID_ARG : pointeur state nul ;
 * - autre ESP_ERR_*     : erreur de lecture PCNT.
 */
esp_err_t input_read(
    input_state_t *state
);

/*
 * Remet à zéro les compteurs des deux manettes et du volant.
 * Cette opération ne vérifie pas leur position mécanique réelle.
 */
esp_err_t input_reset_encoders(void);