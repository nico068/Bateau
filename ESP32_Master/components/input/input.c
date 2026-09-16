#include "input.h"

#include "app_config.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"

#include <stddef.h>

/* Paramètres de conversion des entrées analogiques. */
#define ADC_CENTER_RAW              2048
#define ADC_HALF_RANGE_RAW          2048
#define INPUT_NORMALIZED_MAX        1000

/* Filtre anti-rebond des encodeurs, exprimé en nanosecondes. */
#define PCNT_FILTER_NS              1000

/* Limites matérielles des unités pulse counter. */
#define PCNT_COUNTER_HIGH_LIMIT     32767
#define PCNT_COUNTER_LOW_LIMIT      (-32768)

/* Canaux ADC utilisés par le joystick sur l'ESP32 classique. */
#define JOYSTICK_X_ADC_CHANNEL      ADC_CHANNEL_0
#define JOYSTICK_Y_ADC_CHANNEL      ADC_CHANNEL_3

/* Handle partagé de l'unité ADC1. */
static adc_oneshot_unit_handle_t adc_handle;

/* Handles des trois compteurs quadrature. */
static pcnt_unit_handle_t throttle_g_unit;
static pcnt_unit_handle_t throttle_d_unit;
static pcnt_unit_handle_t wheel_unit;

/* Convertit une lecture ADC 12 bits en valeur comprise entre -1000 et +1000. */
static int16_t adc_normalized(adc_channel_t channel)
{
    int raw_value = 0;

    if (adc_oneshot_read(adc_handle, channel, &raw_value) != ESP_OK) {
        return 0;
    }

    int normalized =
        (raw_value - ADC_CENTER_RAW) *
        INPUT_NORMALIZED_MAX /
        ADC_HALF_RANGE_RAW;

    /* Limiter les écarts liés aux tolérances de l'ADC et du joystick. */
    if (normalized < -INPUT_NORMALIZED_MAX) {
        normalized = -INPUT_NORMALIZED_MAX;
    }

    if (normalized > INPUT_NORMALIZED_MAX) {
        normalized = INPUT_NORMALIZED_MAX;
    }

    /* Supprimer les petites oscillations autour du centre. */
    if (
        normalized > -JOYSTICK_DEAD_ZONE &&
        normalized < JOYSTICK_DEAD_ZONE
    ) {
        normalized = 0;
    }

    return (int16_t)normalized;
}

/*
 * Configure une unité pulse counter pour un encodeur quadrature A/B.
 * Les deux canaux matériels sont utilisés afin de compter les quatre fronts.
 */
static esp_err_t encoder_init(
    pcnt_unit_handle_t *unit_handle,
    int pin_a,
    int pin_b
)
{
    /* Créer l'unité et définir sa plage de comptage. */
    const pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_COUNTER_HIGH_LIMIT,
        .low_limit = PCNT_COUNTER_LOW_LIMIT,
    };

    esp_err_t err = pcnt_new_unit(
        &unit_config,
        unit_handle
    );

    if (err != ESP_OK) {
        return err;
    }

    /* Filtrer les impulsions très courtes dues aux parasites. */
    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = PCNT_FILTER_NS,
    };

    err = pcnt_unit_set_glitch_filter(
        *unit_handle,
        &filter_config
    );

    if (err != ESP_OK) {
        pcnt_del_unit(*unit_handle);
        *unit_handle = NULL;
        return err;
    }

    /* Canal A : le niveau B détermine le sens du comptage. */
    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };
    pcnt_channel_handle_t channel_a;

    err = pcnt_new_channel(
        *unit_handle,
        &channel_a_config,
        &channel_a
    );

    if (err != ESP_OK) {
        pcnt_del_unit(*unit_handle);
        *unit_handle = NULL;
        return err;
    }

    err = pcnt_channel_set_edge_action(
        channel_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE
    );

    if (err != ESP_OK) {
        pcnt_del_unit(*unit_handle);
        *unit_handle = NULL;
        return err;
    }

    err = pcnt_channel_set_level_action(
        channel_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    );

    if (err != ESP_OK) {
        pcnt_del_unit(*unit_handle);
        *unit_handle = NULL;
        return err;
    }

    /* Canal B : logique complémentaire pour le décodage quadrature. */
    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
    };
    pcnt_channel_handle_t channel_b;

    err = pcnt_new_channel(
        *unit_handle,
        &channel_b_config,
        &channel_b
    );

    if (err != ESP_OK) {
        pcnt_del_unit(*unit_handle);
        *unit_handle = NULL;
        return err;
    }

    err = pcnt_channel_set_edge_action(
        channel_b,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE
    );

    if (err != ESP_OK) {
        pcnt_del_unit(*unit_handle);
        *unit_handle = NULL;
        return err;
    }

    err = pcnt_channel_set_level_action(
        channel_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    );

    if (err != ESP_OK) {
        pcnt_del_unit(*unit_handle);
        *unit_handle = NULL;
        return err;
    }

    /* Démarrer le compteur après sa configuration complète. */
    err = pcnt_unit_enable(*unit_handle);
    if (err != ESP_OK) {
        return err;
    }

    return pcnt_unit_start(*unit_handle);
}

/* Convertit un compteur vers une consigne non signée comprise entre 0 et 1000. */
static uint16_t scale_unsigned(
    int value,
    int minimum,
    int maximum
)
{
    if (maximum <= minimum) {
        return 0;
    }

    if (value < minimum) {
        value = minimum;
    }

    if (value > maximum) {
        value = maximum;
    }

    return (uint16_t)(
        ((value - minimum) * INPUT_NORMALIZED_MAX) /
        (maximum - minimum)
    );
}

/* Convertit un compteur vers une consigne signée comprise entre -1000 et +1000. */
static int16_t scale_signed(
    int value,
    int minimum,
    int maximum
)
{
    if (maximum <= minimum) {
        return 0;
    }

    if (value < minimum) {
        value = minimum;
    }

    if (value > maximum) {
        value = maximum;
    }

    return (int16_t)(
        -INPUT_NORMALIZED_MAX +
        ((value - minimum) * (2 * INPUT_NORMALIZED_MAX)) /
        (maximum - minimum)
    );
}

esp_err_t input_init(void)
{
    /* Créer l'unité ADC1 avec le pilote moderne oneshot. */
    const adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t err = adc_oneshot_new_unit(
        &adc_config,
        &adc_handle
    );

    if (err != ESP_OK) {
        return err;
    }

    /* Configurer les deux canaux du joystick en 12 bits. */
    const adc_oneshot_chan_cfg_t channel_config = {
        /* ESP-IDF 5.5 remplace ADC_ATTEN_DB_11 par ADC_ATTEN_DB_12. */
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    err = adc_oneshot_config_channel(
        adc_handle,
        JOYSTICK_X_ADC_CHANNEL,
        &channel_config
    );

    if (err != ESP_OK) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return err;
    }

    err = adc_oneshot_config_channel(
        adc_handle,
        JOYSTICK_Y_ADC_CHANNEL,
        &channel_config
    );

    if (err != ESP_OK) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return err;
    }

    /* Créer les trois compteurs quadrature. */
    err = encoder_init(
        &throttle_g_unit,
        PIN_THROTTLE_G_A,
        PIN_THROTTLE_G_B
    );

    if (err != ESP_OK) {
        return err;
    }

    err = encoder_init(
        &throttle_d_unit,
        PIN_THROTTLE_D_A,
        PIN_THROTTLE_D_B
    );

    if (err != ESP_OK) {
        return err;
    }

    return encoder_init(
        &wheel_unit,
        PIN_WHEEL_A,
        PIN_WHEEL_B
    );
}

esp_err_t input_reset_encoders(void)
{
    /* Remettre les trois compteurs à zéro. */
    esp_err_t err = pcnt_unit_clear_count(throttle_g_unit);
    if (err != ESP_OK) {
        return err;
    }

    err = pcnt_unit_clear_count(throttle_d_unit);
    if (err != ESP_OK) {
        return err;
    }

    return pcnt_unit_clear_count(wheel_unit);
}

esp_err_t input_read(input_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int left = 0;
    int right = 0;
    int wheel = 0;

    /* Lire les trois compteurs matériels. */
    esp_err_t err = pcnt_unit_get_count(
        throttle_g_unit,
        &left
    );

    if (err != ESP_OK) {
        return err;
    }

    err = pcnt_unit_get_count(
        throttle_d_unit,
        &right
    );

    if (err != ESP_OK) {
        return err;
    }

    err = pcnt_unit_get_count(
        wheel_unit,
        &wheel
    );

    if (err != ESP_OK) {
        return err;
    }

    /* Lire les axes analogiques. */
    state->joystick_x = adc_normalized(JOYSTICK_X_ADC_CHANNEL);
    state->joystick_y = adc_normalized(JOYSTICK_Y_ADC_CHANNEL);

    /* Appliquer les limites configurées dans app_config.h. */
    state->throttle_g = scale_unsigned(
        left,
        THROTTLE_G_COUNT_MIN,
        THROTTLE_G_COUNT_MAX
    );

    state->throttle_d = scale_unsigned(
        right,
        THROTTLE_D_COUNT_MIN,
        THROTTLE_D_COUNT_MAX
    );

    state->wheel = scale_signed(
        wheel,
        WHEEL_COUNT_MIN,
        WHEEL_COUNT_MAX
    );

    return ESP_OK;
}