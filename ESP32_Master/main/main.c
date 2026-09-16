#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "app_config.h"
#include "boat_protocol.h"
#include "can_driver.h"
#include "input.h"
#include "sensor_manager.h"

/* Étiquette utilisée dans les messages de journalisation. */
static const char *TAG = "app";

/*
 * Convertit une valeur comprise entre -1000 et +1000
 * en commande servo comprise entre 0 et 1000.
 */
static uint16_t signed_to_servo(int16_t value)
{
    if (value < -1000) {
        value = -1000;
    }

    if (value > 1000) {
        value = 1000;
    }

    return (uint16_t)((value + 1000) / 2);
}

/*
 * Tâche principale de commande.
 *
 * Lit le joystick et les encodeurs puis envoie
 * les quatre commandes à l'ESP32 des actionneurs.
 */
static void control_task(void *arg)
{
    (void)arg;

    input_state_t input = { 0 };
    unsigned send_errors = 0;

    for (;;) {
        /* Lire les entrées utilisateur. */
        esp_err_t err = input_read(&input);

        if (err == ESP_OK) {
            /*
             * Si le bouton joystick est actif, utiliser l'axe X.
             * Sinon, utiliser l'encodeur du volant.
             */
            const int16_t steering =
                input.joystick_button ? input.joystick_x : input.wheel;

            /* Construire la commande des quatre actionneurs. */
            const boat_actuator_command_t command = {
                .turbine_g = signed_to_servo(steering),
                .turbine_d = signed_to_servo(steering),
                .reverse_g = input.throttle_g,
                .reverse_d = input.throttle_d,
            };

            /* Envoyer la commande sur le CAN interne. */
            err = boat_protocol_send_actuator_command(&command);
        }

        /* Limiter la fréquence des messages d'erreur. */
        if (err != ESP_OK) {
            send_errors++;

            if ((send_errors % 100U) == 1U) {
                ESP_LOGE(
                    TAG,
                    "Commande non envoyee: %s",
                    esp_err_to_name(err)
                );
            }
        }

        /* Période de commande : 10 ms, soit 100 Hz. */
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/*
 * Tâche de réception des positions des actionneurs.
 *
 * Les angles reçus sont exprimés en degrés multipliés par 10.
 */
static void actuator_status_task(void *arg)
{
    (void)arg;

    boat_actuator_status_t status;

    for (;;) {
        esp_err_t err = boat_protocol_receive_actuator_status(
            &status,
            ACTUATOR_STATUS_TIMEOUT_MS
        );

        if (err == ESP_OK) {
            ESP_LOGD(
                TAG,
                "Positions x10: TG=%d TD=%d RG=%d RD=%d",
                status.turbine_g_angle10,
                status.turbine_d_angle10,
                status.reverse_g_angle10,
                status.reverse_d_angle10
            );
        } else if (
            err != ESP_ERR_TIMEOUT &&
            err != ESP_ERR_NOT_FOUND
        ) {
            ESP_LOGW(
                TAG,
                "Reception CAN: %s",
                esp_err_to_name(err)
            );
        }
    }
}

/* Tâche de lecture périodique du MPU9250. */
static void imu_task(void *arg)
{
    (void)arg;

    for (;;) {
        sensor_manager_read_imu();
        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/* Tâche de lecture périodique du BME280. */
static void environment_task(void *arg)
{
    (void)arg;

    for (;;) {
        sensor_manager_read_environment();
        vTaskDelay(pdMS_TO_TICKS(ENVIRONMENT_PERIOD_MS));
    }
}

/*
 * Point d'entrée principal de l'application ESP-IDF.
 */
void app_main(void)
{
    /* Initialiser le CAN interne : TX GPIO17, RX GPIO16, 500 kbit/s. */
    ESP_ERROR_CHECK(
        can_driver_init(
            PIN_CAN_TX,
            PIN_CAN_RX,
            CAN_BITRATE
        )
    );

    /* Initialiser joystick et encodeurs. */
    ESP_ERROR_CHECK(input_init());

    /* Initialiser le bus I²C et les capteurs. */
    ESP_ERROR_CHECK(
        sensor_manager_init(
            PIN_I2C_SDA,
            PIN_I2C_SCL,
            I2C_FREQUENCY_HZ
        )
    );

    /* Tâche de commande principale, priorité élevée. */
    configASSERT(
        xTaskCreate(
            control_task,
            "control",
            4096,
            NULL,
            10,
            NULL
        ) == pdPASS
    );

    /* Tâche de réception CAN, priorité élevée. */
    configASSERT(
        xTaskCreate(
            actuator_status_task,
            "can_rx",
            4096,
            NULL,
            9,
            NULL
        ) == pdPASS
    );

    /* Tâche de lecture MPU9250. */
    configASSERT(
        xTaskCreate(
            imu_task,
            "imu",
            4096,
            NULL,
            5,
            NULL
        ) == pdPASS
    );

    /* Tâche de lecture BME280. */
    configASSERT(
        xTaskCreate(
            environment_task,
            "environment",
            4096,
            NULL,
            3,
            NULL
        ) == pdPASS
    );

    ESP_LOGI(TAG, "Console bateau initialisee");
}