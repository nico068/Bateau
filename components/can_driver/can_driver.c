#include "can_driver.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#include "can_driver.h"
#include "freertos/FreeRTOS.h"

#include <string.h>

/*
 * Convertit un débit en bits par seconde vers
 * la structure de temporisation attendue par TWAI.
 */
static esp_err_t get_timing(
    uint32_t bitrate,
    twai_timing_config_t *timing
)
{
    /* Refuser un pointeur de destination invalide. */
    if (timing == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (bitrate) {
        case 125000:
            *timing =
                (twai_timing_config_t)
                TWAI_TIMING_CONFIG_125KBITS();
            break;

        case 250000:
            *timing =
                (twai_timing_config_t)
                TWAI_TIMING_CONFIG_250KBITS();
            break;

        case 500000:
            *timing =
                (twai_timing_config_t)
                TWAI_TIMING_CONFIG_500KBITS();
            break;

        case 1000000:
            *timing =
                (twai_timing_config_t)
                TWAI_TIMING_CONFIG_1MBITS();
            break;

        default:
            /*
             * Refuser tout débit qui ne possède pas
             * de configuration TWAI prédéfinie.
             */
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

esp_err_t can_driver_init(
    int tx_gpio,
    int rx_gpio,
    uint32_t bitrate
)
{
    /*
     * Sélectionner les paramètres temporels correspondant
     * au débit demandé.
     */
    twai_timing_config_t timing;

    esp_err_t err =
        get_timing(
            bitrate,
            &timing
        );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Mode normal :
     * - transmission active ;
     * - réception active ;
     * - participation aux acquittements CAN.
     *
     * Le SN65HVD230 réalise uniquement l'adaptation électrique
     * entre les signaux logiques ESP32 et CANH/CANL.
     */
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(
            tx_gpio,
            rx_gpio,
            TWAI_MODE_NORMAL
        );

    /*
     * Activer les alertes utilisées par le diagnostic :
     * - bus-off ;
     * - bus récupéré ;
     * - erreur passive ;
     * - file RX pleine.
     */
    general.alerts_enabled =
        TWAI_ALERT_BUS_OFF |
        TWAI_ALERT_BUS_RECOVERED |
        TWAI_ALERT_ERR_PASS |
        TWAI_ALERT_RX_QUEUE_FULL;

    /*
     * Le pilote CAN ne connaît pas les identifiants métier.
     * Le filtrage est réalisé plus haut par boat_protocol.
     */
    twai_filter_config_t filter =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /*
     * Installer le contrôleur TWAI.
     */
    err =
        twai_driver_install(
            &general,
            &timing,
            &filter
        );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Démarrer le contrôleur après l'installation
     * de sa configuration.
     */
    err =
        twai_start();

    if (err != ESP_OK) {
        /*
         * Ne pas laisser un pilote installé mais inutilisable.
         */
        twai_driver_uninstall();
    }

    return err;
}

esp_err_t can_driver_send(
    uint32_t identifier,
    const uint8_t *data,
    uint8_t length,
    uint32_t timeout_ms
)
{
    /*
     * Vérifier :
     * - le pointeur data ;
     * - la longueur maximale CAN classique ;
     * - l'identifiant standard sur 11 bits.
     */
    if (
        (data == NULL && length != 0) ||
        length > CAN_CLASSIC_MAX_DATA_LENGTH ||
        identifier > CAN_STANDARD_ID_MAX
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * L'initialisation à zéro sélectionne une trame standard
     * de données :
     * - extd = 0 ;
     * - rtr  = 0 ;
     * - self = 0.
     */
    twai_message_t message = {
        .identifier = identifier,
        .data_length_code = length
    };

    /*
     * Copier la charge utile uniquement si elle existe.
     */
    if (length != 0) {
        memcpy(
            message.data,
            data,
            length
        );
    }

    /*
     * Placer la trame dans la file d'émission.
     * ESP_OK signifie que la file TX a accepté la trame.
     */
    return twai_transmit(
        &message,
        pdMS_TO_TICKS(timeout_ms)
    );
}

esp_err_t can_driver_receive(
    can_frame_t *frame,
    uint32_t timeout_ms
)
{
    /* Refuser un pointeur de destination invalide. */
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t message;

    /*
     * Attendre une trame dans la file de réception.
     */
    esp_err_t err =
        twai_receive(
            &message,
            pdMS_TO_TICKS(timeout_ms)
        );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Le protocole interne utilise uniquement des trames
     * standard de données.
     */
    if (
        message.extd ||
        message.rtr
    ) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /*
     * Vérifier que le DLC reste compatible
     * avec une trame CAN classique de 8 octets.
     */
    if (
        message.data_length_code >
        CAN_CLASSIC_MAX_DATA_LENGTH
    ) {
        return ESP_ERR_INVALID_SIZE;
    }

    frame->identifier =
        message.identifier;

    frame->data_length =
        message.data_length_code;

    memcpy(
        frame->data,
        message.data,
        message.data_length_code
    );

    return ESP_OK;
}

esp_err_t can_driver_get_status(
    twai_status_info_t *status
)
{
    /* Refuser un pointeur de destination invalide. */
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return twai_get_status_info(
        status
    );
}

esp_err_t can_driver_recover(void)
{
    /*
     * Lire l'état courant du contrôleur avant
     * de lancer une récupération.
     */
    twai_status_info_t status;

    esp_err_t err =
        twai_get_status_info(
            &status
        );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * La récupération n'est valide que si le contrôleur
     * est réellement dans l'état bus-off.
     */
    if (
        status.state !=
        TWAI_STATE_BUS_OFF
    ) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Démarrer la séquence asynchrone de récupération CAN.
     *
     * Après l'alerte TWAI_ALERT_BUS_RECOVERED, le contrôleur
     * devra être redémarré avec twai_start().
     */
    return twai_initiate_recovery();
}