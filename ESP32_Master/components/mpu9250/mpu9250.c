#include "mpu9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stddef.h>

/* Registres principaux du MPU9250. */
#define MPU9250_REG_ACCEL_XOUT_H      0x3B
#define MPU9250_REG_CONFIG            0x1A
#define MPU9250_REG_GYRO_CONFIG       0x1B
#define MPU9250_REG_ACCEL_CONFIG      0x1C
#define MPU9250_REG_INT_PIN_CFG       0x37
#define MPU9250_REG_PWR_MGMT_1        0x6B
#define MPU9250_REG_WHO_AM_I          0x75

/* Registres du magnétomètre AK8963 intégré au module. */
#define AK8963_REG_STATUS_1           0x02
#define AK8963_REG_DATA_START         0x03
#define AK8963_REG_CONTROL_1          0x0A
#define AK8963_REG_SENSITIVITY_START  0x10

/* Identifiants acceptés : MPU9250 et MPU9255. */
#define MPU9250_WHO_AM_I_VALUE        0x71
#define MPU9255_WHO_AM_I_VALUE        0x73

/* Facteurs correspondant aux plages ±2 g, ±250 °/s et 16 bits AK8963. */
#define ACCEL_LSB_PER_G               16384.0f
#define GYRO_LSB_PER_DPS              131.0f
#define MAG_UT_PER_LSB                0.15f

/* Adresses I2C mémorisées lors de l'initialisation du composant. */
static i2c_master_dev_handle_t mpu_device;
static i2c_master_dev_handle_t mag_device;

/* Correction de sensibilité propre à chaque axe, lue dans l'AK8963. */
static float mag_adjust[3] = {1.0f, 1.0f, 1.0f};

/* Écrit un octet dans un registre d'un périphérique du bus I2C numéro 0. */
static esp_err_t write_reg(
    i2c_master_dev_handle_t device,
    uint8_t reg,
    uint8_t value
)
{
    const uint8_t data[2] = { reg, value };
    return i2c_master_transmit(device, data, sizeof(data), 20);
}

/* Sélectionne un registre puis lit une série continue d'octets. */
static esp_err_t read_regs(
    i2c_master_dev_handle_t device,
    uint8_t reg,
    uint8_t *data,
    size_t length
)
{
    return i2c_master_transmit_receive(device, &reg, 1, data, length, 20);
}

/* Convertit deux octets big-endian du MPU9250 en entier signé. */
static int16_t s16be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/* Convertit deux octets little-endian de l'AK8963 en entier signé. */
static int16_t s16le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

esp_err_t mpu9250_init(
    i2c_master_bus_handle_t bus_handle,
    uint8_t mpu_address,
    uint8_t magnetometer_address,
    uint32_t frequency_hz
)
{
    if (bus_handle == NULL || frequency_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Ajouter les deux périphériques au bus I2C moderne. */
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = frequency_hz,
    };
    i2c_device_config_t mpu_config = device_config;
    i2c_device_config_t mag_config = device_config;
    mpu_config.device_address = mpu_address;
    mag_config.device_address = magnetometer_address;

    esp_err_t err = i2c_master_bus_add_device(
        bus_handle,
        &mpu_config,
        &mpu_device
    );
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_bus_add_device(
        bus_handle,
        &mag_config,
        &mag_device
    );
    if (err != ESP_OK) {
        return err;
    }

    /* Vérifier que le périphérique répond avec un identifiant compatible. */
    uint8_t id = 0;
    err = read_regs(mpu_device, MPU9250_REG_WHO_AM_I, &id, 1);
    if (err != ESP_OK) {
        return err;
    }
    if (id != MPU9250_WHO_AM_I_VALUE && id != MPU9255_WHO_AM_I_VALUE) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Sortir le MPU9250 du mode sommeil, puis attendre la stabilisation. */
    err = write_reg(mpu_device, MPU9250_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * CONFIG = 0x03       : filtre numérique interne.
     * GYRO_CONFIG = 0x00  : plage ±250 degrés par seconde.
     * ACCEL_CONFIG = 0x00 : plage ±2 g.
     * INT_PIN_CFG = 0x02  : bypass I2C pour accéder directement à l'AK8963.
     */
    const uint8_t registers[][2] = {
        {MPU9250_REG_CONFIG, 0x03},
        {MPU9250_REG_GYRO_CONFIG, 0x00},
        {MPU9250_REG_ACCEL_CONFIG, 0x00},
        {MPU9250_REG_INT_PIN_CFG, 0x02}
    };
    for (unsigned i = 0; i < 4; i++) {
        err = write_reg(mpu_device, registers[i][0], registers[i][1]);
        if (err != ESP_OK) {
            return err;
        }
    }

    /* Mettre l'AK8963 hors tension avant chaque changement de mode. */
    err = write_reg(mag_device, AK8963_REG_CONTROL_1, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    /* Mode fuse-ROM : donne accès aux coefficients de sensibilité usine. */
    err = write_reg(mag_device, AK8963_REG_CONTROL_1, 0x0F);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t asa[3];
    err = read_regs(mag_device, AK8963_REG_SENSITIVITY_START, asa, sizeof(asa));
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 3; i++) {
        mag_adjust[i] = ((asa[i] - 128.0f) / 256.0f) + 1.0f;
    }

    err = write_reg(mag_device, AK8963_REG_CONTROL_1, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    /* Mode continu 2, sortie 16 bits, fréquence de mesure de 100 Hz. */
    return write_reg(mag_device, AK8963_REG_CONTROL_1, 0x16);
}

esp_err_t mpu9250_read(mpu9250_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * Bloc de 14 octets : accélération XYZ, température interne,
     * puis gyroscope XYZ. La température est volontairement ignorée ici.
     */
    uint8_t data[14];
    esp_err_t err = read_regs(mpu_device, MPU9250_REG_ACCEL_XOUT_H, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }
    /* Convertir les mesures brutes dans les unités physiques. */
    out->ax_g = s16be(data) / ACCEL_LSB_PER_G;
    out->ay_g = s16be(data + 2) / ACCEL_LSB_PER_G;
    out->az_g = s16be(data + 4) / ACCEL_LSB_PER_G;
    out->gx_dps = s16be(data + 8) / GYRO_LSB_PER_DPS;
    out->gy_dps = s16be(data + 10) / GYRO_LSB_PER_DPS;
    out->gz_dps = s16be(data + 12) / GYRO_LSB_PER_DPS;

    /* Une mesure magnétique n'est valide que si DRDY=1 et HOFL=0. */
    out->magnetic_valid = false;

    uint8_t status = 0;
    if (read_regs(mag_device, AK8963_REG_STATUS_1, &status, 1) == ESP_OK && (status & 1U) != 0) {
        uint8_t magnetic[7];
        if (read_regs(mag_device, AK8963_REG_DATA_START, magnetic, sizeof(magnetic)) == ESP_OK &&
            (magnetic[6] & 0x08U) == 0) {
            out->mx_ut = s16le(magnetic) * MAG_UT_PER_LSB * mag_adjust[0];
            out->my_ut = s16le(magnetic + 2) * MAG_UT_PER_LSB * mag_adjust[1];
            out->mz_ut = s16le(magnetic + 4) * MAG_UT_PER_LSB * mag_adjust[2];
            out->magnetic_valid = true;
        }
    }
    return ESP_OK;
}