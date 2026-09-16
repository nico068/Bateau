// SPDX-License-Identifier: GPL-3.0-only

#pragma once

/* Broches du contrôleur CAN interne SN65HVD230. */
#define PIN_CAN_TX                    17
#define PIN_CAN_RX                    16

/* Vitesse du bus CAN interne entre les ESP32. */
#define CAN_BITRATE                   500000

/* Broches et fréquence du bus I²C maître. */
#define PIN_I2C_SDA                   21
#define PIN_I2C_SCL                   22
#define I2C_FREQUENCY_HZ              400000

/* Adresses I²C des capteurs. */
#define BME280_I2C_ADDRESS            0x77
#define MPU9250_I2C_ADDRESS           0x68
#define AK8963_I2C_ADDRESS            0x0C

/* Joystick. */
#define PIN_JOYSTICK_X                36
#define PIN_JOYSTICK_Y                39
#define PIN_JOYSTICK_BUTTON           13
#define JOYSTICK_DEAD_ZONE            30

/* Encodeur manette gauche. */
#define PIN_THROTTLE_G_A              25
#define PIN_THROTTLE_G_B              26

/* Encodeur manette droite. */
#define PIN_THROTTLE_D_A              32
#define PIN_THROTTLE_D_B              33

/* Encodeur volant. */
#define PIN_WHEEL_A                   34
#define PIN_WHEEL_B                   35

/* Limites des compteurs. */
#define THROTTLE_G_COUNT_MIN          0
#define THROTTLE_G_COUNT_MAX          1000
#define THROTTLE_D_COUNT_MIN          0
#define THROTTLE_D_COUNT_MAX          1000
#define WHEEL_COUNT_MIN               (-1000)
#define WHEEL_COUNT_MAX               1000

/* Périodes des tâches. */
#define CONTROL_PERIOD_MS             10
#define IMU_PERIOD_MS                 20
#define ENVIRONMENT_PERIOD_MS         500
#define ACTUATOR_STATUS_TIMEOUT_MS    20

/* Calibration gyroscope. */
#define GYRO_BIAS_X_DPS               0.0f
#define GYRO_BIAS_Y_DPS               0.0f
#define GYRO_BIAS_Z_DPS               0.0f

/* Calibration magnétomètre. */
#define MAG_OFFSET_X_UT               0.0f
#define MAG_OFFSET_Y_UT               0.0f
#define MAG_OFFSET_Z_UT               0.0f

/* Facteurs d’échelle magnétomètre. */
#define MAG_SCALE_X                   1.0f
#define MAG_SCALE_Y                   1.0f
#define MAG_SCALE_Z                   1.0f