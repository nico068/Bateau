#pragma once

/* Bus CAN interne via SN65HVD230 */
#define PIN_CAN_TX                   17
#define PIN_CAN_RX                   16
#define CAN_BITRATE                  500000
#define CAN_ID_ACTUATOR_CMD          0x210
#define CAN_ID_ACTUATOR_STATUS       0x290
#define CAN_TIMEOUT_MS               200

/* Bus I2C et ADS1115 */
#define PIN_I2C_SDA                  21
#define PIN_I2C_SCL                  22
#define ADS1115_ADDRESS              0x48

#define ADS_CH_TURBINE_G             0
#define ADS_CH_TURBINE_D             1
#define ADS_CH_REVERSE_G             2
#define ADS_CH_REVERSE_D             3

/* Servomoteurs RC */
#define PIN_SERVO_TURBINE_G          32
#define PIN_SERVO_TURBINE_D          25
#define PIN_SERVO_REVERSE_G          33
#define PIN_SERVO_REVERSE_D          26

#define SERVO_FREQUENCY_HZ           50
#define SERVO_PULSE_MIN_US           1000
#define SERVO_PULSE_CENTER_US        1500
#define SERVO_PULSE_MAX_US           2000
#define SERVO_COMMAND_MAX            1000

/* Limites mécaniques */
#define TURBINE_G_ANGLE_MIN_DEG      (-45)
#define TURBINE_G_ANGLE_MAX_DEG      45

#define TURBINE_D_ANGLE_MIN_DEG      (-45)
#define TURBINE_D_ANGLE_MAX_DEG      45

#define REVERSE_G_ANGLE_MIN_DEG      0
#define REVERSE_G_ANGLE_MAX_DEG      90

#define REVERSE_D_ANGLE_MIN_DEG      0
#define REVERSE_D_ANGLE_MAX_DEG      90

/* Calibration ADS1115 provisoire */
#define ADS_RAW_MIN                  0
#define ADS_RAW_MAX                  32767

#define ANGLE_SCALE                  10