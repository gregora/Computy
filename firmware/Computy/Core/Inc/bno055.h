/**
  * @file    bno055.h
  * @brief   Driver for BNO055 IMU sensor (I2C interface)
  * @author  Your Name
  */

#ifndef BNO055_H
#define BNO055_H

#include "stm32f2xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* BNO055 I2C Address */
#define BNO055_I2C_ADDR1 (0x28 << 1)  // Default when COM3 pin is low
#define BNO055_I2C_ADDR2 (0x29 << 1)  // When COM3 pin is high

/* BNO055 Register Map */
#define BNO055_PAGE_ID_ADDR             0x07
#define BNO055_CHIP_ID_ADDR             0x00
#define BNO055_OPR_MODE_ADDR            0x3D
#define BNO055_PWR_MODE_ADDR            0x3E
#define BNO055_SYS_TRIGGER_ADDR         0x3F
#define BNO055_AXIS_MAP_CONFIG_ADDR     0x41
#define BNO055_AXIS_MAP_SIGN_ADDR       0x42
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14

/* Operation modes */
#define BNO055_OPERATION_MODE_CONFIG    0x00
#define BNO055_OPERATION_MODE_NDOF      0x0C

/* Power modes */
#define BNO055_POWER_MODE_NORMAL        0x00

/* BNO055 ID */
#define BNO055_ID                       0xA0

/* Quaternion structure */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} BNO055_Quaternion_t;

/* Linear acceleration structure */
typedef struct {
    float x;
    float y;
    float z;
} BNO055_LinearAccel_t;

/* Euler angles structure */
typedef struct {
    float pitch;    // Rotation around Y axis (radians)
    float roll;     // Rotation around X axis (radians)
    float yaw;      // Rotation around Z axis (radians)
} BNO055_Euler_t;

/* Angular velocity structure */
typedef struct {
    float x;  // Angular velocity around X axis (rad/s)
    float y;  // Angular velocity around Y axis (rad/s)
    float z;  // Angular velocity around Z axis (rad/s)
} BNO055_AngularVelocity_t;

/* BNO055 handle structure */
typedef struct {
    I2C_HandleTypeDef *i2cHandle;
    uint8_t i2cAddr;
    bool initialized;
} BNO055_Handle_t;

/* Function prototypes */
HAL_StatusTypeDef BNO055_Init(BNO055_Handle_t *hdev, I2C_HandleTypeDef *i2cHandle, uint8_t i2cAddr);
HAL_StatusTypeDef BNO055_ReadQuaternion(BNO055_Handle_t *hdev, BNO055_Quaternion_t *quat);
HAL_StatusTypeDef BNO055_ReadLinearAccel(BNO055_Handle_t *hdev, BNO055_LinearAccel_t *accel);
HAL_StatusTypeDef BNO055_Reset(BNO055_Handle_t *hdev);
HAL_StatusTypeDef BNO055_SetOperationMode(BNO055_Handle_t *hdev, uint8_t mode);
void BNO055_QuaternionToEuler(const BNO055_Quaternion_t *quat, BNO055_Euler_t *euler);
HAL_StatusTypeDef BNO055_ReadAngularVelocity(BNO055_Handle_t *hdev, BNO055_AngularVelocity_t *angular_velocity);

#endif /* BNO055_H */
