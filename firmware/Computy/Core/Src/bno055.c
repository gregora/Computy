/**
  * @file    bno055.c
  * @brief   Driver for BNO055 IMU sensor (I2C interface)
  * @author  Your Name
  */

#include "bno055.h"
#include <math.h>
#include <string.h>

/* Private function prototypes */
static HAL_StatusTypeDef BNO055_ReadRegister(BNO055_Handle_t *hdev, uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef BNO055_WriteRegister(BNO055_Handle_t *hdev, uint8_t reg, uint8_t data);
static float BNO055_QuaternionConversion(uint16_t rawValue);

/**
  * @brief  Initialize the BNO055 sensor
  * @param  hdev: Pointer to BNO055 handle structure
  * @param  i2cHandle: Pointer to I2C handle
  * @param  i2cAddr: I2C address of BNO055
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_Init(BNO055_Handle_t *hdev, I2C_HandleTypeDef *i2cHandle, uint8_t i2cAddr)
{
    uint8_t chip_id = 0;
    
    if(hdev == NULL || i2cHandle == NULL)
        return HAL_ERROR;
    
    /* Initialize handle */
    hdev->i2cHandle = i2cHandle;
    hdev->i2cAddr = i2cAddr;
    hdev->initialized = false;
    
    /* Check device ID */
    if(BNO055_ReadRegister(hdev, BNO055_CHIP_ID_ADDR, &chip_id, 1) != HAL_OK)
        return HAL_ERROR;
    
    if(chip_id != BNO055_ID)
        return HAL_ERROR;
    
    /* Reset the device */
    if(BNO055_Reset(hdev) != HAL_OK)
        return HAL_ERROR;
    
    HAL_Delay(650); // Wait for reset to complete
    
    /* Set power mode to normal */
    if(BNO055_WriteRegister(hdev, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL) != HAL_OK)
        return HAL_ERROR;


    /* First set to CONFIG mode for axis remap */
    if(BNO055_SetOperationMode(hdev, BNO055_OPERATION_MODE_CONFIG) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(20); // Wait for mode change



    /* Set axis remap */
    /* DEFAULT */
    if(BNO055_WriteRegister(hdev, BNO055_AXIS_MAP_CONFIG_ADDR, 0x24) != HAL_OK)
        return HAL_ERROR;
    /* Set axis map sign */
    if(BNO055_WriteRegister(hdev, BNO055_AXIS_MAP_SIGN_ADDR, 0x00) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(20); // Wait for mode change


    /* Set operation mode to NDOF (sensor fusion) */
    if(BNO055_SetOperationMode(hdev, BNO055_OPERATION_MODE_NDOF) != HAL_OK)
        return HAL_ERROR;


    HAL_Delay(20); // Wait for mode change


    hdev->initialized = true;
    
    return HAL_OK;
}

/**
  * @brief  Read quaternion data from BNO055
  * @param  hdev: Pointer to BNO055 handle structure
  * @param  quat: Pointer to quaternion structure to store data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadQuaternion(BNO055_Handle_t *hdev, BNO055_Quaternion_t *quat)
{
    uint8_t data[8];
    
    if(hdev == NULL || quat == NULL || !hdev->initialized)
        return HAL_ERROR;
    
    /* Read 8 bytes of quaternion data (w, x, y, z) */
    if(BNO055_ReadRegister(hdev, BNO055_QUATERNION_DATA_W_LSB_ADDR, data, 8) != HAL_OK)
        return HAL_ERROR;
    
    /* Convert raw data to quaternion values */
    quat->w = BNO055_QuaternionConversion((uint16_t)((data[1] << 8) | data[0]));
    quat->x = BNO055_QuaternionConversion((uint16_t)((data[3] << 8) | data[2]));
    quat->y = BNO055_QuaternionConversion((uint16_t)((data[5] << 8) | data[4]));
    quat->z = BNO055_QuaternionConversion((uint16_t)((data[7] << 8) | data[6]));
    
    return HAL_OK;
}

/**
  * @brief  Read linear acceleration data from BNO055
  * @param  hdev: Pointer to BNO055 handle structure
  * @param  accel: Pointer to linear acceleration structure to store data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadLinearAccel(BNO055_Handle_t *hdev, BNO055_LinearAccel_t *accel)
{
    uint8_t data[6];
    int16_t raw_x, raw_y, raw_z;
    
    if(hdev == NULL || accel == NULL || !hdev->initialized)
        return HAL_ERROR;
    
    /* Read 6 bytes of linear acceleration data (x, y, z) */
    if(BNO055_ReadRegister(hdev, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, data, 6) != HAL_OK)
        return HAL_ERROR;
    
    /* Convert raw data to m/s^2 (1 m/s^2 = 100 LSB) */
    raw_x = (int16_t)((data[1] << 8) | data[0]);
    raw_y = (int16_t)((data[3] << 8) | data[2]);
    raw_z = (int16_t)((data[5] << 8) | data[4]);
    
    accel->x = (float)raw_x / 100.0f;
    accel->y = (float)raw_y / 100.0f;
    accel->z = (float)raw_z / 100.0f;
    
    return HAL_OK;
}

/**
  * @brief  Reset the BNO055 sensor
  * @param  hdev: Pointer to BNO055 handle structure
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_Reset(BNO055_Handle_t *hdev)
{
    if(hdev == NULL)
        return HAL_ERROR;
    
    /* Write reset bit (0x20) to SYS_TRIGGER register */
    return BNO055_WriteRegister(hdev, BNO055_SYS_TRIGGER_ADDR, 0x20);
}

/**
  * @brief  Set operation mode of BNO055
  * @param  hdev: Pointer to BNO055 handle structure
  * @param  mode: Operation mode to set
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_SetOperationMode(BNO055_Handle_t *hdev, uint8_t mode)
{
    if(hdev == NULL)
        return HAL_ERROR;
    
    return BNO055_WriteRegister(hdev, BNO055_OPR_MODE_ADDR, mode);
}

/**
  * @brief  Read from BNO055 register
  * @param  hdev: Pointer to BNO055 handle structure
  * @param  reg: Register address to read from
  * @param  data: Pointer to store read data
  * @param  len: Number of bytes to read
  * @retval HAL status
  */
static HAL_StatusTypeDef BNO055_ReadRegister(BNO055_Handle_t *hdev, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(hdev->i2cHandle, hdev->i2cAddr, reg, 
                           I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Write to BNO055 register
  * @param  hdev: Pointer to BNO055 handle structure
  * @param  reg: Register address to write to
  * @param  data: Data to write
  * @retval HAL status
  */
static HAL_StatusTypeDef BNO055_WriteRegister(BNO055_Handle_t *hdev, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hdev->i2cHandle, hdev->i2cAddr, reg, 
                            I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Convert raw quaternion value to unit quaternion
  * @param  rawValue: Raw 16-bit quaternion value
  * @retval Converted quaternion value (1.0 = 16384 LSB)
  */
static float BNO055_QuaternionConversion(uint16_t rawValue)
{
    return (float)((int16_t)rawValue) / 16384.0f;
}



/**
  * @brief  Convert quaternion to Euler angles (pitch, roll, yaw)
  * @param  quat: Pointer to quaternion structure
  * @param  euler: Pointer to Euler angles structure to store result
  * @note   Euler angles are in radians
  * @note   Rotation order: ZYX (yaw, pitch, roll)
  */
void BNO055_QuaternionToEuler(const BNO055_Quaternion_t *quat, BNO055_Euler_t *euler)
{
    if(quat == NULL || euler == NULL)
        return;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (quat->w * quat->x + quat->y * quat->z);
    float cosr_cosp = 1.0f - 2.0f * (quat->x * quat->x + quat->y * quat->y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (quat->w * quat->y - quat->z * quat->x);
    if(fabsf(sinp) >= 1.0f)
        euler->pitch = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    else
        euler->pitch = asinf(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (quat->w * quat->z + quat->x * quat->y);
    float cosy_cosp = 1.0f - 2.0f * (quat->y * quat->y + quat->z * quat->z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}


/**
  * @brief  Read angular velocity (gyroscope) data from BNO055
  * @param  hdev: Pointer to BNO055 handle structure
  * @param  angular_velocity: Pointer to angular velocity structure to store data
  * @retval HAL status
  * @note   Angular velocity is in radians per second
  */
HAL_StatusTypeDef BNO055_ReadAngularVelocity(BNO055_Handle_t *hdev, BNO055_AngularVelocity_t *angular_velocity)
{
    uint8_t data[6];
    int16_t raw_x, raw_y, raw_z;

    if(hdev == NULL || angular_velocity == NULL || !hdev->initialized)
        return HAL_ERROR;

    /* Read 6 bytes of angular velocity data (x, y, z) */
    if(BNO055_ReadRegister(hdev, BNO055_GYRO_DATA_X_LSB_ADDR, data, 6) != HAL_OK)
        return HAL_ERROR;

    /* Convert raw data to rad/s (1 rad/s = 16 LSB) */
    raw_x = (int16_t)((data[1] << 8) | data[0]);
    raw_y = (int16_t)((data[3] << 8) | data[2]);
    raw_z = (int16_t)((data[5] << 8) | data[4]);

    angular_velocity->x = (float)raw_x / 16.0f;
    angular_velocity->y = (float)raw_y / 16.0f;
    angular_velocity->z = (float)raw_z / 16.0f;

    return HAL_OK;
}
