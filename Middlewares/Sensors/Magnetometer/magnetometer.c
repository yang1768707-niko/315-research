#include "magnetometer.h"

#include <stddef.h>

#define IIS2MDC_I2C_ADDRESS_HAL        (0x1EU << 1U)
#define IIS2MDC_REG_WHO_AM_I           0x4FU
#define IIS2MDC_REG_CFG_A              0x60U
#define IIS2MDC_REG_CFG_B              0x61U
#define IIS2MDC_REG_CFG_C              0x62U
#define IIS2MDC_REG_STATUS             0x67U
#define IIS2MDC_REG_OUT_X_L            0x68U
#define IIS2MDC_REG_AUTO_INCREMENT     0x80U
#define IIS2MDC_WHO_AM_I_VALUE         0x40U
#define IIS2MDC_CFG_A_SOFT_RESET       0x20U
#define IIS2MDC_CFG_A_RUN              0x8CU
#define IIS2MDC_CFG_A_MODE_MASK        0x03U
#define IIS2MDC_CFG_A_MODE_IDLE        0x03U
#define IIS2MDC_CFG_A_IDLE             \
  ((IIS2MDC_CFG_A_RUN & (uint8_t)~IIS2MDC_CFG_A_MODE_MASK) | \
   IIS2MDC_CFG_A_MODE_IDLE)
#define IIS2MDC_CFG_B_RUN              0x01U
#define IIS2MDC_CFG_C_RUN              0x10U
#define IIS2MDC_STATUS_XYZ_READY       0x08U
#define IIS2MDC_I2C_TIMEOUT_MS         20U
#define IIS2MDC_RESET_DELAY_MS         10U
#define IIS2MDC_STARTUP_DELAY_MS       20U

static I2C_HandleTypeDef *s_i2c;
static Magnetometer_Data_t s_last_data;
static uint8_t s_last_data_valid;
static uint8_t s_online;
static uint8_t s_who_am_i;
static uint8_t s_low_power;

static HAL_StatusTypeDef Magnetometer_ReadRegister(uint8_t reg, uint8_t *value)
{
  if ((s_i2c == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(s_i2c, IIS2MDC_I2C_ADDRESS_HAL, reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1U,
                          IIS2MDC_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Magnetometer_WriteRegister(uint8_t reg, uint8_t value)
{
  if (s_i2c == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Write(s_i2c, IIS2MDC_I2C_ADDRESS_HAL, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           IIS2MDC_I2C_TIMEOUT_MS);
}

static int16_t Magnetometer_RawTo0p1uT(int16_t raw)
{
  int32_t scaled = (int32_t)raw * 3;

  /* IIS2MDC sensitivity is 0.15 uT/LSB, or 1.5 units of 0.1 uT. */
  if (scaled >= 0)
  {
    scaled = (scaled + 1) / 2;
  }
  else
  {
    scaled = (scaled - 1) / 2;
  }

  if (scaled > INT16_MAX)
  {
    scaled = INT16_MAX;
  }
  else if (scaled < INT16_MIN)
  {
    scaled = INT16_MIN;
  }

  return (int16_t)scaled;
}

static HAL_StatusTypeDef Magnetometer_ReadSample(Magnetometer_Data_t *data)
{
  uint8_t raw_bytes[6];
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;

  if ((s_i2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  if (HAL_I2C_Mem_Read(s_i2c, IIS2MDC_I2C_ADDRESS_HAL,
                       (uint16_t)(IIS2MDC_REG_OUT_X_L | IIS2MDC_REG_AUTO_INCREMENT),
                       I2C_MEMADD_SIZE_8BIT, raw_bytes, sizeof(raw_bytes),
                       IIS2MDC_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  raw_x = (int16_t)(((uint16_t)raw_bytes[1] << 8U) | raw_bytes[0]);
  raw_y = (int16_t)(((uint16_t)raw_bytes[3] << 8U) | raw_bytes[2]);
  raw_z = (int16_t)(((uint16_t)raw_bytes[5] << 8U) | raw_bytes[4]);
  data->x_0p1_ut = Magnetometer_RawTo0p1uT(raw_x);
  data->y_0p1_ut = Magnetometer_RawTo0p1uT(raw_y);
  data->z_0p1_ut = Magnetometer_RawTo0p1uT(raw_z);
  return HAL_OK;
}

HAL_StatusTypeDef Magnetometer_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;

  s_i2c = hi2c;
  s_online = 0U;
  s_last_data_valid = 0U;
  s_who_am_i = 0xFFU;
  s_low_power = 0U;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  status = HAL_I2C_IsDeviceReady(hi2c, IIS2MDC_I2C_ADDRESS_HAL, 3U,
                                 IIS2MDC_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Magnetometer_ReadRegister(IIS2MDC_REG_WHO_AM_I, &s_who_am_i);
  if ((status != HAL_OK) || (s_who_am_i != IIS2MDC_WHO_AM_I_VALUE))
  {
    return HAL_ERROR;
  }

  status = Magnetometer_WriteRegister(IIS2MDC_REG_CFG_A, IIS2MDC_CFG_A_SOFT_RESET);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(IIS2MDC_RESET_DELAY_MS);

  status = Magnetometer_WriteRegister(IIS2MDC_REG_CFG_B, IIS2MDC_CFG_B_RUN);
  if (status == HAL_OK)
  {
    status = Magnetometer_WriteRegister(IIS2MDC_REG_CFG_C, IIS2MDC_CFG_C_RUN);
  }
  if (status == HAL_OK)
  {
    status = Magnetometer_WriteRegister(IIS2MDC_REG_CFG_A, IIS2MDC_CFG_A_RUN);
  }
  if (status == HAL_OK)
  {
    HAL_Delay(IIS2MDC_STARTUP_DELAY_MS);
    s_online = 1U;
  }

  return status;
}

HAL_StatusTypeDef Magnetometer_EnterLowPower(void)
{
  HAL_StatusTypeDef status;

  if (s_online == 0U)
  {
    return HAL_ERROR;
  }
  if (s_low_power != 0U)
  {
    return HAL_OK;
  }

  status = Magnetometer_WriteRegister(IIS2MDC_REG_CFG_A,
                                      IIS2MDC_CFG_A_IDLE);
  if (status == HAL_OK)
  {
    s_low_power = 1U;
  }
  return status;
}

HAL_StatusTypeDef Magnetometer_ExitLowPower(void)
{
  HAL_StatusTypeDef status;

  if (s_online == 0U)
  {
    return HAL_ERROR;
  }
  if (s_low_power == 0U)
  {
    return HAL_OK;
  }

  status = Magnetometer_WriteRegister(IIS2MDC_REG_CFG_A,
                                      IIS2MDC_CFG_A_RUN);
  if (status == HAL_OK)
  {
    s_low_power = 0U;
    HAL_Delay(IIS2MDC_STARTUP_DELAY_MS);
  }
  return status;
}

uint8_t Magnetometer_GetWhoAmI(void)
{
  return s_who_am_i;
}

uint8_t Magnetometer_IsOnline(void)
{
  return s_online;
}

uint16_t Magnetometer_AppendPayload(uint8_t *buffer, uint16_t buffer_size)
{
  uint8_t status_register;
  Magnetometer_Data_t data;

  if ((buffer == NULL) || (buffer_size < MAGNETOMETER_PAYLOAD_SIZE) ||
      (s_online == 0U))
  {
    return 0U;
  }
  if (Magnetometer_ExitLowPower() != HAL_OK)
  {
    return 0U;
  }

  if (Magnetometer_ReadRegister(IIS2MDC_REG_STATUS, &status_register) != HAL_OK)
  {
    s_online = 0U;
    return 0U;
  }

  if ((status_register & IIS2MDC_STATUS_XYZ_READY) != 0U)
  {
    if (Magnetometer_ReadSample(&data) != HAL_OK)
    {
      s_online = 0U;
      return 0U;
    }
    s_last_data = data;
    s_last_data_valid = 1U;
  }

  if (s_last_data_valid == 0U)
  {
    return 0U;
  }

  buffer[0] = (uint8_t)((uint16_t)s_last_data.x_0p1_ut & 0xFFU);
  buffer[1] = (uint8_t)((uint16_t)s_last_data.x_0p1_ut >> 8U);
  buffer[2] = (uint8_t)((uint16_t)s_last_data.y_0p1_ut & 0xFFU);
  buffer[3] = (uint8_t)((uint16_t)s_last_data.y_0p1_ut >> 8U);
  buffer[4] = (uint8_t)((uint16_t)s_last_data.z_0p1_ut & 0xFFU);
  buffer[5] = (uint8_t)((uint16_t)s_last_data.z_0p1_ut >> 8U);
  return MAGNETOMETER_PAYLOAD_SIZE;
}
