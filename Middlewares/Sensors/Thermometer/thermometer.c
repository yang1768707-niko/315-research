#include "thermometer.h"

#define THERMOMETER_PAYLOAD_SIZE          2U
#define STTS751_I2C_ADDRESS                (0x4AU << 1U)
#define STTS751_REG_TEMPERATURE_HIGH       0x00U
#define STTS751_REG_TEMPERATURE_LOW        0x02U
#define STTS751_REG_CONFIGURATION          0x03U
#define STTS751_REG_CONVERSION_RATE        0x04U
#define STTS751_REG_PRODUCT_ID             0xFDU
#define STTS751_REG_MANUFACTURER_ID        0xFEU
#define STTS751_PRODUCT_ID                 0x01U
#define STTS751_MANUFACTURER_ID            0x53U
#define STTS751_CONFIGURATION_12_BIT       0x0CU
#define STTS751_CONFIGURATION_STANDBY      0x40U
#define STTS751_CONVERSION_RATE_1_HZ       0x04U
#define STTS751_I2C_TRIALS                  3U
#define STTS751_I2C_TIMEOUT_MS              100U

static I2C_HandleTypeDef *s_i2c;
static int16_t s_last_centi_c;
static uint8_t s_ready;
static uint8_t s_product_id;
static uint8_t s_manufacturer_id;
static volatile uint8_t s_alert_flags;
static uint8_t s_low_power;

static HAL_StatusTypeDef Thermometer_ReadRegister(uint8_t reg, uint8_t *value)
{
  if ((s_i2c == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(s_i2c,
                          STTS751_I2C_ADDRESS,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          value,
                          1U,
                          STTS751_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Thermometer_WriteRegister(uint8_t reg, uint8_t value)
{
  if (s_i2c == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Write(s_i2c,
                           STTS751_I2C_ADDRESS,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1U,
                           STTS751_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Thermometer_ReadRaw(int16_t *raw_temperature)
{
  uint8_t high_byte;
  uint8_t low_byte;
  uint16_t raw_12_bit;

  if (raw_temperature == NULL)
  {
    return HAL_ERROR;
  }

  if (Thermometer_ReadRegister(STTS751_REG_TEMPERATURE_HIGH, &high_byte) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Thermometer_ReadRegister(STTS751_REG_TEMPERATURE_LOW, &low_byte) != HAL_OK)
  {
    return HAL_ERROR;
  }

  raw_12_bit = (uint16_t)((((uint16_t)high_byte << 8U) | low_byte) >> 4U);
  if ((raw_12_bit & 0x0800U) != 0U)
  {
    raw_12_bit |= 0xF000U;
  }

  *raw_temperature = (int16_t)raw_12_bit;
  return HAL_OK;
}

HAL_StatusTypeDef Thermometer_Init(I2C_HandleTypeDef *hi2c)
{
  s_i2c = hi2c;
  s_ready = 0U;
  s_product_id = 0U;
  s_manufacturer_id = 0U;
  s_low_power = 0U;

  if (s_i2c == NULL)
  {
    return HAL_ERROR;
  }
  if (HAL_I2C_IsDeviceReady(s_i2c,
                            STTS751_I2C_ADDRESS,
                            STTS751_I2C_TRIALS,
                            STTS751_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Thermometer_ReadRegister(STTS751_REG_PRODUCT_ID, &s_product_id) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Thermometer_ReadRegister(STTS751_REG_MANUFACTURER_ID, &s_manufacturer_id) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if ((s_product_id != STTS751_PRODUCT_ID) ||
      (s_manufacturer_id != STTS751_MANUFACTURER_ID))
  {
    return HAL_ERROR;
  }
  if (Thermometer_WriteRegister(STTS751_REG_CONFIGURATION,
                                STTS751_CONFIGURATION_12_BIT) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Thermometer_WriteRegister(STTS751_REG_CONVERSION_RATE,
                                STTS751_CONVERSION_RATE_1_HZ) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_ready = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Thermometer_EnterLowPower(void)
{
  HAL_StatusTypeDef status;

  if (s_ready == 0U)
  {
    return HAL_ERROR;
  }
  if (s_low_power != 0U)
  {
    return HAL_OK;
  }

  status = Thermometer_WriteRegister(STTS751_REG_CONFIGURATION,
                                     STTS751_CONFIGURATION_12_BIT |
                                     STTS751_CONFIGURATION_STANDBY);
  if (status == HAL_OK)
  {
    s_low_power = 1U;
  }
  return status;
}

HAL_StatusTypeDef Thermometer_ExitLowPower(void)
{
  HAL_StatusTypeDef status;

  if (s_ready == 0U)
  {
    return HAL_ERROR;
  }
  if (s_low_power == 0U)
  {
    return HAL_OK;
  }

  status = Thermometer_WriteRegister(STTS751_REG_CONFIGURATION,
                                     STTS751_CONFIGURATION_12_BIT);
  if (status == HAL_OK)
  {
    s_low_power = 0U;
  }
  return status;
}

int16_t Thermometer_GetNextCentiC(void)
{
  int16_t raw_temperature;
  int32_t scaled;

  if ((s_ready == 0U) && (Thermometer_Init(s_i2c) != HAL_OK))
  {
    return s_last_centi_c;
  }
  if (Thermometer_ExitLowPower() != HAL_OK)
  {
    return s_last_centi_c;
  }
  if (Thermometer_ReadRaw(&raw_temperature) != HAL_OK)
  {
    s_ready = 0U;
    return s_last_centi_c;
  }

  scaled = (int32_t)raw_temperature * 100L;
  scaled += (scaled >= 0L) ? 8L : -8L;
  s_last_centi_c = (int16_t)(scaled / 16L);
  return s_last_centi_c;
}

uint16_t Thermometer_AppendPayload(uint8_t *buffer, uint16_t buffer_size)
{
  int16_t value;

  if ((buffer == NULL) || (buffer_size < THERMOMETER_PAYLOAD_SIZE))
  {
    return 0U;
  }
  if ((s_ready == 0U) && (Thermometer_Init(s_i2c) != HAL_OK))
  {
    return 0U;
  }

  value = Thermometer_GetNextCentiC();
  if (s_ready == 0U)
  {
    return 0U;
  }

  buffer[0] = (uint8_t)((uint16_t)value & 0xFFU);
  buffer[1] = (uint8_t)((uint16_t)value >> 8U);
  s_alert_flags = 0U;
  return THERMOMETER_PAYLOAD_SIZE;
}

uint16_t Thermometer_SaveState(void)
{
  return (uint16_t)s_alert_flags;
}

void Thermometer_RestoreState(uint16_t state)
{
  s_alert_flags |= (uint8_t)state;
}

void Thermometer_OnEventAlert(void)
{
  s_alert_flags |= THERMOMETER_ALERT_EVENT;
}

void Thermometer_OnThermAlert(void)
{
  s_alert_flags |= THERMOMETER_ALERT_THERM;
}

uint8_t Thermometer_GetAlertFlags(void)
{
  return s_alert_flags;
}

uint8_t Thermometer_IsReady(void)
{
  return s_ready;
}

uint8_t Thermometer_GetProductId(void)
{
  return s_product_id;
}

uint8_t Thermometer_GetManufacturerId(void)
{
  return s_manufacturer_id;
}
