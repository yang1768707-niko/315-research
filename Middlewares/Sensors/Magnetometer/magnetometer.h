#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include <stdint.h>

#define MAGNETOMETER_PAYLOAD_SIZE  6U

typedef struct
{
  int16_t x_0p1_ut;
  int16_t y_0p1_ut;
  int16_t z_0p1_ut;
} Magnetometer_Data_t;

HAL_StatusTypeDef Magnetometer_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Magnetometer_EnterLowPower(void);
HAL_StatusTypeDef Magnetometer_ExitLowPower(void);
uint8_t Magnetometer_GetWhoAmI(void);
uint8_t Magnetometer_IsOnline(void);
uint16_t Magnetometer_AppendPayload(uint8_t *buffer, uint16_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* MAGNETOMETER_H */
