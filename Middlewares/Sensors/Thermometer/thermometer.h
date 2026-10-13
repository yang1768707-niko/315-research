#ifndef THERMOMETER_H
#define THERMOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include <stdint.h>

#define THERMOMETER_ALERT_EVENT  0x01U
#define THERMOMETER_ALERT_THERM  0x02U

HAL_StatusTypeDef Thermometer_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Thermometer_EnterLowPower(void);
HAL_StatusTypeDef Thermometer_ExitLowPower(void);
int16_t Thermometer_GetNextCentiC(void);
uint16_t Thermometer_AppendPayload(uint8_t *buffer, uint16_t buffer_size);
uint16_t Thermometer_SaveState(void);
void Thermometer_RestoreState(uint16_t state);
void Thermometer_OnEventAlert(void);
void Thermometer_OnThermAlert(void);
uint8_t Thermometer_GetAlertFlags(void);
uint8_t Thermometer_IsReady(void);
uint8_t Thermometer_GetProductId(void);
uint8_t Thermometer_GetManufacturerId(void);

#ifdef __cplusplus
}
#endif

#endif /* THERMOMETER_H */
