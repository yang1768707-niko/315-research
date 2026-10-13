#ifndef ANALOG_MICROPHONE_H
#define ANALOG_MICROPHONE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include <stdint.h>

#define ANALOG_MICROPHONE_PAYLOAD_SIZE  1U

HAL_StatusTypeDef AnalogMicrophone_Init(ADC_HandleTypeDef *hadc,
                                        TIM_HandleTypeDef *htim);
HAL_StatusTypeDef AnalogMicrophone_ConfigureWindow(uint16_t window_ms);
HAL_StatusTypeDef AnalogMicrophone_Start(void);
void AnalogMicrophone_Process(void);
void AnalogMicrophone_Stop(void);
uint16_t AnalogMicrophone_AppendPayload(uint8_t *buffer, uint16_t buffer_size);
uint32_t AnalogMicrophone_GetDroppedBlocks(void);
uint8_t AnalogMicrophone_IsRunning(void);
uint8_t AnalogMicrophone_HasValue(void);
uint8_t AnalogMicrophone_GetLastValue(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_MICROPHONE_H */
