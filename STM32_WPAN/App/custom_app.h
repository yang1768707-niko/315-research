/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.h
  * @author  MCD Application Team
  * @brief   Header for custom_app.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_APP_H
#define CUSTOM_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  CUSTOM_CONN_HANDLE_EVT,
  CUSTOM_DISCON_HANDLE_EVT,
} Custom_App_Opcode_Notification_evt_t;

typedef struct
{
  Custom_App_Opcode_Notification_evt_t     Custom_Evt_Opcode;
  uint16_t                                 ConnectionHandle;
} Custom_App_ConnHandle_Not_evt_t;
/* USER CODE BEGIN ET */
typedef struct
{
  int16_t temperature_centi_c;
  int16_t magnetic_0p1_ut[3];
  uint8_t digital_mic_db_spl;
  uint8_t analog_mic_db_spl;
  uint16_t vibration_rate_hz;
  uint32_t vibration_sample_count;
  int32_t vibration_features_milli[3][5];
  uint8_t valid_mask;
  uint8_t vibration_features_ready;
  /* Consume the old two padding bytes; persisted snapshot size stays 80. */
  uint8_t scalar_required_mask;
  uint8_t scalar_policy_version;
} Custom_APP_ZigbeeSnapshot_t;

#define CUSTOM_APP_ZB_VALID_TEMPERATURE  0x01U
#define CUSTOM_APP_ZB_VALID_MAGNETIC     0x02U
#define CUSTOM_APP_ZB_VALID_DIGITAL_MIC  0x04U
#define CUSTOM_APP_ZB_VALID_ANALOG_MIC   0x08U
#define CUSTOM_APP_ZB_VALID_VIBRATION    0x10U

#define CUSTOM_APP_ZB_SENSOR_VIBRATION 0x01U
#define CUSTOM_APP_ZB_SENSOR_DIGITAL_MIC 0x02U
#define CUSTOM_APP_ZB_SENSOR_ANALOG_MIC 0x04U
#define CUSTOM_APP_ZB_SENSOR_ALL 0x07U
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum
{
  CUSTOM_APP_WORK_MODE_ONLINE = 0U,
  CUSTOM_APP_WORK_MODE_INSPECTION = 1U
} Custom_APP_WorkMode_t;

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void Custom_APP_Init(void);
void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification);
/* USER CODE BEGIN EF */
/* BLE policy only; these accessors do not change Zigbee or STOP2 policy. */
Custom_APP_WorkMode_t Custom_APP_GetWorkMode(void);
Custom_APP_WorkMode_t Custom_APP_GetConfiguredWorkMode(void);
void Custom_APP_TxPoolAvailable(void);
void Custom_APP_Process(void);
void Custom_APP_ProtocolMtu(uint16_t handle, uint16_t mtu);
/* Returns 0 while Flash preparation is busy/failed; caller must retry before STOP2. */
uint8_t Custom_APP_PrepareStop2(void);
void Custom_APP_ResumeFromStop2(void);
uint8_t Custom_APP_ZigbeeBeginSnapshot(uint16_t vibration_rate_hz,
  uint16_t vibration_window_ms, uint16_t digital_mic_window_ms,
  uint16_t analog_mic_window_ms, uint8_t sensor_enable_mask, uint32_t batch_id);
uint8_t Custom_APP_ZigbeeSnapshotReady(void);
void Custom_APP_ZigbeeFinishSnapshot(Custom_APP_ZigbeeSnapshot_t *snapshot);
void Custom_APP_ZigbeeActivityChanged(void);
void Custom_APP_ZigbeeAbortSnapshot(void);

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_APP_H */
