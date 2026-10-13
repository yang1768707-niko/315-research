/* Bound Debug-build overhead in FIFO transfers; no bus timing changes. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif
#include "vibrometer.h"

#include <string.h>

#define IIS3DWB_WHO_AM_I_REG               0x0FU
#define IIS3DWB_FIFO_CTRL1_REG              0x07U
#define IIS3DWB_FIFO_CTRL2_REG              0x08U
#define IIS3DWB_FIFO_CTRL3_REG              0x09U
#define IIS3DWB_FIFO_CTRL4_REG              0x0AU
#define IIS3DWB_INT1_CTRL_REG               0x0DU
#define IIS3DWB_CTRL1_XL_REG                0x10U
#define IIS3DWB_CTRL3_C_REG                 0x12U
#define IIS3DWB_CTRL4_C_REG                 0x13U
#define IIS3DWB_CTRL5_C_REG                 0x14U
#define IIS3DWB_FIFO_STATUS1_REG            0x3AU
#define IIS3DWB_FIFO_DATA_OUT_TAG           0x78U

#define IIS3DWB_SPI_READ_BIT                0x80U
#define IIS3DWB_SPI_TIMEOUT_MS              100U
#define IIS3DWB_INT1_FIFO_THRESHOLD         0x08U
#define IIS3DWB_FIFO_WATERMARK              128U
#define IIS3DWB_FIFO_BDR_XL_26667           0x0AU
#define IIS3DWB_FIFO_MODE_BYPASS            0x00U
#define IIS3DWB_FIFO_MODE_CONTINUOUS        0x06U
#define IIS3DWB_FIFO_ROUNDING_ENABLE        0x20U

static SPI_HandleTypeDef *VibrometerSpi;
static uint8_t VibrometerReady;
static uint8_t VibrometerSpiTxBuffer[VIBROMETER_FIFO_DMA_MAX_BYTES + 1U];
static uint8_t VibrometerSpiRxBuffer[VIBROMETER_FIFO_DMA_MAX_BYTES + 1U];
static uint8_t VibrometerDmaRx[2][VIBROMETER_FIFO_DMA_MAX_BYTES + 1U];
static volatile uint8_t VibrometerDmaState;
static uint8_t VibrometerDmaIndex;
static uint16_t VibrometerDmaLength;
static uint32_t VibrometerDmaStarted;

void Vibrometer_AbortFifoRead(void)
{
  if (VibrometerDmaState != 0U) {
    (void)HAL_SPI_Abort(VibrometerSpi);
    HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);
    VibrometerDmaState = 0U;
  }
}

HAL_StatusTypeDef Vibrometer_ReadFifoDataAsync(uint8_t *data, uint16_t length)
{
  if (VibrometerSpi == NULL || data == NULL || length == 0U || length > VIBROMETER_FIFO_DMA_MAX_BYTES)
    return HAL_ERROR;
  if (VibrometerDmaState == 0U) {
    VibrometerSpiTxBuffer[0] = IIS3DWB_FIFO_DATA_OUT_TAG | IIS3DWB_SPI_READ_BIT;
    VibrometerDmaLength = length;
    VibrometerDmaStarted = HAL_GetTick();
    VibrometerDmaState = 1U;
    HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(VibrometerSpi, VibrometerSpiTxBuffer,
        VibrometerDmaRx[VibrometerDmaIndex], length + 1U) != HAL_OK) {
      Vibrometer_AbortFifoRead();
      return HAL_ERROR;
    }
    return HAL_BUSY;
  }
  if (VibrometerDmaLength != length || VibrometerDmaState == 3U ||
      HAL_GetTick() - VibrometerDmaStarted > 20U) {
    Vibrometer_AbortFifoRead();
    return HAL_ERROR;
  }
  if (VibrometerDmaState == 1U) return HAL_BUSY;
  memcpy(data, VibrometerDmaRx[VibrometerDmaIndex] + 1U, length);
  VibrometerDmaIndex ^= 1U;
  VibrometerDmaState = 0U;
  return HAL_OK;
}

static HAL_StatusTypeDef Vibrometer_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef Vibrometer_ReadRegisters(uint8_t reg,
                                                  uint8_t *data,
                                                  uint16_t length);

HAL_StatusTypeDef Vibrometer_Init(SPI_HandleTypeDef *hspi, uint8_t *who_am_i)
{
  HAL_StatusTypeDef status;

  if ((hspi == NULL) || (who_am_i == NULL))
  {
    return HAL_ERROR;
  }

  VibrometerSpi = hspi;
  VibrometerReady = 0U;
  *who_am_i = 0xFFU;

  HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);
  HAL_Delay(100U);

  status = Vibrometer_ReadRegisters(IIS3DWB_WHO_AM_I_REG, who_am_i, 1U);
  if ((status != HAL_OK) || (*who_am_i != VIBROMETER_WHO_AM_I_VALUE))
  {
    return (status == HAL_OK) ? HAL_ERROR : status;
  }

  status = Vibrometer_WriteRegister(IIS3DWB_FIFO_CTRL4_REG,
                                     IIS3DWB_FIFO_MODE_BYPASS);
  if (status == HAL_OK)
  {
    VibrometerReady = 1U;
  }
  return status;
}

uint8_t Vibrometer_IsReady(void)
{
  return VibrometerReady;
}

HAL_StatusTypeDef Vibrometer_StartContinuousFifo(void)
{
  HAL_StatusTypeDef status;

  if (VibrometerReady == 0U)
  {
    return HAL_ERROR;
  }

  status = Vibrometer_WriteRegister(IIS3DWB_CTRL3_C_REG, 0x01U);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(10U);
  if ((Vibrometer_WriteRegister(IIS3DWB_CTRL3_C_REG, 0x44U) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_CTRL4_C_REG, 0x04U) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_CTRL5_C_REG,
                                IIS3DWB_FIFO_ROUNDING_ENABLE) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_CTRL1_XL_REG, 0xA4U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  HAL_Delay(20U);
  if ((Vibrometer_WriteRegister(IIS3DWB_FIFO_CTRL4_REG,
                                IIS3DWB_FIFO_MODE_BYPASS) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_FIFO_CTRL1_REG,
                                (uint8_t)(IIS3DWB_FIFO_WATERMARK & 0xFFU)) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_FIFO_CTRL2_REG,
                                (uint8_t)((IIS3DWB_FIFO_WATERMARK >> 8U) & 0x01U)) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_FIFO_CTRL3_REG,
                                IIS3DWB_FIFO_BDR_XL_26667) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_INT1_CTRL_REG,
                                IIS3DWB_INT1_FIFO_THRESHOLD) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return Vibrometer_WriteRegister(IIS3DWB_FIFO_CTRL4_REG,
                                   IIS3DWB_FIFO_MODE_CONTINUOUS);
}

HAL_StatusTypeDef Vibrometer_Stop(void)
{
  if (VibrometerReady == 0U)
  {
    return HAL_ERROR;
  }

  if ((Vibrometer_WriteRegister(IIS3DWB_FIFO_CTRL4_REG,
                                IIS3DWB_FIFO_MODE_BYPASS) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_INT1_CTRL_REG, 0U) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_CTRL1_XL_REG, 0U) != HAL_OK))
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Vibrometer_StopSamplingKeepFifo(void)
{
  if (VibrometerReady == 0U)
  {
    return HAL_ERROR;
  }

  /* Stop producing new acceleration words but leave continuous FIFO mode
     intact long enough for CPU1 to drain the samples already buffered. */
  if ((Vibrometer_WriteRegister(IIS3DWB_INT1_CTRL_REG, 0U) != HAL_OK) ||
      (Vibrometer_WriteRegister(IIS3DWB_CTRL1_XL_REG, 0U) != HAL_OK))
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Vibrometer_ReadFifoStatus(uint16_t *sample_count,
                                            uint8_t *status_flags)
{
  uint8_t status_registers[2] = {0U, 0U};
  HAL_StatusTypeDef status;

  if ((sample_count == NULL) || (status_flags == NULL))
  {
    return HAL_ERROR;
  }

  status = Vibrometer_ReadRegisters(IIS3DWB_FIFO_STATUS1_REG,
                                     status_registers,
                                     sizeof(status_registers));
  if (status == HAL_OK)
  {
    *sample_count = (uint16_t)((uint16_t)status_registers[0] |
                              (((uint16_t)status_registers[1] & 0x03U) << 8U));
    *status_flags = status_registers[1];
  }
  return status;
}

HAL_StatusTypeDef Vibrometer_ReadFifoData(uint8_t *data, uint16_t length)
{
  return Vibrometer_ReadRegisters(IIS3DWB_FIFO_DATA_OUT_TAG, data, length);
}

void Vibrometer_SpiTxRxCompleteCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == VibrometerSpi && VibrometerDmaState == 1U) {
    HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);
    VibrometerDmaState = 2U;
  }
}

void Vibrometer_SpiErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == VibrometerSpi && VibrometerDmaState != 0U) {
    HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);
    VibrometerDmaState = 3U;
  }
}

static HAL_StatusTypeDef Vibrometer_WriteRegister(uint8_t reg, uint8_t value)
{
  if (VibrometerDmaState != 0U) return HAL_BUSY;
  uint8_t tx_data[2] = {(uint8_t)(reg & 0x7FU), value};
  HAL_StatusTypeDef status;

  if (VibrometerSpi == NULL)
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(VibrometerSpi,
                            tx_data,
                            sizeof(tx_data),
                            IIS3DWB_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef Vibrometer_ReadRegisters(uint8_t reg,
                                                  uint8_t *data,
                                                  uint16_t length)
{
  if (VibrometerDmaState != 0U) return HAL_BUSY;
  HAL_StatusTypeDef status;

  if ((VibrometerSpi == NULL) || (data == NULL) || (length == 0U) ||
      (length > VIBROMETER_FIFO_DMA_MAX_BYTES))
  {
    return HAL_ERROR;
  }

  (void)memset(VibrometerSpiTxBuffer, 0, (size_t)length + 1U);
  VibrometerSpiTxBuffer[0] = (uint8_t)(reg | IIS3DWB_SPI_READ_BIT);
  HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_RESET);
  /*
   * The former implementation started DMA and then busy-waited for its ISR,
   * so it offered no CPU concurrency and could stall the whole application
   * for 100 ms when concurrent ADC/SAI/IPCC interrupts delayed completion.
   * A bounded polling transfer is shorter and leaves the SPI handle in a
   * deterministic READY state after every FIFO burst.
   */
  status = HAL_SPI_TransmitReceive(VibrometerSpi,
                                   VibrometerSpiTxBuffer,
                                   VibrometerSpiRxBuffer,
                                   (uint16_t)(length + 1U),
                                   IIS3DWB_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(CS_ADWB_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    (void)memcpy(data, &VibrometerSpiRxBuffer[1], length);
  }
  return status;
}
