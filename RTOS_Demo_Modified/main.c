/* ────────── CubeMX 기본 헤더 ────────── */
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

/* ────────── 매크로 & 타입 선언 ────────── */
#define PMIC_I2C_ADDR   (0x48 << 1)
#define EEPROM_DTC_ADDR 0x0000U
#define CAN_ID_DTC_REQ  0x700U
#define CAN_ID_DTC_RESP 0x708U

typedef enum { PMIC_REG_UV=0x00, PMIC_REG_OC=0x01, PMIC_REG_OV=0x02 } PMIC_Reg_t;

typedef union __attribute__((packed))
{
    uint8_t raw[8];
    struct {
        uint8_t uv_flag:1, oc_flag:1, ov_flag:1, reserved1:5;
        uint8_t uv_th, oc_th, ov_th, reserved2[4];
    } bits;
} PMIC_RegFrame_t;

extern I2C_HandleTypeDef  hi2c1;
extern SPI_HandleTypeDef  hspi2;
extern CAN_HandleTypeDef  hcan1;
extern UART_HandleTypeDef huart3;

/* ────────── 전역 변수 ────────── */
static volatile uint8_t dtc_code[3]   = {0};   /* IRQ/Task 동시 접근 → volatile */
static volatile uint8_t pmic_rx_buf[8];        /* DMA 대상 버퍼 → volatile */

static osMutexId_t hMutex;

/* ────────── 함수 프로토타입 ────────── */
static void Task_I2C (void *argument);
static void Task_SPI (void *argument);
static void Task_CAN (void *argument);
static void Task_UART(void *argument);

static HAL_StatusTypeDef PMIC_ReadReg_DMA (PMIC_Reg_t reg, PMIC_RegFrame_t *dst);
static HAL_StatusTypeDef EEPROM_Write_DMA(uint16_t addr, const uint8_t *d, uint16_t len);
static HAL_StatusTypeDef EEPROM_Read_DMA (uint16_t addr,       uint8_t *d, uint16_t len);
static inline void CAN_SendDTC(const uint8_t *dtc);
static inline void UART_Log  (const char *msg);

/* ────────── main ────────── */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();

  /* ---- FreeRTOS 객체 생성 ---- */
  osKernelInitialize();
  const osMutexAttr_t mAttr = { .name = "TaskMutex" };
  hMutex = osMutexNew(&mAttr);

  const osThreadAttr_t attrN = { .stack_size = 512 };
  const osThreadAttr_t attrL = { .stack_size = 512, .priority = osPriorityLow };

  osThreadNew(Task_I2C , NULL, &attrN);
  osThreadNew(Task_SPI , NULL, &attrL);
  osThreadNew(Task_CAN , NULL, &attrL);
  osThreadNew(Task_UART, NULL, &attrL);

  osKernelStart();           /* 스케줄러 시작 */
  while (1) { }              /* 도달하지 않음 */
}

/* ────────── RTOS 태스크 ────────── */
static void Task_I2C(void *argument)
{
  PMIC_RegFrame_t frame;
  for (;;)
  {
    PMIC_ReadReg_DMA(PMIC_REG_UV, &frame);

    dtc_code[0] = frame.bits.uv_flag ? 0x12 : 0x00;
    dtc_code[1] = frame.bits.oc_flag ? 0x34 : 0x00;
    dtc_code[2] = frame.bits.ov_flag ? 0x56 : 0x00;

    osMutexRelease(hMutex);        /* SPI 태스크 깨움 */
    osDelay(10);
  }
}

static void Task_SPI(void *argument)
{
  for (;;)
  {
    osMutexAcquire(hMutex, osWaitForever);
    EEPROM_Write_DMA(EEPROM_DTC_ADDR, dtc_code, sizeof dtc_code);
    osDelay(2);
    EEPROM_Read_DMA (EEPROM_DTC_ADDR, dtc_code, sizeof dtc_code);
    osMutexRelease(hMutex);        /* CAN 태스크 깨움 */
  }
}

static void Task_CAN(void *argument)
{
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  for (;;)
  {
    osMutexAcquire(hMutex, osWaitForever);
    CAN_SendDTC(dtc_code);
    osMutexRelease(hMutex);        /* UART 태스크 깨움 */
  }
}

static void Task_UART(void *argument)
{
  for (;;)
  {
    osMutexAcquire(hMutex, osWaitForever);
    UART_Log("ECU OK\r\n");
    osDelay(100);
  }
}

/* ────────── HAL 래퍼 ────────── */
static HAL_StatusTypeDef PMIC_ReadReg_DMA(PMIC_Reg_t reg, PMIC_RegFrame_t *dst)
{
  if (HAL_I2C_Mem_Read_DMA(&hi2c1, PMIC_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)pmic_rx_buf, sizeof pmic_rx_buf) != HAL_OK)
      return HAL_ERROR;
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  memcpy(dst->raw, (const void *)pmic_rx_buf, sizeof pmic_rx_buf);
  return HAL_OK;
}

static HAL_StatusTypeDef EEPROM_Write_DMA(uint16_t addr, const uint8_t *d, uint16_t len)
{
  uint8_t hdr[3] = {0x02, addr >> 8, addr & 0xFF};
  HAL_SPI_Transmit_DMA(&hspi2, hdr, 3);
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
  HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)d, len);
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
  return HAL_OK;
}

static HAL_StatusTypeDef EEPROM_Read_DMA(uint16_t addr, uint8_t *d, uint16_t len)
{
  uint8_t hdr[3] = {0x03, addr >> 8, addr & 0xFF};
  HAL_SPI_Transmit_DMA(&hspi2, hdr, 3);
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
  HAL_SPI_Receive_DMA(&hspi2, d, len);
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
  return HAL_OK;
}

/* ────────── inline 함수 ────────── */
static inline void CAN_SendDTC(const uint8_t *dtc)
{
  CAN_TxHeaderTypeDef tx = {
      .StdId = CAN_ID_DTC_RESP,
      .IDE   = CAN_ID_STD,
      .RTR   = CAN_RTR_DATA,
      .DLC   = 3
  };
  uint32_t mbx;
  HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)dtc, &mbx);
}

static inline void UART_Log(const char *msg)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* ────────── CAN RX 콜백 ────────── */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx; uint8_t data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data);

  if (rx.StdId == CAN_ID_DTC_REQ && data[0] == 0x14) {   /* DTC Clear */
      memset((void *)dtc_code, 0, sizeof dtc_code);
      osMutexRelease(hMutex);      /* SPI 태스크에게 알림 */
  }

  char buf[64];
  snprintf(buf, sizeof(buf),
           "RX %03lX [%d] %02X %02X %02X\r\n",
           (unsigned long)rx.StdId, rx.DLC,
           data[0], data[1], data[2]);
  UART_Log(buf);
}

/* ────────── 클록 & 오류 핸들러 ────────── */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
      Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
