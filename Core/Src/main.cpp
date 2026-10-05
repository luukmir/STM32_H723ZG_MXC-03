/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_system.hpp"
#include "effects_chain.hpp"
#include "ui_controller.hpp"
#include "ui_input_manager.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
namespace LinkSync {
constexpr uint16_t DummyPatternWord{0x96A5};
constexpr uint32_t StableBlocksRequired{8};
constexpr uint32_t LockCheckWords{32};
constexpr uint32_t FrontUsbFadeInFrames{AudioConfig::SampleRate / 50U};
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA4;
SAI_HandleTypeDef hsai_BlockB4;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai4_a;
DMA_HandleTypeDef hdma_sai4_b;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef hcom_uart[];
volatile uint32_t gFrontUsbStableBlocks{0};
volatile uint32_t gFrontUsbLockAcquired{0};
volatile uint32_t gF411ReadyWasAsserted{0};
volatile uint32_t gFrontUsbFadeFrameIndex{0};
volatile uint32_t gFrontUsbFadeActive{0};
volatile uint32_t g_debug_samples_in = 0;
volatile uint32_t g_debug_samples_out = 0;
volatile uint32_t g_debug_dma_half_callbacks = 0;
volatile uint32_t g_debug_dma_full_callbacks = 0;

volatile uint32_t debug{0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_BDMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI4_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void setH723Ready(bool active) {
  HAL_GPIO_WritePin(H723_READY_OUT_GPIO_Port, H723_READY_OUT_Pin,
                    active ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool isF411ReadyAsserted() {
  return HAL_GPIO_ReadPin(F411_READY_IN_GPIO_Port, F411_READY_IN_Pin) ==
         GPIO_PIN_SET;
}

static void clearFrontUsbLockState() {
  gFrontUsbStableBlocks = 0;
  gFrontUsbLockAcquired = 0;
  gFrontUsbFadeFrameIndex = 0;
  gFrontUsbFadeActive = 0;
  setH723Ready(false);
}

static void clearFrontUsbRxBuffer() {
  std::fill(frontUSBRxBuffer.begin(), frontUSBRxBuffer.end(), 0);
}

static bool isDummyPatternStable(uint32_t start) {
  const uint32_t maxWords = AudioConfig::DMABufferSize - start;
  const uint32_t wordsToCheck =
      (LinkSync::LockCheckWords < maxWords) ? LinkSync::LockCheckWords
                                            : maxWords;

  for (uint32_t i = 0; i < wordsToCheck; ++i) {
    const uint16_t sample =
        static_cast<uint16_t>(frontUSBRxBuffer[start + i] & 0xFFFF);
    if (sample != LinkSync::DummyPatternWord) {
      return false;
    }
  }

  return true;
}

static void analyzeFrontUsbBlock(uint32_t start) {
  if (!isF411ReadyAsserted()) {
    clearFrontUsbLockState();
    return;
  }

  const bool isStableBlock = isDummyPatternStable(start);

  if (isStableBlock) {
    if (gFrontUsbStableBlocks < LinkSync::StableBlocksRequired) {
      ++gFrontUsbStableBlocks;
    }
  } else {
    gFrontUsbStableBlocks = 0;
  }

  if (gFrontUsbStableBlocks >= LinkSync::StableBlocksRequired) {
    if (gFrontUsbLockAcquired == 0U) {
      gFrontUsbFadeFrameIndex = 0;
      gFrontUsbFadeActive = 1;
      gFrontUsbLockAcquired = 1;
      setH723Ready(true);
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Initialize effects chain
  FX::initializeChain();

  // Initialize UI
  UI::initialize();
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_SAI1_Init();
  MX_SAI4_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  auto *pMonitorTx{reinterpret_cast<uint8_t *>(monitorTxBuffer.data())};
  auto *pADCRx{reinterpret_cast<uint8_t *>(ADCRxBuffer.data())};
  auto *pFrontUSBRx{reinterpret_cast<uint8_t *>(frontUSBRxBuffer.data())};
  auto *pRearUSBRx{reinterpret_cast<uint8_t *>(rearUSBRxBuffer.data())};

  setH723Ready(false);

  // Arm SAI1 RX first so H723 is ready to capture immediately when F411 starts.
  if (HAL_SAI_Receive_DMA(&hsai_BlockA1, pFrontUSBRx,
                          AudioConfig::DMABufferSize) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_SAI_Receive_DMA(&hsai_BlockB1, pRearUSBRx,
                          AudioConfig::DMABufferSize) != HAL_OK) {
    Error_Handler();
  }

  // Start SAI4 path immediately so H723 can run standalone.
  if (HAL_SAI_Transmit_DMA(&hsai_BlockB4, pMonitorTx,
                           AudioConfig::DMABufferSize) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_SAI_Receive_DMA(&hsai_BlockA4, pADCRx, AudioConfig::DMABufferSize) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim4) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }

  UI::render();
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  UiInput::initialize(htim3, hcom_uart[COM1]);
  gF411ReadyWasAsserted = isF411ReadyAsserted() ? 1U : 0U;

  while (1) {
    const bool isReadyNow = isF411ReadyAsserted();
    if (!isReadyNow) {
      if (gF411ReadyWasAsserted != 0U) {
        clearFrontUsbRxBuffer();
      }

      if (gFrontUsbLockAcquired != 0U || gFrontUsbStableBlocks != 0U) {
        clearFrontUsbLockState();
      }
    }
    gF411ReadyWasAsserted = isReadyNow ? 1U : 0U;

    debug = (int32_t)mainUSBRxBuffer.getAvailableFrames();

    UiInput::processLoop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI4A|RCC_PERIPHCLK_SAI4B
                              |RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 25;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.Sai4AClockSelection = RCC_SAI4ACLKSOURCE_PLL3;
  PeriphClkInitStruct.Sai4BClockSelection = RCC_SAI4BCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA1.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI4;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI4;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI4_Init(void)
{

  /* USER CODE BEGIN SAI4_Init 0 */

  /* USER CODE END SAI4_Init 0 */

  /* USER CODE BEGIN SAI4_Init 1 */

  /* USER CODE END SAI4_Init 1 */
  hsai_BlockA4.Instance = SAI4_Block_A;
  hsai_BlockA4.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA4.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA4.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA4.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA4.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA4.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA4.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA4.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA4.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockA4, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB4.Instance = SAI4_Block_B;
  hsai_BlockB4.Init.AudioMode = SAI_MODESLAVE_TX;
  hsai_BlockB4.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB4.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB4.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB4.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB4.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB4.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB4.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB4, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI4_Init 2 */

  /* USER CODE END SAI4_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sICConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sICConfig.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
  /* BDMA_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// --- Hardware / HAL Callbacks (C Linkage) ---
// These functions must use C linkage to be visible to the STM32 HAL drivers.
extern "C" {
void mainUSBRxBufferReset() { mainUSBRxBuffer.reset(); }
void mainUSBRxBufferWrite(int16_t *data, uint32_t length) {
  mainUSBRxBuffer.write(data, length);
}

int32_t mainUSBRxBufferGetAvailableFrames(void) {
  return (int32_t)mainUSBRxBuffer.getAvailableFrames();
}

static void processADCMonoLeftChannelInput(uint32_t start,
                                                  uint32_t numFrames) {
  for (uint32_t i = start, j = 0; j < numFrames; ++j, i += 2) {
    float32_t leftSample{static_cast<float32_t>(ADCRxBuffer[i]) *
                         Audio::Convert::Int32ToFloat};
    Mixer::chLeft[j] = leftSample * Mixer::param.gainADCLeft;
    Mixer::chRight[j] = leftSample * Mixer::param.gainADCLeft;
  }
}

static void processADCMonoRightChannelInput(uint32_t start,
                                                   uint32_t numFrames) {
  for (uint32_t i = start, j = 0; j < numFrames; ++j, i += 2) {
    float32_t rightSample{static_cast<float32_t>(ADCRxBuffer[i + 1]) *
                          Audio::Convert::Int32ToFloat};
    Mixer::chLeft[j] += rightSample * Mixer::param.gainADCRight;
    Mixer::chRight[j] += rightSample * Mixer::param.gainADCRight;
  }
}

[[maybe_unused]] static void processADCStereoInput(uint32_t start,
                                                   uint32_t numFrames) {
  for (uint32_t i = start, j = 0; j < numFrames; ++j, i += 2) {
    float32_t leftSample{static_cast<float32_t>(ADCRxBuffer[i]) *
                         Audio::Convert::Int32ToFloat};
    float32_t rightSample{static_cast<float32_t>(ADCRxBuffer[i + 1]) *
                          Audio::Convert::Int32ToFloat};

    Mixer::chLeft[j] = leftSample * Mixer::param.gainADCLeft;
    Mixer::chRight[j] = rightSample * Mixer::param.gainADCRight;
  }
}

static void processFX(uint32_t numFrames) {
  for (uint32_t i = 0; i < FX::MaxChainCount; ++i) {
    if (FX::effectChain[i] != nullptr && !FX::isBypassed[i]) {
      FX::effectChain[i]->process(Mixer::chLeft.data(), Mixer::chRight.data(),
                                  numFrames);
    }
  }
}

static void mixMainUSBAudio(uint32_t numFrames, uint32_t numSamples) {
  static std::array<int16_t, AudioConfig::FramesPerBlock * 2> usbTemp;

  std::fill(usbTemp.begin(), usbTemp.end(), 0);
  mainUSBRxBuffer.read(usbTemp.data(), numSamples);
  g_debug_samples_out += numSamples;

  for (uint32_t j = 0; j < numFrames; ++j) {
    float32_t usbLeft =
        static_cast<float32_t>(usbTemp[j * 2]) * Audio::Convert::Int16ToFloat;
    float32_t usbRight = static_cast<float32_t>(usbTemp[j * 2 + 1]) *
                         Audio::Convert::Int16ToFloat;
    Mixer::chLeft[j] += usbLeft * Mixer::param.gainMainUSB;
    Mixer::chRight[j] += usbRight * Mixer::param.gainMainUSB;
  }
}

static void mixFrontUSBAudio(uint32_t start, uint32_t numFrames) {
  if (gFrontUsbLockAcquired == 0U) {
    return;
  }

  for (uint32_t i = start, j = 0; j < numFrames; ++j, i += 2) {
    int16_t usbLeftRaw = static_cast<int16_t>(frontUSBRxBuffer[i] & 0xFFFF);
    int16_t usbRightRaw =
        static_cast<int16_t>(frontUSBRxBuffer[i + 1] & 0xFFFF);
    float32_t usbLeft =
        static_cast<float32_t>(usbLeftRaw) * Audio::Convert::Int16ToFloat;
    float32_t usbRight =
        static_cast<float32_t>(usbRightRaw) * Audio::Convert::Int16ToFloat;

    float32_t fadeGain{1.0f};
    if (gFrontUsbFadeActive != 0U) {
      if (gFrontUsbFadeFrameIndex < LinkSync::FrontUsbFadeInFrames) {
        fadeGain = static_cast<float32_t>(gFrontUsbFadeFrameIndex) /
                   static_cast<float32_t>(LinkSync::FrontUsbFadeInFrames);
        ++gFrontUsbFadeFrameIndex;
      } else {
        gFrontUsbFadeActive = 0U;
      }
    }

    Mixer::chLeft[j] += usbLeft * Mixer::param.gainFrontUSB * fadeGain;
    Mixer::chRight[j] += usbRight * Mixer::param.gainFrontUSB * fadeGain;
  }
}

static void mixRearUSBAudio(uint32_t start, uint32_t numFrames) {
  for (uint32_t i = start, j = 0; j < numFrames; ++j, i += 2) {
    int16_t usbLeftRaw = static_cast<int16_t>(rearUSBRxBuffer[i] & 0xFFFF);
    int16_t usbRightRaw =
        static_cast<int16_t>(rearUSBRxBuffer[i + 1] & 0xFFFF);
    float32_t usbLeft =
        static_cast<float32_t>(usbLeftRaw) * Audio::Convert::Int16ToFloat;
    float32_t usbRight =
        static_cast<float32_t>(usbRightRaw) * Audio::Convert::Int16ToFloat;

    Mixer::chLeft[j] += usbLeft * Mixer::param.gainRearUSB;
    Mixer::chRight[j] += usbRight * Mixer::param.gainRearUSB;
  }
}

static void processOutput(uint32_t start, uint32_t numFrames) {
  for (uint32_t i = start, j = 0; j < numFrames; ++j, i += 2) {
    monitorTxBuffer[i] =
        __SSAT(static_cast<int32_t>(Mixer::chLeft[j] * Mixer::param.gainMaster *
                                    Audio::Convert::FloatToInt32),
               32);
    monitorTxBuffer[i + 1] = __SSAT(
        static_cast<int32_t>(Mixer::chRight[j] * Mixer::param.gainMaster *
                             Audio::Convert::FloatToInt32),
        32);
  }
}

static void handleAudioBlock(uint32_t start, uint32_t end) {
  const uint32_t numSamples{end - start};
  const uint32_t numFrames{numSamples / 2};

  processADCMonoLeftChannelInput(start, numFrames);
  processADCMonoRightChannelInput(start, numFrames);
  processFX(numFrames);
  mixMainUSBAudio(numFrames, numSamples);
  mixFrontUSBAudio(start, numFrames);
  mixRearUSBAudio(start, numFrames);
  processOutput(start, numFrames);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
  if (hsai == &hsai_BlockA1) {
    analyzeFrontUsbBlock(0);
  }
  if (hsai == &hsai_BlockA4) {
    ++g_debug_dma_half_callbacks;
    handleAudioBlock(0, AudioConfig::DMABufferSize / 2);
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
  if (hsai == &hsai_BlockA1) {
    analyzeFrontUsbBlock(AudioConfig::DMABufferSize / 2);
  }
  if (hsai == &hsai_BlockA4) {
    ++g_debug_dma_full_callbacks;
    handleAudioBlock(AudioConfig::DMABufferSize / 2,
                     AudioConfig::DMABufferSize);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  UiInput::onUartRxComplete(huart);
}

void BSP_PB_Callback(Button_TypeDef Button) {
  UiInput::onUserButtonInterrupt(Button);
}
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{

  /* Disables the MPU */
  HAL_MPU_Disable();
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
