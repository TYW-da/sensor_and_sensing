/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include "stdio.h"
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t A_raw = 0;
uint16_t B_raw = 0;
float angle = 0;

typedef struct {
    int steps_per_revolution;
    int position;
    uint16_t prev_A;
    uint16_t prev_B;
    int last_position;
    uint32_t last_update_time;
    float speed;
} QuadratureEncoder;

void QuadratureEncoder_Init(QuadratureEncoder *encoder, int steps_per_revolution) {
    encoder->steps_per_revolution = steps_per_revolution;
    encoder->position = 0;
    encoder->prev_A = 0;
    encoder->prev_B = 0;
    encoder->last_position = 0;
    encoder->last_update_time = HAL_GetTick();
    encoder->speed = 0.0f;
}

void QuadratureEncoder_Update(QuadratureEncoder *encoder, uint16_t A_raw, uint16_t B_raw) {
    uint16_t A = A_raw;
    uint16_t B = B_raw;

    uint16_t state = (encoder->prev_A << 1) | encoder->prev_B;
    uint16_t new_state = (A << 1) | B;

    // Determine the delta based on state transitions
    int16_t delta = 0;
    if (state == 0b00) {
        if (new_state == 0b01) delta = 1;
        else if (new_state == 0b10) delta = -1;
    } else if (state == 0b01) {
        if (new_state == 0b11) delta = 1;
        else if (new_state == 0b00) delta = -1;
    } else if (state == 0b10) {
        if (new_state == 0b00) delta = 1;
        else if (new_state == 0b11) delta = -1;
    } else if (state == 0b11) {
        if (new_state == 0b10) delta = 1;
        else if (new_state == 0b01) delta = -1;
    }

    encoder->position += delta;

    encoder->prev_A = A;
    encoder->prev_B = B;

    // Calculate speed
    uint32_t current_time = HAL_GetTick();
    uint32_t dt = current_time - encoder->last_update_time; // Time difference in milliseconds

    if (dt > 0) {
        int position_change = encoder->position - encoder->last_position;
        encoder->speed = (1000.0f * position_change) / dt; // Speed in steps per second
    }

    // Update last position and time
    encoder->last_position = encoder->position;
    encoder->last_update_time = current_time;
}

float QuadratureEncoder_GetAngle(QuadratureEncoder *encoder) {
    return ((float)encoder->position / encoder->steps_per_revolution) * 360.0f;
}


QuadratureEncoder encoder;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  QuadratureEncoder_Init(&encoder, 80);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  A_raw = HAL_GPIO_ReadPin(ACH_GPIO_Port, ACH_Pin);
	  B_raw = HAL_GPIO_ReadPin(BCH_GPIO_Port, BCH_Pin);

	  QuadratureEncoder_Update(&encoder, A_raw, B_raw);

      angle = QuadratureEncoder_GetAngle(&encoder);


      snprintf(UserTxBufferFS, sizeof(UserTxBufferFS), "Speed: %.2f steps/s | Angle: %.2f°\r\n", encoder.speed, angle);

      /* Send data to USB CDC */
      CDC_Transmit_FS((uint8_t*)UserTxBufferFS, strlen(UserTxBufferFS));

      HAL_Delay(100); // Delay to prevent excessive printing

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACH_Pin BCH_Pin */
  GPIO_InitStruct.Pin = ACH_Pin|BCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
