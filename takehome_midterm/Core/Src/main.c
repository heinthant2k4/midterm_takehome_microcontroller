/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

#define MA_N 12

/* ADC */
uint16_t adc_dma_value = 0;
uint16_t adc_raw = 0;
uint16_t adc_filtered = 0;

/* Moving average */
uint16_t ma_buffer[MA_N] = {0};
uint32_t ma_sum = 0;
uint8_t ma_index = 0;
uint8_t ma_count = 0;

/* Mode control */
uint8_t mode = 0;

GPIO_PinState last_button_state = GPIO_PIN_SET;
uint32_t last_button_time = 0;

uint32_t last_blink_time = 0;
GPIO_PinState pc13_state = GPIO_PIN_SET;

/* Actuators */
uint16_t servo_us = 1500;
uint16_t led_ccr = 0;

/* STM32 -> ESP32 uplink */
uint8_t uplink_packet[11];
uint8_t checksum_debug = 0;
uint8_t uart_tx_busy = 0;

/* ESP32 -> STM32 downlink */
uint8_t uart_rx_byte = 0;
uint8_t downlink_packet[6];

uint8_t rx_state = 0;
uint8_t rx_index = 0;
uint8_t test_downlink[6] =
{
    0xCC,
    0x33,
    2,
    0x08,
    0x00,
    0xFF
};

/* Last VALID ESP32 command */
uint16_t esp32_command_value = 2048;

/* Link monitoring */
uint32_t last_valid_rx_time = 0;
uint8_t link_ok = 0;
uint8_t received_valid_frame_once = 0;

/* Debug counters */
uint32_t valid_rx_count = 0;
uint32_t invalid_rx_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */

uint16_t MovingAverage_Update(uint16_t new_value)
{
    ma_sum -= ma_buffer[ma_index];

    ma_buffer[ma_index] = new_value;
    ma_sum += new_value;

    ma_index++;

    if (ma_index >= MA_N)
    {
        ma_index = 0;
    }

    if (ma_count < MA_N)
    {
        ma_count++;
    }

    return (uint16_t)(ma_sum / ma_count);
}


uint16_t ADC_To_LED_CCR(uint16_t adc)
{
    return (uint16_t)(((uint32_t)adc * 399) / 4095);
}


uint16_t ADC_To_Servo_US(uint16_t adc)
{
    return (uint16_t)(1000 + ((uint32_t)adc * 1000) / 4095);
}


void Check_Button(void)
{
    GPIO_PinState current_state =
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

    /*
     * Internal pull-up:
     * released = HIGH
     * pressed  = LOW
     */
    if ((last_button_state == GPIO_PIN_SET) &&
        (current_state == GPIO_PIN_RESET))
    {
        uint32_t now = HAL_GetTick();

        if ((now - last_button_time) >= 250)
        {
            mode++;

            if (mode > 2)
            {
                mode = 0;
            }

            last_button_time = now;

            /* Restart mode-indicator timing */
            last_blink_time = now;
            pc13_state = GPIO_PIN_SET;
        }
    }

    last_button_state = current_state;
}


void Update_Mode_LED(void)
{
    uint32_t now = HAL_GetTick();

    if (mode == 0)
    {
        /*
         * Blue Pill PC13 built-in LED
         * is normally active LOW.
         *
         * Mode 0 = solid ON.
         */
        HAL_GPIO_WritePin(
            GPIOC,
            GPIO_PIN_13,
            GPIO_PIN_RESET
        );
    }

    else if (mode == 1)
    {
        /*
         * 1 Hz blink:
         * complete period = 1000 ms
         * toggle every 500 ms
         */
        if ((now - last_blink_time) >= 500)
        {
            last_blink_time = now;

            pc13_state =
                (pc13_state == GPIO_PIN_SET)
                ? GPIO_PIN_RESET
                : GPIO_PIN_SET;

            HAL_GPIO_WritePin(
                GPIOC,
                GPIO_PIN_13,
                pc13_state
            );
        }
    }

    else
    {
        /*
         * Mode 2: 5 Hz
         * complete period = 200 ms
         * toggle every 100 ms
         */
        if ((now - last_blink_time) >= 100)
        {
            last_blink_time = now;

            pc13_state =
                (pc13_state == GPIO_PIN_SET)
                ? GPIO_PIN_RESET
                : GPIO_PIN_SET;

            HAL_GPIO_WritePin(
                GPIOC,
                GPIO_PIN_13,
                pc13_state
            );
        }
    }
}


uint8_t XOR_Checksum(
    uint8_t *data,
    uint8_t start,
    uint8_t end
)
{
    uint8_t chk = 0;

    for (uint8_t i = start; i <= end; i++)
    {
        chk ^= data[i];
    }

    return chk;
}


void Build_Uplink_Packet(void)
{
    /*
     * STM32 -> ESP32
     *
     * 0  = 0xAA
     * 1  = 0x55
     * 2  = mode
     * 3  = raw_H
     * 4  = raw_L
     * 5  = filt_H
     * 6  = filt_L
     * 7  = us_H
     * 8  = us_L
     * 9  = N
     * 10 = checksum
     */

    uplink_packet[0] = 0xAA;
    uplink_packet[1] = 0x55;

    uplink_packet[2] = mode;

    uplink_packet[3] =
        (uint8_t)((adc_raw >> 8) & 0xFF);

    uplink_packet[4] =
        (uint8_t)(adc_raw & 0xFF);

    uplink_packet[5] =
        (uint8_t)((adc_filtered >> 8) & 0xFF);

    uplink_packet[6] =
        (uint8_t)(adc_filtered & 0xFF);

    uplink_packet[7] =
        (uint8_t)((servo_us >> 8) & 0xFF);

    uplink_packet[8] =
        (uint8_t)(servo_us & 0xFF);

    uplink_packet[9] = MA_N;

    checksum_debug =
        XOR_Checksum(uplink_packet, 2, 9);

    uplink_packet[10] = checksum_debug;
}


void Process_Downlink_Packet(uint8_t *packet)
{
    /*
     * ESP32 -> STM32
     *
     * 0 = 0xCC
     * 1 = 0x33
     * 2 = mode
     * 3 = cmd_H
     * 4 = cmd_L
     * 5 = checksum
     */

    /* Check header first */
    if ((packet[0] != 0xCC) ||
        (packet[1] != 0x33))
    {
        invalid_rx_count++;
        return;
    }

    /* Check XOR checksum */
    uint8_t expected_checksum =
        packet[2] ^
        packet[3] ^
        packet[4];

    if (expected_checksum != packet[5])
    {
        invalid_rx_count++;
        return;
    }

    /* Mode must be valid */
    if (packet[2] > 2)
    {
        invalid_rx_count++;
        return;
    }

    uint16_t received_command =
        ((uint16_t)packet[3] << 8) |
        packet[4];

    /* Command uses ADC-style 12-bit range */
    if (received_command > 4095)
    {
        invalid_rx_count++;
        return;
    }

    /*
     * IMPORTANT:
     * Only update control values after
     * ALL validation passes.
     */
    mode = packet[2];

    esp32_command_value =
        received_command;

    last_valid_rx_time =
        HAL_GetTick();

    received_valid_frame_once = 1;
    link_ok = 1;

    valid_rx_count++;
}


void Update_Link_Status(void)
{
    if (received_valid_frame_once == 0)
    {
        link_ok = 0;
        return;
    }

    if ((HAL_GetTick() - last_valid_rx_time)
        <= 500)
    {
        link_ok = 1;
    }
    else
    {
        link_ok = 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */

  /* ADC calibration required before conversion */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Start servo PWM */
  HAL_TIM_PWM_Start(
      &htim1,
      TIM_CHANNEL_1
  );

  /* Start external LED PWM */
  HAL_TIM_PWM_Start(
      &htim4,
      TIM_CHANNEL_1
  );

  /* Start ADC in DMA circular mode */
  HAL_ADC_Start_DMA(
      &hadc1,
      (uint32_t *)&adc_dma_value,
      1
  );

  /*
   * Start TIM3.
   * TIM3 TRGO triggers ADC at 1000 Hz.
   */
  HAL_TIM_Base_Start(&htim3);

  /*
   * Start TIM2 at 20 Hz.
   * This is our main control scheduler.
   */
  HAL_TIM_Base_Start_IT(&htim2);

  /*
   * Start USART2 reception.
   * Receive one byte at a time.
   */
  HAL_UART_Receive_IT(
      &huart2,
      &uart_rx_byte,
      1
  );

  Process_Downlink_Packet(test_downlink);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Check_Button();
	  Update_Mode_LED();
	  Update_Link_Status();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        adc_raw = adc_dma_value;

        adc_filtered = MovingAverage_Update(adc_raw);

        uint16_t servo_control = 0;
        uint16_t led_control = 0;

        if (mode == 0)
        {
            servo_control = adc_filtered;
            led_control = adc_filtered;
        }
        else if (mode == 1)
        {
            servo_control = adc_filtered;
            led_control = adc_raw;
        }
        else
        {
            servo_control = esp32_command_value;
            led_control = esp32_command_value;
        }

        servo_us = ADC_To_Servo_US(servo_control);
        led_ccr = ADC_To_LED_CCR(led_control);

        __HAL_TIM_SET_COMPARE(
            &htim1,
            TIM_CHANNEL_1,
            servo_us
        );

        __HAL_TIM_SET_COMPARE(
            &htim4,
            TIM_CHANNEL_1,
            led_ccr
        );

        if (uart_tx_busy == 0)
        {
            Build_Uplink_Packet();

            uart_tx_busy = 1;

            HAL_StatusTypeDef tx_status;

            tx_status = HAL_UART_Transmit_IT(
                &huart2,
                uplink_packet,
                11
            );

            if (tx_status != HAL_OK)
            {
                uart_tx_busy = 0;
            }
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart_tx_busy = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        switch (rx_state)
        {
            case 0:

                if (uart_rx_byte == 0xCC)
                {
                    downlink_packet[0] = uart_rx_byte;
                    rx_state = 1;
                }

                break;

            case 1:

                if (uart_rx_byte == 0x33)
                {
                    downlink_packet[1] = uart_rx_byte;
                    rx_index = 2;
                    rx_state = 2;
                }
                else if (uart_rx_byte == 0xCC)
                {
                    downlink_packet[0] = 0xCC;
                    rx_state = 1;
                }
                else
                {
                    rx_state = 0;
                }

                break;

            case 2:

                downlink_packet[rx_index] = uart_rx_byte;
                rx_index++;

                if (rx_index >= 6)
                {
                    Process_Downlink_Packet(downlink_packet);

                    rx_index = 0;
                    rx_state = 0;
                }

                break;

            default:

                rx_index = 0;
                rx_state = 0;

                break;
        }

        HAL_UART_Receive_IT(
            &huart2,
            &uart_rx_byte,
            1
        );
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        rx_index = 0;
        rx_state = 0;

        HAL_UART_Receive_IT(
            &huart2,
            &uart_rx_byte,
            1
        );
    }
}

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
#ifdef USE_FULL_ASSERT
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
