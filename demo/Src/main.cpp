/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "matrix.h"
#include "robotics.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0xffff);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Example 1: PUMA560 ------------------------------------------------------------------------*/
    float m[6] = {0.2645, 0.17, 0.1705, 0, 0, 0};
    Matrixf<3, 6> rc((float[18]){0, -8.5e-2, 0, 0, 0, 0, 
                                 13.225e-2, 0, 0, 0, 0, 0,
                                 0, 3.7e-2, 8.525e-2, 0, 0, 0});
    Matrixf<3, 3> I[6];
    I[0] = matrixf::diag<3, 3>((float[3]){1.542e-3, 0, 1.542e-3});
    I[1] = matrixf::diag<3, 3>((float[3]){0, 0.409e-3, 0.409e-3});
    I[2] = matrixf::diag<3, 3>((float[3]){0.413e-3, 0.413e-3, 0});
    I[3] = matrixf::eye<3, 3>() * 3.0f;
    I[4] = matrixf::eye<3, 3>() * 2.0f;
    I[5] = matrixf::eye<3, 3>() * 1.0f;

    robotics::Link links[6];
    links[0] = robotics::Link(0, 26.45e-2, 0, -PI / 2, robotics::R, 0, 0, 0, m[0], rc.col(0), I[0]);
    links[1] = robotics::Link(0, 5.5e-2, 17e-2, 0, robotics::R, 0, 0, 0, m[1], rc.col(1), I[1]);
    links[2] = robotics::Link(0, 0, 0, -PI / 2, robotics::R, 0, 0, 0, m[2], rc.col(2), I[2]);
    links[3] = robotics::Link(0, 17.05e-2, 0, PI / 2, robotics::R, 0, 0, 0, m[3], rc.col(3), I[3]);
    links[4] = robotics::Link(0, 0, 0, -PI / 2, robotics::R, 0, 0, 0, m[4], rc.col(4), I[4]);
    links[5] = robotics::Link(0, 0, 0, 0, robotics::R, 0, 0, 0, m[5], rc.col(5), I[5]);
    robotics::Serial_Link<6> p560(links);

    float q[6] = {0.2, -0.5, -0.3, -0.6, 0.5, 0.2};
    float qv[6] = {1, 0.5, -1, 0.3, 0, -1};
    float qa[6] = {0.2, -0.3, 0.1, 0, -1, 0};
    float he[6] = {1, 2, -3, -0.5, -2, 1};

    Matrixf<4, 4> T = p560.fkine(q);
    Matrixf<6, 6> J = p560.jacob(q);
    Matrixf<6, 1> q_ikine = p560.ikine(T, Matrixf<6, 1>((float[6]){0, 0, 0, 0, 0.1, 0}));
    Matrixf<6, 1> torq = p560.rne(q, qv, qa, he);

    /* Example 2: UR -----------------------------------------------------------------------------*/
//    robotics::Link links[6];
//    links[0] = robotics::Link(0, 0, 0, PI / 2, robotics::R, 0);
//    links[1] = robotics::Link(0, 0, 5, 0, robotics::R, PI / 2);
//    links[2] = robotics::Link(0, 0, 5, 0, robotics::R, -PI / 2);
//    links[3] = robotics::Link(0, 1, 0, -PI / 2, robotics::R, 0);
//    links[4] = robotics::Link(0, 1, 0, -PI / 2, robotics::R, -PI / 2);
//    links[5] = robotics::Link(0, 1, 0, 0, robotics::R, 0);
//    robotics::Serial_Link<6> UR(links);

//    float q[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
//    Matrixf<4, 4> T = UR.fkine(q);
//    Matrixf<6, 6> J = UR.jacob(q);
//    Matrixf<6, 1> q_ikine = UR.ikine(T);

    /* Example 3: SCARA --------------------------------------------------------------------------*/
//    robotics::Link links[4];
//    links[0] = robotics::Link(0, 5, 5, 0);
//    links[1] = robotics::Link(0, 0, 5, 0);
//    links[2] = robotics::Link(0, 0, 0, PI);
//    links[3] = robotics::Link(0, 0, 0, 0, robotics::P);
//    robotics::Serial_Link<4> SCARA(links);

//    float q[4] = {0.2, 0.5, 0.3, 3};
//    Matrixf<4, 4> T = SCARA.fkine(q);
//    Matrixf<6, 4> J = SCARA.jacob(q);
//    Matrixf<4, 1> q_ikine = SCARA.ikine(T, Matrixf<4, 1>((float[4]){0, 0, 0, 0}));

//    // QR & SVD, unfinished in current version
//    // QR<6, 4> J_qr(J);
//    // Matrixf<6, 6> Q = J_qr.Q();
//    // Matrixf<6, 4> R = J_qr.R();
//    // SVD<6, 4> J_svd(J);

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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
