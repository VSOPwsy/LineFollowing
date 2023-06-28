/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
struct PID {
    float desired_angle;  // Desired angle
    float desired_distance; // Desired distance from center
    float previous_angle_error; // Angle error from previous cycle
    float previous_distance_error; // Distance error from previous cycle
    float integral_angle; // Sum of angle errors
    float integral_distance; // Sum of distance errors
    float Kp_angle, Ki_angle, Kd_angle; // PID coefficients for angle
    float Kp_distance, Ki_distance, Kd_distance; // PID coefficients for distance
};

struct PID pid_control;


struct UART {
    unsigned char frame_header;
    unsigned char angle_error;
    unsigned char distance_error;
    unsigned char frame_tail;
    unsigned char state;
    unsigned char buf[2];
    unsigned char received;
};

struct UART uart;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void update_pid(struct PID *pid, float current_angle, float current_distance, void (*motor_function)(int, int));
void Motor_Rotation(int, int);

void Motor_Left_Front(int);
void Motor_Right_Front(int);
void Motor_Left_Rear(int);
void Motor_Right_Rear(int);
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
  uart.frame_header = 0xAA;
  uart.angle_error = 0;
  uart.distance_error = 0;
  uart.frame_tail = 0x55;
  uart.state = 0;

  for (int i = 0; i < 2; i++)
  {
      uart.buf[i] = 0;
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_UART_Receive_IT(&huart2, (unsigned char *)&uart.received, 1);
    float current_angle = uart.angle_error; // Get the current angle of the motors
    float current_distance = uart.distance_error; // Get the current distance from center of the motors
    update_pid(&pid_control, current_angle, current_distance, Motor_Rotation);
    
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
}

/* USER CODE BEGIN 4 */
void update_pid(struct PID *pid, float current_angle, float current_distance, void (*motor_function)(int, int)) {
    float angle_error = pid->desired_angle - current_angle;
    pid->integral_angle += angle_error;
    float angle_derivative = angle_error - pid->previous_angle_error;

    float distance_error = pid->desired_distance - current_distance;
    pid->integral_distance += distance_error;
    float distance_derivative = distance_error - pid->previous_distance_error;

    int output_angle = pid->Kp_angle*angle_error + pid->Ki_angle*pid->integral_angle + pid->Kd_angle*angle_derivative;
    int output_distance = pid->Kp_distance*distance_error + pid->Ki_distance*pid->integral_distance + pid->Kd_distance*distance_derivative;
    motor_function(output_angle, output_distance);

    pid->previous_angle_error = angle_error;
    pid->previous_distance_error = distance_error;
}

void Motor_Rotation(int angle, int distance)
{
  Motor_Left_Front(40000 + angle);
  Motor_Left_Rear(40000 + angle);
  Motor_Right_Front(40000 - angle);
  Motor_Right_Front(40000 - angle);
}

void Motor_Left_Front(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -0.5 * speed + 32768);
}
void Motor_Right_Front(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0.5 * speed + 32768);
}
void Motor_Left_Rear(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -0.5 * speed + 32768);
}
void Motor_Right_Rear(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0.5 * speed + 32768);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (uart.state == 0 && uart.received == uart.frame_header)
    {
      uart.state += 1;
    }
    else if (uart.state == 1)
    {
      uart.state += 1;
      uart.buf[0] = uart.received;
    }
    else if (uart.state == 2)
    {
      uart.state += 1;
      uart.buf[1] = uart.received;
    }
    else if (uart.state == 3 && uart.received == uart.frame_tail)
    {
      uart.state = 0;
      uart.angle_error = uart.buf[0];
      uart.distance_error = uart.buf[1];
      uart.buf[0] = 0;
      uart.buf[1] = 0;
    }
    else
    {
      uart.state = 0;
      uart.buf[0] = 0;
      uart.buf[1] = 0;
    }

    HAL_UART_Receive_IT(&huart2, (unsigned char *)&uart.received, 1);
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
