#include "task_1.h"
#include "line_following.h"
#include "uart.h"

#include "usart.h"
#include "gpio.h"

void TASK_1_Init()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    UART_Init();
    Line_Following_Init();
}

void task_1()
{
    while (uart_rx_data.data[2] == 0)
    {
        follow_line();
    }
    
}

void follow_line()
{
  float current_angle = uart_rx_data.data[0];
  float current_distance = uart_rx_data.data[1];
  update_pid(&pid_control, current_angle, current_distance, Motor_Rotation);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Receive();
    HAL_UART_Receive_IT(&huart2, (unsigned char *)&uart_rx_data.received, 1);
}
