#include "uart.h"

struct UART_RX_DATA uart_rx_data;

void UART_Init(void)
{
    uart_rx_data.frame_header = 0xAA;
    uart_rx_data.angle_error = 0;
    uart_rx_data.distance_error = 0;
    uart_rx_data.frame_tail = 0x55;
    uart_rx_data.state = 0;
    uart_rx_data.buf[0] = 0;
    uart_rx_data.buf[1] = 0;
    uart_rx_data.received = 0;
}

void UART_Receive(void)
{
    if (uart_rx_data.state == 0 && uart_rx_data.received == uart_rx_data.frame_header)
    {
      uart_rx_data.state += 1;
    }
    else if (uart_rx_data.state == 1)
    {
      uart_rx_data.state += 1;
      uart_rx_data.buf[0] = uart_rx_data.received;
    }
    else if (uart_rx_data.state == 2)
    {
      uart_rx_data.state += 1;
      uart_rx_data.buf[1] = uart_rx_data.received;
    }
    else if (uart_rx_data.state == 3 && uart_rx_data.received == uart_rx_data.frame_tail)
    {
      uart_rx_data.state = 0;
      uart_rx_data.angle_error = uart_rx_data.buf[0];
      uart_rx_data.distance_error = uart_rx_data.buf[1];
      uart_rx_data.buf[0] = 0;
      uart_rx_data.buf[1] = 0;
    }
    else
    {
      uart_rx_data.state = 0;
      uart_rx_data.buf[0] = 0;
      uart_rx_data.buf[1] = 0;
    }
}
