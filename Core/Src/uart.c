#include "uart.h"

struct UART_RX_DATA uart_rx_data;

void UART_Init(void)
{
    uart_rx_data.frame_header = 0xAA;
    uart_rx_data.length = 0;
    uart_rx_data.frame_tail = 0x55;
    uart_rx_data.state = 0;
    uart_rx_data.received = 0;

    for (int i = 0; i < 40; i++)
    {
      uart_rx_data.data[i] = 0;
      uart_rx_data.buf[i] = 0;
    }
}

void UART_Receive(void)
{
    if (uart_rx_data.state == 0 && uart_rx_data.received == uart_rx_data.frame_header)
    {
      uart_rx_data.state += 1;
    }
    else if (uart_rx_data.state == 1 && uart_rx_data.received < 40)
    {
      uart_rx_data.length = uart_rx_data.received;
      uart_rx_data.state += 1;
    }
    else if (uart_rx_data.state > 1 && uart_rx_data.state - 2 < uart_rx_data.length)
    {
      uart_rx_data.buf[uart_rx_data.state - 2] = uart_rx_data.received;
      uart_rx_data.state += 1;
    }
    else if (uart_rx_data.state - 2 == uart_rx_data.length && uart_rx_data.received == uart_rx_data.frame_tail)
    {
      uart_rx_data.state = 0;
      for (int i = 0; i < uart_rx_data.length; i++)
      {
        uart_rx_data.data[i] = uart_rx_data.buf[i];
        uart_rx_data.buf[i] = 0;
      }
    }
    else
    {
      uart_rx_data.state = 0;
      for (int i = 0; i < uart_rx_data.state - 2; i++)
      {
        uart_rx_data.buf[i] = 0;
      }
    }
}
