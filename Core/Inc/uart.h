struct UART_RX_DATA {
    unsigned char frame_header;
    unsigned char length;
    unsigned char data[40];
    unsigned char frame_tail;
    unsigned char state;
    unsigned char buf[40];
    unsigned char received;
};

extern struct UART_RX_DATA uart_rx_data;

void UART_Init(void);

void UART_Receive(void);
