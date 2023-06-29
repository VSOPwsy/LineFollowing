struct UART_RX_DATA {
    unsigned char frame_header;
    unsigned char angle_error;
    unsigned char distance_error;
    unsigned char stop;
    unsigned char frame_tail;
    unsigned char state;
    unsigned char buf[3];
    unsigned char received;
};

extern struct UART_RX_DATA uart_rx_data;

void UART_Init(void);

void UART_Receive(void);
