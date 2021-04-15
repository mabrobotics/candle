#ifndef UART_H_
#define UART_H_

#ifdef __cplusplus
extern "C" {
#endif

int uart_init(char * dev, int baudrate);
int uart_transmit(int fd, char* txBuffer, int txLen);
int uart_receive(int fd, char* rxBuffer, int timeoutMs);
int uart_restore(int fd);

#ifdef __cplusplus
}
#endif

#endif