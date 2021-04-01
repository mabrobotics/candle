#include "uart.h"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>  
#include <time.h>

#include <stdbool.h>

#include <stdio.h>
#include <string.h>

struct termios tty;
struct termios ti_prev;

// #define UART_VERBOSE

int _map_baudrate(int baudrate)
{
    if ( baudrate <= 9600)
        baudrate = B9600;
    else if ( baudrate <= 57600)
        baudrate = B57600;
    else if ( baudrate <= 115200)
        baudrate = B115200;
    else if ( baudrate <= 460800)
        baudrate = B460800;
    else if ( baudrate <= 1000000)
        baudrate = B1000000;
    else if ( baudrate <= 1500000)
        baudrate = B1500000;
    else if ( baudrate <= 2000000)
        baudrate = B2000000;
    return baudrate;    
}

int uart_init(char * dev, int baudrate)
{
#ifdef UART_VERBOSE
    printf("UART starting at %s with %d baud.\n\r", dev, baudrate);
#endif
    baudrate = _map_baudrate(baudrate);
    int fd;

    fd = open(dev, O_RDWR);
    if (fd < 0) 
        return -1;
    
    tcgetattr(fd, &ti_prev);    // Save the previous serial config
    tcgetattr(fd, &tty);         // Read the previous serial config
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;  // Clear bit size setting
    tty.c_cflag |= CS8;     // 8 Bits mode
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;     // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special chars on RX
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 1;    // Wait for up to 0.1s (1 decisecond), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetospeed(&tty,baudrate);  // Set the TX baud rate
    cfsetispeed(&tty,baudrate);  // Set the RX baud rate

    //cfmakeraw(&tty);
    tcsetattr(fd, TCSANOW, &tty);  // Set the new serial config
#ifdef UART_VERBOSE
    printf("UART open.\n\r");
#endif  
    return fd;
}

int uart_transmit(int fd, char* txBuffer, int txLen)
{
#ifdef UART_VERBOSE
    printf("UART Transmitting %d bytes: ", txLen);
    for(int i = 0; i < txLen; i++)
        printf("0x%X (%c), ", (unsigned char)txBuffer[i], txBuffer[i]);
    printf("\n\r");
#endif
    int bytes = write(fd, txBuffer, txLen);
    return bytes;
}

int uart_receive(int fd, char* rxBuffer, int timeoutMs)
{    
    bool shouldStop = false;
    char tmpBuffer[128] = {0};
    usleep(5000);
    int bytesRead = read(fd, tmpBuffer, 128);

#ifdef UART_VERBOSE
    printf("UART Received %d bytes: ", bytesRead);
    for(int i = 0; i < bytesRead; i++)
        printf("0x%X (%c), ", (unsigned char)tmpBuffer[i], tmpBuffer[i]);
    printf("\n\r");
#endif
    memcpy(rxBuffer, tmpBuffer, bytesRead);
    return bytesRead;
}


int uart_restore(int fd)
{
    ti_prev.c_cflag &= ~HUPCL;        // This to release the RTS after close
    tcsetattr(fd, TCSANOW, &ti_prev); // Restore the previous serial config
    close(fd);
    return 1;
}