
#ifndef CREATE_UART0_H
#define CREATE_UART0_H

#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/drivers/GPIO.h>
#include <ti/devices/cc32xx/driverlib/gpio.h>

// Define UART1 TX and RX Pin Numbers
#define UART0_TX_PIN    9  // Pin 9 for UART0 TX
#define UART0_RX_PIN   10  // Pin 10 for UART0 RX

// Function to configure GPIO for UART0
void configureGPIOForUART0(void);



#endif // CREATE_UART0_H
