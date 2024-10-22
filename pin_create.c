#include <stdbool.h>
#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/drivers/GPIO.h>
#include <ti/devices/cc32xx/driverlib/gpio.h>
#include <ti/devices/cc32xx/driverlib/pin.h>
#include "create_uart0_term.h"

void configureGPIOForUART0(void) {
    // Configure Pin 46 (UART1 TX)
    GPIO_setConfig(UART0_TX_PIN, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // Configure Pin 47 (UART1 RX)
    GPIO_setConfig(UART0_RX_PIN, GPIO_CFG_INPUT | GPIO_CFG_IN_PU);
}
