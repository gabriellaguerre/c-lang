
#include <ti/drivers/UART2.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include <ti/devices/cc32xx/driverlib/prcm.h>  // Include PRCM for clock management
#include <ti/devices/cc32xx/driverlib/pin.h>   // Include PinTypeUART for pin configuration
#include <uart_term.h>  // Include for UART_PRINT

// UART handle
UART2_Handle uartRS485Handle;
UART2_Params uartParams;

// Initialize UART for RS485
void initAnt() {

    UART2_Params_init(&uartParams);
    uartParams.baudRate = 9600;                  // Set baud rate to 9600
    uartParams.dataLength = UART2_DataLen_8;     // 8 data bits
    uartParams.stopBits = UART2_StopBits_1;      // 1 stop bit
    uartParams.parityType = UART2_Parity_NONE;   // No parity
    uartParams.readMode = UART2_Mode_BLOCKING;   // blocking mode for read
    uartParams.writeMode = UART2_Mode_BLOCKING;  // blocking mode for write


    // Open the UART with the configured parameters
    uartRS485Handle = UART2_open(UART0_rs485comm, &uartParams);
    if (uartRS485Handle == NULL) {
        // UART2_open() failed
       UART_PRINT("UART2_open failed\n");
        while (1);
    }
}

// Close UART
void closeUART() {
    // Close the UART
    UART2_close(uartRS485Handle);
}
