
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
    } else {
        UART_PRINT("UART2_open succeeded\n");
    }
}

// Close UART
void closeUART() {
    // Close the UART
    UART2_close(uartRS485Handle);
}


void sendRS485Data(const char *data, size_t dataSize) {
    // Set DE high to enable transmission mode
    GPIO_write(6, 1);

    // Transmit data over UART
    UART2_write(uartRS485Handle, data, dataSize, NULL);

    // After transmission, set DE low to enable reception mode
    GPIO_write(6, 0);
}


void receiveAntennaData() {

    UART_PRINT("Receiving data from antenna...\n");
    int bytesRead;
    int bytesWritten;

    while (1) {
        // GPIO_write(CONFIG_GPIO_RE_DE, 0);  // Set to receive mode
          // int bytesRead = UART2_read(uartRS485Handle, buffer, sizeof(buffer), NULL);

        bytesRead = 0;
        while (bytesRead == 0) {
            status = UART2_read(uartRS485Handle, buffer, BUFFER_SIZE, &bytesRead);
            UART_PRINT("status: %d, bytesRead: %u\n", status, bytesRead);

        if(status != UART2_STATUS_SUCCESS) {
                UART_PRINT("error reading bytesRead");
                break;
            }

        }
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            UART_PRINT("Received from antenna: %s\n, buffer");

            bytesWritten = 0;
            status2 = UART2_write(uartRS485Handle, buffer, BUFFER_SIZE, &bytesWritten);
            UART_PRINT("status2: %d, bytesWritten: %d\n", status2, bytesWritten);

            if(status2 != UART2_STATUS_SUCCESS) {
                UART_PRINT("error reading bytesRead");
                break;
           }

        }

    }
}
