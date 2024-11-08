
#include <ti/drivers/UART2.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include <ti/devices/cc32xx/driverlib/prcm.h>  // Include PRCM for clock management
#include <ti/devices/cc32xx/driverlib/pin.h>   // Include PinTypeUART for pin configuration
#include <uart_term.h>  // Include for UART_PRINT

// UART handle
UART2_Handle uartRS485Handle;
UART2_Params uartParams;

// Function to send debug messages over RS485
void sendRS485DebugMessage(const char* message) {
    UART2_write(uartRS485Handle, message, strlen(message), NULL);
}

// Declare callback functions
void readCallback(UART2_Handle handle, void *buf, size_t count, void *userArg, int_fast16_t status);
void writeCallback(UART2_Handle handle, void *buf, size_t count, void *userArg, int_fast16_t status);

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
        sendRS485DebugMessage("UART2_open failed\n");
        while (1);
    } else {
        sendRS485DebugMessage("UART2_open succeeded\n");
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

    while (1) {
        GPIO_write(CONFIG_GPIO_RE_DE, 0);  // Set to receive mode

        int bytesRead = UART2_read(uartRS485Handle, buffer, sizeof(buffer), NULL);
        UART_PRINT("Read attempt: bytesRead = %d\n", bytesRead);

        if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; // Null-terminate the received data
            UART_PRINT("Received from antenna: %s\n", buffer);

        } else if (bytesRead == SL_ERROR_BSD_EAGAIN) {
               // No data available, wait and try again
            usleep(100000); // Adjust as necessary

        } else {
            UART_PRINT("Error reading from UART\n");
            break;
        }
        // Add a small delay to avoid busy-waiting

    }
}
