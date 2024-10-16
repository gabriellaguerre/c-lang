
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

    // Configure GPIO12 (pin 9) and GPIO13 (pin 10) for UART0
    // PinTypeUART(PIN_01, PIN_MODE_1);  // UART0 TX
    // PinTypeUART(PIN_02, PIN_MODE_1); // UART0 RX
    // Initialize the UART parameters
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 9600;                  // Set baud rate to 9600
    uartParams.dataLength = UART2_DataLen_8;     // 8 data bits
    uartParams.stopBits = UART2_StopBits_1;      // 1 stop bit
    uartParams.parityType = UART2_Parity_NONE;   // No parity
    uartParams.readMode = UART2_Mode_CALLBACK;   // Non-blocking mode for read
    uartParams.writeMode = UART2_Mode_CALLBACK;  // Non-blocking mode for write
    uartParams.readCallback = readCallback;      // Read callback function
    uartParams.writeCallback = writeCallback;    // Write callback function

    // Open the UART with the configured parameters
    uartRS485Handle = UART2_open(CONFIG_UART2_0, &uartParams);
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

// Callback function for read operations
void readCallback(UART2_Handle handle, void *buf, size_t count, void *userArg, int_fast16_t status) {
    if (status == UART2_STATUS_SUCCESS) {
        sendRS485DebugMessage("Read successful\n");
    } else {
        sendRS485DebugMessage("Read failed\n");
    }
}

// Callback function for write operations
void writeCallback(UART2_Handle handle, void *buf, size_t count, void *userArg, int_fast16_t status) {
    if (status == UART2_STATUS_SUCCESS) {
        sendRS485DebugMessage("Write successful\n");
    } else {
        sendRS485DebugMessage("Write failed\n");
    }
}

void sendRS485Data(const char *data, size_t dataSize) {
    // Set DE high to enable transmission mode
    GPIO_write(51, 1);

    // Transmit data over UART
    UART2_write(uartRS485Handle, data, dataSize, NULL);

    // After transmission, set DE low to enable reception mode
    GPIO_write(51, 0);
}

void receiveRS485Data(char *buffer, size_t bufferSize) {
    // Set DE low to enable reception mode
    GPIO_write(51, 0);

    // Read data from UART
    int bytesRead = UART2_read(uartRS485Handle, buffer, bufferSize, NULL);
    return bytesRead;
}

void initRS485ControlPin() {
    // Initialize the GPIO driver if not already initialized
//    GPIO_init();

    // Set GPIO6 (Pin 51) as an output pin for DE/RE control
    GPIO_setConfig(51, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW);

    // Ensure the transceiver is in receive mode by setting DE low initially
    GPIO_write(51, 0);  // Set DE/RE to receive mode (low)
}
