// #include <stdlib.h>
// #include <string.h>
// #include <ti/drivers/net/wifi/simplelink.h>
// #include <ti/drivers/UART2.h>
// #include <ti/drivers/GPIO.h>
// #include <uart_term.h>
// #include "ti_drivers_config.h"
// #include <unistd.h>
// #include "uart_ant.h"
// #include "tcp_server.h"
// #include <pthread.h> // Include pthread header

// #define RS485_SERVER_PORT 5000
// #define RS485_BUFFER_SIZE 512 //up from 256

// extern UART2_Handle uartRS485Handle;

// // Function to send debug messages via RS485
// void sendRS485DebugMessage(const char *debugMessage) {
//     int status = UART2_write(uartRS485Handle, debugMessage, strlen(debugMessage), NULL);
//     if (status < 0) {
//         UART_PRINT("Failed to send debug message via RS485\n");
//     }
// }

// int setupTCPServer() {
//     int serverSock;
//     int status;
//     SlSockAddrIn_t serverAddr;
//     int nonBlocking = 1;

//     // Debug message
//     sendRS485DebugMessage("Setting up TCP server...\n");

//     serverSock = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
//     if (serverSock < 0) {
//         sendRS485DebugMessage("Error creating socket\n");
//         return serverSock;
//     }

//     serverAddr.sin_family = SL_AF_INET;
//     serverAddr.sin_port = sl_Htons(RS485_SERVER_PORT);
//     serverAddr.sin_addr.s_addr = SL_INADDR_ANY;

//     status = sl_Bind(serverSock, (SlSockAddr_t *)&serverAddr, sizeof(SlSockAddrIn_t));
//     if (status < 0) {
//         sl_Close(serverSock);
//         sendRS485DebugMessage("Error binding socket\n");
//         return status;
//     }

//     status = sl_Listen(serverSock, 0);
//     if (status < 0) {
//         sl_Close(serverSock);
//         sendRS485DebugMessage("Error listening on socket\n");
//         return status;
//     }

//     status = sl_SetSockOpt(serverSock, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &nonBlocking, sizeof(nonBlocking));
//     if (status < 0) {
//         sl_Close(serverSock);
//         sendRS485DebugMessage("Error setting socket to non-blocking mode\n");
//         return status;
//     }

//     return serverSock;
// }

// void *rs485ServerTask(void *pvParameter) {
//     int serverSock = *((int *)pvParameter);  // Correctly dereference the pointer
//     int clientSock;
//     SlSockAddrIn_t clientAddr;
//     int addrSize = sizeof(SlSockAddrIn_t);
//     char buffer[RS485_BUFFER_SIZE];
//     int status;

//     while (1) {
//         // Accept a new client connection
//         clientSock = sl_Accept(serverSock, (SlSockAddr_t *)&clientAddr, (SlSocklen_t *)&addrSize);
//         if (clientSock < 0) {
//             usleep(10000);  // Sleep for 10ms to avoid busy-waiting
//             continue;
//         }

//         sendRS485DebugMessage("Client connected\n");

//         // Continuous communication with the client
//         while (1) {
//             // Receive data from the client
//             status = sl_Recv(clientSock, buffer, RS485_BUFFER_SIZE, 0);
//             if (status > 0) {
//                 sendRS485DebugMessage("Received data from client\n");

//                 // Forward data to UART (RS485)
//                 status = UART2_write(uartRS485Handle, buffer, status, NULL);
//                 if (status > 0) {
//                     sendRS485DebugMessage("Data sent to RS485\n");
//                 } else {
//                     sendRS485DebugMessage("UART2_write failed\n");
//                 }

//                 // Read response from UART (RS485)
//                 int bytesRead = UART2_read(uartRS485Handle, buffer, RS485_BUFFER_SIZE, NULL);
//                 if (bytesRead > 0) {
//                     sendRS485DebugMessage("Data received from RS485\n");

//                     // Forward response back to the client
//                     status = sl_Send(clientSock, buffer, bytesRead, 0);
//                     if (status < 0) {
//                         sendRS485DebugMessage("Failed to send data back to client\n");
//                     }
//                 } else {
//                     sendRS485DebugMessage("UART2_read failed\n");
//                 }
//             } else if (status == 0) {
//                 // Connection closed by the client
//                 sendRS485DebugMessage("Client disconnected\n");
//                 continue;
//             } else if (status == SL_ERROR_BSD_EAGAIN) {
//                 // No data available, continue waiting
//                 usleep(10000);  // Sleep for 10ms
//             } else {
//                 // Error occurred
//                 sendRS485DebugMessage("Error receiving data from client\n");
//                 sl_Close(clientSock);
//                 break;
//             }
//         }
//     }
//     return NULL;
// }
//Refactoring tcp_server to define and create pins from pinMux
#include <stddef.h>
#include <stdint.h>

#ifndef DeviceFamily_CC3220
#define DeviceFamily_CC3220
#endif

#include <ti/devices/DeviceFamily.h>

#include "ti_drivers_config.h"
/*  =============================== UART2 ===============================
*/

#include <ti/drivers/UART2.h>
#include <ti/devices/cc32xx/inc/hw_ints.h>
#include <ti/devices/cc32xx/inc/hw_memmap.h>
#include <ti/drivers/uart2/UART2CC32XX.h>

#define CONFIG_UART2_COUNT 2
#define CONFIG_UART2_0 0  // Define UART2_0 as index 0
#define CONFIG_UART2_1 1  // Define UART2_1 as index 1

#define UART0_BASE UARTA0_BASE
#define UART1_BASE UARTA1_BASE
#define INT_UART0  INT_UARTA0
#define INT_UART1  INT_UARTA1

static unsigned char uart2RxRingBuffer0[32];
static unsigned char uart2RxRingBuffer1[32];
/* TX ring buffer allocated to be used for nonblocking mode */
//static unsigned char uart2TxRingBuffer0[32];
static unsigned char uart2TxRingBuffer1[32];

/* UART2 Pin Definitions for GPIO12 (Pin 9) and GPIO13 (Pin 10) */
#define UART2CC32XX_PIN_09_UART0_TX 9    // GPIO12, Pin 9 for UART0_TX
#define UART2CC32XX_PIN_10_UART0_RX 10   // GPIO13, Pin 10 for UART0_RX

#define UART2CC32XX_PIN_02_UART1_TX 2
#define UART2CC32XX_PIN_03_UART1_RX 3

UART2CC32XX_Object uart2CC32XXObjects0;

static const UART2CC32XX_HWAttrs uart2CC32XXHWAttrs0 = {
    .baseAddr           = UART0_BASE,
    .intNum             = INT_UART0,
    .intPriority        = 0x20,
    .flowControl        = UART2_FLOWCTRL_NONE,
    .rxDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
    .txDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
    // Configure for Pin 9 (TX) and Pin 10 (RX)
    .rxPin              = UART2CC32XX_PIN_10_UART0_RX,
    .txPin              = UART2CC32XX_PIN_09_UART0_TX,
    .ctsPin             = UART2CC32XX_PIN_UNASSIGNED,
    .rtsPin             = UART2CC32XX_PIN_UNASSIGNED,
    .rxBufPtr           = uart2RxRingBuffer0,
    .rxBufSize          = sizeof(uart2RxRingBuffer0),
    .txBufPtr           = NULL,
    .txBufSize          = 0,
};

UART2CC32XX_Object uart2CC32XXObjects1;

static const UART2CC32XX_HWAttrs uart2CC32XXHWAttrs1 = {
    .baseAddr           = UART1_BASE,
    .intNum             = INT_UART1,
    .intPriority        = (~0),
    .flowControl        = UART2_FLOWCTRL_NONE,
    .rxDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
    .txDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
    .rxPin              = UART2CC32XX_PIN_03_UART1_RX,
    .txPin              = UART2CC32XX_PIN_02_UART1_TX,
    .ctsPin             = UART2CC32XX_PIN_UNASSIGNED,
    .rtsPin             = UART2CC32XX_PIN_UNASSIGNED,
    .rxBufPtr           = uart2RxRingBuffer1,
    .rxBufSize          = sizeof(uart2RxRingBuffer1),
    .txBufPtr           = uart2TxRingBuffer1,
    .txBufSize          = sizeof(uart2TxRingBuffer1)
};

const UART2_Config UART2_config[CONFIG_UART2_COUNT] = {
    {   /* CONFIG_UART2_0 */
        .object      = &uart2CC32XXObjects0,
        .hwAttrs     = &uart2CC32XXHWAttrs0
    },
    {   /* CONFIG_UART2_1 */
        .object      = &uart2CC32XXObjects1,
        .hwAttrs     = &uart2CC32XXHWAttrs1
    },
};

const uint_least8_t CONFIG_UART2_0_CONST = CONFIG_UART2_0;
const uint_least8_t CONFIG_UART2_1_CONST = CONFIG_UART2_1;
const uint_least8_t UART2_count = CONFIG_UART2_COUNT;
