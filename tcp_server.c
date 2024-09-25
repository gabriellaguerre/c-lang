#include <stdlib.h>
#include <string.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/GPIO.h>
#include <uart_term.h>
#include "ti_drivers_config.h"
#include <unistd.h>
#include "uart_ant.h"
#include "tcp_server.h"
#include <pthread.h> // Include pthread header

#define RS485_SERVER_PORT 5000
#define RS485_BUFFER_SIZE 512 //up from 256

extern UART2_Handle uartRS485Handle;

// Function to send debug messages via RS485
void sendRS485DebugMessage(const char *debugMessage) {
    int status = UART2_write(uartRS485Handle, debugMessage, strlen(debugMessage), NULL);
    if (status < 0) {
        UART_PRINT("Failed to send debug message via RS485\n");
    }
}

int setupTCPServer() {
    int serverSock;
    int status;
    SlSockAddrIn_t serverAddr;
    int nonBlocking = 1;

    // Debug message
    sendRS485DebugMessage("Setting up TCP server...\n");

    serverSock = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
    if (serverSock < 0) {
        sendRS485DebugMessage("Error creating socket\n");
        return serverSock;
    }

    serverAddr.sin_family = SL_AF_INET;
    serverAddr.sin_port = sl_Htons(RS485_SERVER_PORT);
    serverAddr.sin_addr.s_addr = SL_INADDR_ANY;

    status = sl_Bind(serverSock, (SlSockAddr_t *)&serverAddr, sizeof(SlSockAddrIn_t));
    if (status < 0) {
        sl_Close(serverSock);
        sendRS485DebugMessage("Error binding socket\n");
        return status;
    }

    status = sl_Listen(serverSock, 0);
    if (status < 0) {
        sl_Close(serverSock);
        sendRS485DebugMessage("Error listening on socket\n");
        return status;
    }

    status = sl_SetSockOpt(serverSock, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &nonBlocking, sizeof(nonBlocking));
    if (status < 0) {
        sl_Close(serverSock);
        sendRS485DebugMessage("Error setting socket to non-blocking mode\n");
        return status;
    }

    return serverSock;
}

void *rs485ServerTask(void *pvParameter) {
    int serverSock = *((int *)pvParameter);  // Correctly dereference the pointer
    int clientSock;
    SlSockAddrIn_t clientAddr;
    int addrSize = sizeof(SlSockAddrIn_t);
    char buffer[RS485_BUFFER_SIZE];
    int status;

    while (1) {
        // Accept a new client connection
        clientSock = sl_Accept(serverSock, (SlSockAddr_t *)&clientAddr, (SlSocklen_t *)&addrSize);
        if (clientSock < 0) {
            usleep(10000);  // Sleep for 10ms to avoid busy-waiting
            continue;
        }

        sendRS485DebugMessage("Client connected\n");

        // Continuous communication with the client
        while (1) {
            // Receive data from the client
            status = sl_Recv(clientSock, buffer, RS485_BUFFER_SIZE, 0);
            if (status > 0) {
                sendRS485DebugMessage("Received data from client\n");

                // Forward data to UART (RS485)
                status = UART2_write(uartRS485Handle, buffer, status, NULL);
                if (status > 0) {
                    sendRS485DebugMessage("Data sent to RS485\n");
                } else {
                    sendRS485DebugMessage("UART2_write failed\n");
                }

                // Read response from UART (RS485)
                int bytesRead = UART2_read(uartRS485Handle, buffer, RS485_BUFFER_SIZE, NULL);
                if (bytesRead > 0) {
                    sendRS485DebugMessage("Data received from RS485\n");

                    // Forward response back to the client
                    status = sl_Send(clientSock, buffer, bytesRead, 0);
                    if (status < 0) {
                        sendRS485DebugMessage("Failed to send data back to client\n");
                    }
                } else {
                    sendRS485DebugMessage("UART2_read failed\n");
                }
            } else if (status == 0) {
                // Connection closed by the client
                sendRS485DebugMessage("Client disconnected\n");
                continue;
            } else if (status == SL_ERROR_BSD_EAGAIN) {
                // No data available, continue waiting
                usleep(10000);  // Sleep for 10ms
            } else {
                // Error occurred
                sendRS485DebugMessage("Error receiving data from client\n");
                sl_Close(clientSock);
                break;
            }
        }
    }
    return NULL;
}
