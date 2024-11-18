/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
//
//! \addtogroup out_of_box
//! @{
//*****************************************************************************

/* standard includes */
#include <stdlib.h>
#include <string.h>

/* POSIX Header files */
#ifdef USE_FREERTOS
#include <sys/signal.h>
#include <os/freertos/posix/time.h>
#endif

/* TI-DRIVERS Header files */
#include <ti_drivers_config.h>
#include <uart_term.h>
//#include "uart_ant.h"

/* Example/Board Header files */
#include "ota_task.h"
#include "link_local_task.h"
#include "out_of_box.h"

#define OTA_REPORT_SERVER_PORT       (5432)
#define RS485_SERVER_PORT            (5000)    // +++++ RS485 port +++++
#define OTA_REPORT_TIMEOUT           (10)      /* in mSec */
#define OTA_NB_TIMEOUT               (20)      /* in mSec */
#define RS485_NB_TIMEOUT             (20)      // +++++ Timeout for RS485 +++++
#define OTA_BLOCKED_TIMEOUT          (20)      /* in seconds */
#define OTA_NUM_OF_ACCEPT_TRIALS     (1000 / OTA_NB_TIMEOUT * OTA_BLOCKED_TIMEOUT)
#define RS485_NUM_OF_ACCEPT_TRIALS   (1000 / RS485_NB_TIMEOUT * OTA_BLOCKED_TIMEOUT) // +++++ RS485 trials +++++
#define OTA_NUM_OF_MAILBOX_TRIALS    (1000 / OTA_REPORT_TIMEOUT * OTA_BLOCKED_TIMEOUT)

#define OTA_PROGRESS_BAR_STR_LEN     (4)
#define HTTP_HEADER_METADATA_STR_LEN (105)

extern int snprintf(char *_string, size_t _n, const char *_format, ...);

/****************************************************************************
                      GLOBAL VARIABLES
****************************************************************************/
// data buffer used to send messages reporting on ota progress
uint8_t gPayloadData[32];
uint8_t gMetadataResponse[512];
uint8_t rs485DataBuffer[512];    // +++++ RS485 data buffer +++++
extern UART2_Handle uartRS485Handle;
extern UART2_Handle uartHandle;

//*****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************

//*****************************************************************************
//
//! \brief This task handles ota status reports to client
//!
//! \param[in]  None
//!
//! \return None
//!
//*****************************************************************************
#define BUFFER_SIZE 64
char buffer[BUFFER_SIZE];

int32_t status;
int32_t status2;
size_t bytesAntenna;
size_t bytesWritten = 0;


void * otaTask(void *pvParameter)
{


    uint16_t acceptTrials


    int16_t rs485Sock;    // +++++ RS485 socket +++++
    int16_t rs485NewSock = -1;  // +++++ RS485 new client socket +++++
    int32_t status;
    int32_t nonBlocking = 0;
    int16_t addrSize;
    // uint32_t contentLen;

    SlSockAddrIn_t sAddr;
    SlSockAddrIn_t rs485Addr;    // +++++ RS485 address +++++


ota_task_restart:


// +++++++++++++++++++++++++++++++++++++++++++++++ RS485 server setup starts ++++++++++++++++++++++++++++++++++++++++
    rs485Addr.sin_family = SL_AF_INET;
    rs485Addr.sin_port = sl_Htons((uint16_t)RS485_SERVER_PORT);
    rs485Addr.sin_addr.s_addr = SL_INADDR_ANY;

    UART_PRINT("[ota report task] Attempting to open RS485 socket on port %d...\n", RS485_SERVER_PORT);
    rs485Sock = sl_Socket(rs485Addr.sin_family, SL_SOCK_STREAM, 0);
    if(rs485Sock < 0)
    {
        UART_PRINT("[RS485 task] Error opening socket, %d \n\r", rs485Sock);
        goto ota_task_restart;
    }
    UART_PRINT("[ota report task] RS485 socket opened successfully, attempting to bind...\n");
    status = sl_Bind(rs485Sock, (SlSockAddr_t *)&rs485Addr, addrSize);
    if(status < 0)
    {
        UART_PRINT("[RS485 task] Error binding socket, error %d \n\r", status);
        sl_Close(rs485Sock);
        goto ota_task_restart;
    }
    UART_PRINT("[ota report task] RS485 socket bound successfully, attempting to listen...\n");
    status = sl_Listen(rs485Sock, 0);
    if(status < 0)
    {
        UART_PRINT("[RS485 task] Error listening on socket, error %d \n\r", status);
        sl_Close(rs485Sock);
        goto ota_task_restart;
    }
    UART_PRINT("[ota report task] Attempting to set RS485 socket as non-blocking...\n");
    status = sl_SetSockOpt(rs485Sock, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &nonBlocking,
                           sizeof(nonBlocking));
    if(status < 0)
    {
        UART_PRINT("[RS485 task] Error setting socket as non-blocking, error %d \n\r", status);
        sl_Close(rs485Sock);
        goto ota_task_restart;
    }
    initAnt();
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++ RS485 server setup ends +++++++++++++++++++++++++++++++++++++

    while(1)
    {

        acceptTrials = 0;

        rs485NewSock = SL_ERROR_BSD_EAGAIN;
        // UART_PRINT("Inside While Loop line 205");


// ++++++++++++++++++++++++++++++++++++ RS485 client accept starts ++++++++++++++++++++++++++++++++++++++++++++++++++
            UART_PRINT("rs485NewSock value: %d", rs485NewSock);
            while(rs485NewSock < 0)
            {
                UART_PRINT("Attempting to accept RS485 connection with rs485Sock value: %d \n", rs485Sock);
                rs485NewSock = sl_Accept(rs485Sock, (struct SlSockAddr_t *)&sAddr, (SlSocklen_t *)&addrSize);
                UART_PRINT("After sl_Accept, rs485NewSock: %d\n", rs485NewSock);


                // Check if a connection was successfully accepted
                if (rs485NewSock >= 0) {
                    UART_PRINT("[RS485 task] Successfully accepted a connection on RS485\n");
                    GPIO_write(CONFIG_GPIO_RE_DE, 0); // Set RE/DE to receive mode

                    while (1) {
                        // Check for data from the antenna (RS485)
                        bytesAntenna = 0;
                        bytesWifi = 0;

                        status = UART2_read(uart485Handle, buffer, BUFFER_SIZE, &bytesAntenna);
                        UART_PRINT("status: %d, bytesAntenna: %u\n", status, bytesAntenna);

                        if (status != UART2_STATUS_SUCCESS) {
                            UART_PRINT("Error reading bytesRead\n");
                            break;  // Exit loop on error
                        }

                        if (bytesAntenna > 0) {
                            buffer[bytesAntenna] = '\0';
                            UART_PRINT("Received from antenna: %s\n", buffer);

                            //send bytesAntenna data over wifi
                            int bytesSentToWifi = sl_Send(rs485NewSock, buffer, bytesAntenna, 0);
                            UART_PRINT("bytesSentToWifi: %d", bytesSentToWifi);
                        }

                        UART_PRINT("[RS485 task] bytesReceived: %d;  rs485Buffer: %d\n", bytesReceived, rs485Buffer);

                        if (bytesSentToWifi <  0) {
                             UART_PRINT("Error sending data to TCP server\n");
                             break;
                        }
                            // Process the received RS485 data
                            UART_PRINT("Sent to TCP Server: %s\n", buffer);

                            // Send data from wifi to uart
                        **** sendRS485Data(rs485Buffer, bytesReceived);
                            sl_Send(rs485NewSock, rs485DataBuffer, bytesRead, 0);

                            // Optionally send the same data back to the antenna via RS485
//                            sendRS485Data(rs485Buffer, bytesReceived);
                        }


                    // Continue to listen for incoming connections on the UART side
                    bytesRead485 = sl_Recv(rs485NewSock, buffer, sizeof(buffer), 0);
                    UART_PRINT("[RS485 task] bytesRead value: %d\n", bytesRead485);

                    if (bytesRead485 > 0) {
                        UART_PRINT("[RS485 task] Received data: %s\n", buffer);

                         // Process the received data
                        // sendRS485Data(buffer, bytesRead485); // Echo back or process the data
                        sl_Send(rs485NewSock, rs485DataBuffer, bytesRead, 0);
                    } else if (bytesRead485 == SL_ERROR_BSD_EAGAIN) {
                        // No data available, continue listening
                        usleep(RS485_NB_TIMEOUT * 1000);
                    } else {
                        UART_PRINT("[RS485 task] Error receiving data, closing connection\n");
                        sl_Close(rs485NewSock); // Close the socket on error
                        break;
                    }
                }
             } else {
                UART_PRINT("[RS485 task] Error accepting RS485 client connection: %d\n", rs485NewSock);

                if((rs485NewSock == SL_ERROR_BSD_EAGAIN) && nonBlocking)
                {
                    usleep(RS485_NB_TIMEOUT * 1000);
                    acceptTrials++;
                    if(acceptTrials > RS485_NUM_OF_ACCEPT_TRIALS)
                    {
                        UART_PRINT("[RS485 task] Error accepting RS485 client connection\n\r");
                        goto ota_task_end;
                    }
                }
            }
// ++++++++++++++++++++++++++++++++++++++++ RS485 client accept ends +++++++++++++++++++++++++++++++++++++++++++++++++


// +++++++++++++++++++++++++++++++++++++++++++++++++ RS485 data reception +++++++++++++++++++++++++++++++++++++++++++++++++++++++
               status = sl_Recv(rs485NewSock, rs485DataBuffer, sizeof(rs485DataBuffer), 0);
               if(status > 0)
               {
                   UART2_write(uartRS485Handle, rs485DataBuffer, status, NULL);
                   int bytesRead = UART2_read(uartRS485Handle, rs485DataBuffer, sizeof(rs485DataBuffer), NULL);
                   if (bytesRead > 0)
                   {
                       sl_Send(rs485NewSock, rs485DataBuffer, bytesRead, 0);
                   }
               }
               else if(status < 0)
               {
                   sl_Close(rs485NewSock);
                   goto ota_task_accept_start;
               }
                while (1) {
                    status = sl_Recv(rs485NewSock, rs485DataBuffer, sizeof(rs485DataBuffer), 0);
                    if (status > 0) {
                        UART2_write(uartRS485Handle, rs485DataBuffer, status, NULL);
                        int bytesRead = UART2_read(uartRS485Handle, rs485DataBuffer, sizeof(rs485DataBuffer), NULL);
                        if (bytesRead > 0) {
                            sl_Send(rs485NewSock, rs485DataBuffer, bytesRead, 0);
                        }
                    } else if (status == SL_ERROR_BSD_EAGAIN) {
                        // No data received, continue listening
                        usleep(RS485_NB_TIMEOUT * 1000);  // Small delay before retrying
                        continue;
                    } else {
                        // Handle other error cases, but do NOT close the socket
                        UART_PRINT("[RS485 task] Error receiving data, continuing to listen...\n\r");
                        continue;
                    }
                }
            }
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ RS485 data reception ends ++++++++++++++++++++++++++++++++++++
            }
    }
