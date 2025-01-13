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
#define BUFFER_SIZE 1024
char buffer[BUFFER_SIZE];
char rs485buffer[BUFFER_SIZE];
// char hexBuffer[BUFFER_SIZE];

int32_t status;
int32_t status2;
size_t bytesFromAntenna;
// size_t bytesWritten = 0;


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

                rs485NewSock = sl_Accept(rs485Sock, (struct SlSockAddr_t *)&sAddr, (SlSocklen_t *)&addrSize);

                // Check if a connection was successfully accepted
                if (rs485NewSock >= 0) {
                    UART_PRINT("[RS485 task] Successfully accepted a connection on RS485\n");


                    while (1) {
                        // Check for data from the antenna (RS485)
                        bytesAntenna = 0;
                        // bytesWifi = 0;

                        //clearing buffers before reads
                        memset(buffer, 0, BUFFER_SIZE); // Clear the buffer before each read


                        GPIO_write(CONFIG_GPIO_RE_DE, 0); // Set RE/DE to receive mode
                        usleep(100); // Stabilization delay

                        status = UART2_read(uart485Handle, buffer, BUFFER_SIZE, &bytesFromAntenna);
                        buffer[bytesAntenna] = '\0'; // Null-terminate after reading
                        UART_PRINT("status: %d ..... bytesAntenna: %u ...... buffer: %s \n", status, bytesFromAntenna, buffer);


                        if (status == 0) {
                             // Client has disconnected
                             UART_PRINT("[RS485 task] Client disconnected.\n");
                             sl_Close(rs485NewSock);
                             rs485NewSock = -1; // Reset the socket
                             break; // Exit the loop and wait for a new connection
                        }

                        if (status < 0) {
                             if (status == SL_ERROR_BSD_EAGAIN) {
                                    // No data available in non-blocking mode, retry
                                    continue;
                                }
                                UART_PRINT("Error receiving data from Wi-Fi: %d\n", status);
                                break; // Exit loop on critical error
                        }

                        if (status > 0 && buffer[0] != '\0') {

                            messageLength = buffer[1]; // Byte 1 indicates the message length
                            buffer[messageLength] = '\0'; // Null-terminate after reading
                            UART_PRINT("\rline 228 -> bytesFromAntenna: %d ... messageLength: %d\n", bytesFromAntenna, messageLength);

                            if (bytesFromAntenna >= messageLength) {
                                UART_PRINT("Received from Antenna: %.*s\n", messageLength, buffer);

                                while (1) {
                                    size_t filteredLength = filterTrailingZeroes(buffer, bytesFromAntenna);
                                    messageLength = buffer[1];
                                    buffer[messageLength] = '\0';

                                    //Send antenna data over wifi to the connected device
                                    bytesSentToWifi = sl_Send(rs485NewSock, buffer, filteredLength, 0);
                                    UART_PRINT("\rline 240 -> bytesSentToWifi: %d ... messageLength: %d ... filteredLength: %d\n", bytesSentToWifi, messageLength, filteredLength);

                                    // Convert sent data to hex and print
                                    memset(hexBuffer, 0, BUFFER_SIZE);
                                    convertToHex(buffer, hexBuffer, filteredLength);
                                    UART_PRINT("\rHex sent to Wi-Fi: %s\n", hexBuffer);

                                    if (bytesSentToWifi < 0) {
                                    UART_PRINT("Error sending to Wi-Fi client.\n");
                                    break;
                                    }

                                     memset(rs485Buffer, 0, BUFFER_SIZE);
                                     bytesReceivedFromWifi = sl_Recv(rs485NewSock, rs485Buffer, BUFFER_SIZE, 0);
                                     UART_PRINT("\rline 254 -> bytesReceivedFromWifi: %d\n", bytesReceivedFromWifi);

                                    if (bytesReceivedFromWifi > 1 && rs485Buffer[0] != '\0') {
                                         messageLength = rs485Buffer[1];
                                         rs485Buffer[messageLength] = '\0'; // Null-terminate after reading

                                }

                            }



                                bytesSentToUart = sl_Recv(rs485NewSock, rs485buffer,BUFFER_SIZE, 0);

                                if (bytesSentToWifi < 0) {
                                    sl_Close(rs485NewSock);
                                    rs485NewSock = -1;
                                    break; // Exit the loop
                            }

                                if(bytesSentToUart > 0){
                                        messageLength = rs485buffer[1];

                                        if (bytesSentToUart == messageLength) {

                                           rs485buffer[bytesSentToUart] = '\0';  // Null-terminate for printing

                                           //transmit device data to the antenna from the uart
                                           GPIO_write(CONFIG_GPIO_RE_DE, 1); // Set RE/DE to transmit mode
                                           usleep(100);
                                           UART2_write(uartRS485Handle, rs485buffer, bytesSentToUart, &bytesFromAntenna);
                                           usleep(RESPONSE_DELAY_MS * 1000); // Wait for Antenna Unit response (20-50ms)

                                           //open the transceiver to receive antenna data
                                           GPIO_write(CONFIG_GPIO_RE_DE, 0); // Set RE/DE to receive mode
                                           usleep(100);
                                           UART2_read(uartRS485Handle, buffer, BUFFER_SIZE, &bytesFromAntenna);

                                           if(bytesFromAntenna > 0 && buffer[0] != '\0') {
                                            messageLength = buffer[1];
                                            UART_PRINT("\rMessage Length from ANTENNA: %d\n", messageLength);
                                            buffer[bytesFromAntenna] = '\0';  // Null-terminate for printing

                                            bytesSentToWifi = sl_Send(rs485NewSock, buffer, bytesFromAntenna, 0);
                                            UART_PRINT("\rline263 -> bytesFromAntenna: %d ..... buffer: %s .... bytesSentToWifi\n", bytesFromAntenna, buffer, bytesSentToWifi);

                                           }
                                        }
                                }
                                usleep((COMMAND_INTERVAL_MS - RESPONSE_DELAY_MS) * 1000); // Maintain 250ms cycle
                            }

                        }
                    // // Continue to listen for incoming connections on the UART side
                    // bytesRead485 = sl_Recv(rs485NewSock, buffer, sizeof(buffer), 0);
                    // UART_PRINT("[RS485 task] bytesRead value: %d\n", bytesRead485);

                    // if (bytesRead485 > 0) {
                    //     UART_PRINT("[RS485 task] Received data: %s\n", buffer);

                    //      // Process the received data
                    //     // sendRS485Data(buffer, bytesRead485); // Echo back or process the data
                    //     sl_Send(rs485NewSock, rs485DataBuffer, bytesRead, 0);
                    // } else if (bytesRead485 == SL_ERROR_BSD_EAGAIN) {
                    //     // No data available, continue listening
                    //     // usleep(RS485_NB_TIMEOUT * 1000);
                    // } else {
                    //     UART_PRINT("[RS485 task] Error receiving data, closing connection\n");
                    //     sl_Close(rs485NewSock); // Close the socket on error
                    //     break;
                    // }
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
       }
}

// Function to convert a buffer to hex representation
void convertToHex(const char *input, char *hexBuffer, size_t inputLength) {
    for (size_t i = 0; i < inputLength; i++) {
        snprintf(hexBuffer + (i * 2), 3, "%02X", (unsigned char)input[i]);
    }
}

// Function to filter out trailing zeroes
size_t filterTrailingZeroes(char *input, size_t length) {
    size_t actualLength = length;
    while (actualLength > 0 && input[actualLength - 1] == 0) {
        actualLength--; // Reduce length for trailing zero
    }
    return actualLength;
}
