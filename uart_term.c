/*
 *  UART Terminal Implementation for RT-400 using Pins 2 (TX) and 3 (RX)
 */

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "uart_term.h"
#include <ti/drivers/UART2.h>
#include <ti/devices/cc32xx/driverlib/pin.h>  // Ensure correct header file for pin configuration
#include <unistd.h>

extern int vsnprintf(char * s, size_t n, const char * format, va_list arg);

//*****************************************************************************
//                              LOCAL DEFINES
//*****************************************************************************
#define IS_SPACE(x)       (x == 32 ? 1 : 0)

// Define the actual UART TX and RX pin numbers for RT-400 board
//#define RT400_UART_TX_PIN 2 // Pin 2 as TX
//#define RT400_UART_RX_PIN 3 // Pin 3 as RX

//*****************************************************************************
//                         Global Variables
//*****************************************************************************
static UART2_Handle uartHandle;
//static UART2_Params uartParams;

//*****************************************************************************
//                    Initialization Function
//*****************************************************************************
UART2_Handle InitTerm(void)
{
      UART2_Params uartParams;

      UART2_Params_init(&uartParams);

      uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
      uartParams.baudRate = 115200;

      uartHandle = UART2_open(CONFIG_UART2_0, &uartParams);

      return(uartHandle);
    // Initialize UART parameters
//    UART2_Params_init(&uartParams);
//    uartParams.baudRate = 115200;               // Set baud rate to 115200
//    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
//    uartParams.readMode = UART2_Mode_BLOCKING;  // Use blocking mode for UART
//    uartParams.writeMode = UART2_Mode_BLOCKING; // Use blocking mode for write

    // Manually configure the pins for UART TX and RX
    // Use the actual pin numbers for RT-400, make sure to assign to UART instance
//    PIN_setConfig(RT400_UART_TX_PIN, PIN_GPIO_OUTPUT_EN | PIN_FUNC_MODE);
//    PIN_setConfig(RT400_UART_RX_PIN, PIN_INPUT_EN | PIN_FUNC_MODE);

    // Configure pin modes for UART TX and RX
//       PinModeSet(RT400_UART_TX_PIN, PIN_MODE_2);  // Assuming mode 2 is for UART TX
//       PinModeSet(RT400_UART_RX_PIN, PIN_MODE_3);  // Assuming mode 3 is for UART RX

       // Configure the directions of the pins
//       PinDirModeSet(RT400_UART_TX_PIN, PIN_DIR_MODE_OUT); // TX is output
//       PinDirModeSet(RT400_UART_RX_PIN, PIN_DIR_MODE_IN);  // RX is input

    // Open UART1 (or another available UART instance) for these pins
//    uartHandle = UART2_open(CONFIG_UART2_1, &uartParams); // Manually change CONFIG_UART1 to match your available instance.
//
//    if (uartHandle == NULL) {
//        // Error handling in case UART cannot be opened
//        return NULL;
//    }
//
//    return uartHandle; // Return the initialized UART handle
}

//*****************************************************************************
//                     Formatted String Print to UART
//*****************************************************************************
int Report(const char *pcFormat, ...)
{
    int iRet = 0;
    char *pcBuff;
    char *pcTemp;
    int iSize = 256;
    va_list list;

    pcBuff = (char*)malloc(iSize);
    if (pcBuff == NULL) {
        return (-1);
    }

    while (1) {
        va_start(list, pcFormat);
        iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
        va_end(list);
        if ((iRet > -1) && (iRet < iSize)) {
            break;
        } else {
            iSize *= 2;
            if ((pcTemp = realloc(pcBuff, iSize)) == NULL) {
                Message("Could not reallocate memory\n\r");
                iRet = -1;
                break;
            } else {
                pcBuff = pcTemp;
            }
        }
    }

    Message(pcBuff);  // Print the formatted string to UART
    free(pcBuff);

    return iRet;
}

//*****************************************************************************
//                     Output String to UART
//*****************************************************************************
void Message(const char *str)
{
    #ifdef UART_NONPOLLING
        UART2_write(uartHandle, str, strlen(str), NULL);
    #else
        UART2_write(uartHandle, str, strlen(str), NULL);
    #endif
//    UART2_write(uartHandle, str, strlen(str), NULL);
}

//*****************************************************************************
//                  Clear the UART Console
//*****************************************************************************
void ClearTerm()
{
    Message("\33[2J\r"); // ANSI escape sequence to clear terminal
}

//*****************************************************************************
//                     Read a Character from UART
//*****************************************************************************
char getch(void)
{
    char ch;

       UART2_rxEnable(uartHandle);
       UART2_read(uartHandle, &ch, 1, NULL);
       UART2_rxDisable(uartHandle);

       return(ch);
//    char ch;
//    UART2_read(uartHandle, &ch, 1, NULL); // Read a single character from UART
//    return ch;
}

//*****************************************************************************
//                     Output a Character to UART
//*****************************************************************************
void putch(char ch)
{
    UART2_write(uartHandle, &ch, 1, NULL);  // Write a single character to UART
}

//*****************************************************************************
//                     Get Command from UART
//*****************************************************************************
int TrimSpace(char * pcInput)
{
    size_t size;
    char        *endStr;
    char        *strData = pcInput;
    char index = 0;

    size = strlen(strData);

    if(!size)
    {
        return(0);
    }

    endStr = strData + size - 1;
    while((endStr >= strData) && (IS_SPACE(*endStr)))
    {
        endStr--;
    }
    *(endStr + 1) = '\0';

    while(*strData && IS_SPACE(*strData))
    {
        strData++;
        index++;
    }
    memmove(pcInput, strData, strlen(strData) + 1);

    return(strlen(pcInput));
}
//*******************************************************************************************************
int GetCmd(char *pcBuffer, unsigned int uiBufLen)
{
    char cChar;
       int iLen = 0;

       UART2_rxEnable(uartHandle);
       UART2_read(uartHandle, &cChar, 1, NULL);
       UART2_rxDisable(uartHandle);

       iLen = 0;

       //
       // Checking the end of Command
       //
       while(1)
       {
           //
           // Handling overflow of buffer
           //
           if(iLen >= uiBufLen)
           {
               return(-1);
           }

           //
           // Copying Data from UART into a buffer
           //
           if((cChar == '\r') || (cChar == '\n'))
           {
               UART2_write(uartHandle, &cChar, 1, NULL);
               break;
           }
           else if(cChar == '\b')
           {
               //
               // Deleting last character when you hit backspace
               //
               char ch;

               UART2_write(uartHandle, &cChar, 1, NULL);
               ch = ' ';
               UART2_write(uartHandle, &ch, 1, NULL);
               if(iLen)
               {
                   UART2_write(uartHandle, &cChar, 1, NULL);
                   iLen--;
               }
               else
               {
                   ch = '\a';
                   UART2_write(uartHandle, &ch, 1, NULL);
               }
           }
           else
           {
               //
               // Echo the received character
               //
               UART2_write(uartHandle, &cChar, 1, NULL);

               *(pcBuffer + iLen) = cChar;
               iLen++;
           }
           UART2_rxEnable(uartHandle);
           UART2_read(uartHandle, &cChar, 1, NULL);
           UART2_rxDisable(uartHandle);
       }

       *(pcBuffer + iLen) = '\0';

       return(iLen);
//    char cChar;
//    int iLen = 0;
//
//    while (1) {
//        // Read data into buffer until newline or carriage return is found
//        if (iLen >= uiBufLen) {
//            return -1; // Buffer overflow
//        }
//
//        UART2_read(uartHandle, &cChar, 1, NULL); // Read one character
//
//        if ((cChar == '\r') || (cChar == '\n')) {
//            putch(cChar);  // Echo character to UART
//            break;         // Exit on newline or carriage return
//        } else if (cChar == '\b') { // Handle backspace
//            if (iLen > 0) {
//                putch(cChar);
//                iLen--;
//            }
//        } else {
//            putch(cChar);  // Echo the received character
//            pcBuffer[iLen++] = cChar;
//        }
//    }
//
//    pcBuffer[iLen] = '\0';  // Null-terminate the string
//    return iLen;            // Return the length of the received string
}
