// Standard includes
#include <stddef.h>  // for NULL
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>  // Add this to use bool

#include "uart_term.h"
#include "create_uart1_term.h"

extern int vsnprintf(char * s,
                     size_t n,
                     const char * format,
                     va_list arg);

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
#define IS_SPACE(x)       (x == 32 ? 1 : 0)

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************
UART2_Handle uartHandle;

//*****************************************************************************
//
//! Initialization
//!
//! This function
//!        1. Configures the UART to be used.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
UART2_Handle InitTerm(void)
{
//    UART2_Params uartParams;
//
//    UART2_Params_init(&uartParams);
//
//    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
//    uartParams.baudRate = 115200;
//
//    uartHandle = UART2_open(CONFIG_UART2_0, &uartParams);
//
//    return(uartHandle);
        UART2_Params uartParams;

        // Initialize the UART driver
//        UART_init();

        // Initialize UART parameters
        UART2_Params_init(&uartParams);
        uartParams.readMode = UART2_Mode_BLOCKING;
        uartParams.writeMode = UART2_Mode_BLOCKING;
        uartParams.baudRate = 115200;
        uartParams.dataLength = UART2_DataLen_8;
        uartParams.stopBits = UART2_StopBits_1;
        uartParams.parityType = UART2_Parity_NONE;
        uartParams.readReturnMode = UART2_ReadReturnMode_FULL;


        // Open UART1 with the configured parameters
         uartHandle = UART2_open(1, &uartParams);
         if(uartHandle == NULL) {
             //Handle error
         }
         return uartHandle;
}

//*****************************************************************************
//
//! prints the formatted string on to the console
//!
//! \param[in]  format  - is a pointer to the character string specifying the
//!                       format in the following arguments need to be
//!                       interpreted.
//! \param[in]  [variable number of] arguments according to the format in the
//!             first parameters
//!
//! \return count of characters printed
//
//*****************************************************************************
int Report(const char *pcFormat,
           ...)
{
    int iRet = 0;
    char        *pcBuff;
    char        *pcTemp;
    int iSize = 256;
    va_list list;

    pcBuff = (char*)malloc(iSize);
    if(pcBuff == NULL)
    {
        return(-1);
    }
    while(1)
    {
        va_start(list,pcFormat);
        iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
        va_end(list);
        if((iRet > -1) && (iRet < iSize))
        {
            break;
        }
        else
        {
            iSize *= 2;
            if((pcTemp = realloc(pcBuff, iSize)) == NULL)
            {
                Message("Could not reallocate memory\n\r");
                iRet = -1;
                break;
            }
            else
            {
                pcBuff = pcTemp;
            }
        }
    }
    Message(pcBuff);
    free(pcBuff);

    return(iRet);
}

//*****************************************************************************
//
//! Trim the spaces from left and right end of given string
//!
//! \param  pcInput - string on which trimming happens
//!
//! \return length of trimmed string
//
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

//*****************************************************************************
//
//! Get the Command string from UART
//!
//! \param[in]  pucBuffer   - is the command store to which command will be
//!                           populated
//! \param[in]  ucBufLen    - is the length of buffer store available
//!
//! \return Length of the bytes received. -1 if buffer length exceeded.
//!
//*****************************************************************************
int GetCmd(char *pcBuffer,
           unsigned int uiBufLen)
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
}

//*****************************************************************************
//
//! Outputs a character string to the console
//!
//! This function
//!        1. prints the input string character by character on to the console.
//!
//! \param[in]  str - is the pointer to the string to be printed
//!
//! \return none
//!
//! \note If UART_NONPOLLING defined in than Message or UART write should be
//!       called in task/thread context only.
//
//*****************************************************************************
void Message(const char *str)
{
#ifdef UART_NONPOLLING
    UART2_write(uartHandle, str, strlen(str), NULL);
#else
    UART2_write(uartHandle, str, strlen(str), NULL);
#endif
}

//*****************************************************************************
//
//! Clear the console window
//!
//! This function
//!        1. clears the console window.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
void ClearTerm()
{
    Message("\33[2J\r");
}

//*****************************************************************************
//
//! Read a character from the console
//!
//! \param none
//!
//! \return Character
//
//*****************************************************************************
char getch(void)
{
    char ch;

    UART2_rxEnable(uartHandle);
    UART2_read(uartHandle, &ch, 1, NULL);
    UART2_rxDisable(uartHandle);

    return(ch);
}

//*****************************************************************************
//
//! Outputs a character to the console
//!
//! \param[in]  char    - A character to be printed
//!
//! \return none
//
//*****************************************************************************
void putch(char ch)
{
    UART2_write(uartHandle, &ch, 1, NULL);
}
