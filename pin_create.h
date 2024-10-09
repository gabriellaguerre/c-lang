#ifndef RS485_SERVER_H
#define RS485_SERVER_H

#include <ti/drivers/net/wifi/simplelink.h>

// Function to setup the TCP server
int setupTCPServer(void);

// RS485 server task function declaration
void *rs485ServerTask(void *pvParameter);


#endif // RS485_SERVER_H
