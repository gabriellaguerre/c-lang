// //Refactoring tcp_server to define and create pins from pinMux
// #include <stddef.h>
// #include <stdint.h>

// #ifndef DeviceFamily_CC3220
// #define DeviceFamily_CC3220
// #endif

// #include <ti/devices/DeviceFamily.h>

// #include "ti_drivers_config.h"
// /*  =============================== UART2 ===============================
// */

// #include <ti/drivers/UART2.h>
// #include <ti/devices/cc32xx/inc/hw_ints.h>
// #include <ti/devices/cc32xx/inc/hw_memmap.h>
// #include <ti/drivers/uart2/UART2CC32XX.h>

// #define CONFIG_UART2_COUNT 2
// #define CONFIG_UART2_0 0  // Define UART2_0 as index 0
// #define CONFIG_UART2_1 1  // Define UART2_1 as index 1

// #define UART0_BASE UARTA0_BASE
// #define UART1_BASE UARTA1_BASE
// #define INT_UART0  INT_UARTA0
// #define INT_UART1  INT_UARTA1

// static unsigned char uart2RxRingBuffer0[32];
// static unsigned char uart2RxRingBuffer1[32];
// /* TX ring buffer allocated to be used for nonblocking mode */
// //static unsigned char uart2TxRingBuffer0[32];
// static unsigned char uart2TxRingBuffer1[32];

// /* UART2 Pin Definitions for GPIO12 (Pin 9) and GPIO13 (Pin 10) */
// #define PIN_09_UART0_TX 9    // GPIO12, Pin 9 for UART0_TX
// #define PIN_10_UART0_RX 10   // GPIO13, Pin 10 for UART0_RX

// #define PIN_46_UART1_TX 46
// #define PIN_47_UART1_RX 47

// UART2CC32XX_Object uart2CC32XXObjects0;

// static const UART2CC32XX_HWAttrs uart2CC32XXHWAttrs0 = {
//     .baseAddr           = UART0_BASE,
//     .intNum             = INT_UART0,
//     .intPriority        = 0x20,
//     .flowControl        = UART2_FLOWCTRL_NONE,
//     .rxDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
//     .txDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
//     // Configure for Pin 9 (TX) and Pin 10 (RX)
//     .rxPin              = PIN_10_UART0_RX,
//     .txPin              = PIN_09_UART0_TX,
//     .ctsPin             = UART2CC32XX_PIN_UNASSIGNED,
//     .rtsPin             = UART2CC32XX_PIN_UNASSIGNED,
//     .rxBufPtr           = uart2RxRingBuffer0,
//     .rxBufSize          = sizeof(uart2RxRingBuffer0),
//     .txBufPtr           = NULL,
//     .txBufSize          = 0,
// };

// UART2CC32XX_Object uart2CC32XXObjects1;

// static const UART2CC32XX_HWAttrs uart2CC32XXHWAttrs1 = {
//     .baseAddr           = UART1_BASE,
//     .intNum             = INT_UART1,
//     .intPriority        = (~0),
//     .flowControl        = UART2_FLOWCTRL_NONE,
//     .rxDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
//     .txDmaChannel       = UART2CC32XX_DMACH_UNASSIGNED,
//     .rxPin              = PIN_46_UART1_RX,
//     .txPin              = PIN_47_UART1_TX,
//     .ctsPin             = UART2CC32XX_PIN_UNASSIGNED,
//     .rtsPin             = UART2CC32XX_PIN_UNASSIGNED,
//     .rxBufPtr           = uart2RxRingBuffer1,
//     .rxBufSize          = sizeof(uart2RxRingBuffer1),
//     .txBufPtr           = uart2TxRingBuffer1,
//     .txBufSize          = sizeof(uart2TxRingBuffer1)
// };

// const UART2_Config UART2_config[CONFIG_UART2_COUNT] = {
//     {   /* CONFIG_UART2_0 */
//         .object      = &uart2CC32XXObjects0,
//         .hwAttrs     = &uart2CC32XXHWAttrs0
//     },
//     {   /* CONFIG_UART2_1 */
//         .object      = &uart2CC32XXObjects1,
//         .hwAttrs     = &uart2CC32XXHWAttrs1
//     },
// };

// const uint_least8_t CONFIG_UART2_0_CONST = CONFIG_UART2_0;
// const uint_least8_t CONFIG_UART2_1_CONST = CONFIG_UART2_1;
// const uint_least8_t UART2_count = CONFIG_UART2_COUNT;

#include <stdbool.h>
#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/drivers/GPIO.h>
#include <ti/devices/cc32xx/driverlib/gpio.h>
#include <ti/devices/cc32xx/driverlib/pin.h>
#include "create_uart0_term.h"

void configureGPIOForUART0(void) {
    // Configure Pin 46 (UART1 TX)
    GPIO_setConfig(UART0_TX_PIN, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // Configure Pin 47 (UART1 RX)
    GPIO_setConfig(UART0_RX_PIN, GPIO_CFG_INPUT | GPIO_CFG_IN_PU);
}
