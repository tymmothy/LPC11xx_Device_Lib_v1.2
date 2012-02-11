#include "LPC11xx.h"
#include "LPC11xx_syscon.h"
#include "LPC11xx_gpio.h"
#include "LPC11xx_iocon.h"
#include "LPC11xx_uart.h"

volatile uint32_t ms_since_boot;


void SysTick_Handler()
{
    ms_since_boot++;
}

void UART_Handler()
{
   uint16_t tmp;

   tmp = UART_PendingIT(UART);
   if (tmp == UART_IT_RDA) {
       tmp = UART_Recv(UART);
       UART_Send(UART, '!');
       UART_Send(UART, tmp);
   }
}

uint32_t millis()
{
   uint32_t ms_val;


   ms_val = ms_since_boot;

   return ms_val;
}


void delay(uint32_t ms)
{
    uint32_t start_ms = millis();

    while (millis() - start_ms < ms);
}


int main()
{
    //uint32_t tmp;


    NVIC_DisableIRQ(UART_IRQn);


    IOCON_SetPinConfig(IOCON_PinConfig_1_7_TXD, IOCON_Mode_Normal);
    IOCON_SetPinConfig(IOCON_PinConfig_1_6_RXD, IOCON_Mode_Normal);

    SYSCON_AHBClockLineControl(SYSCON_AHBClockLine_UART, Enable);
    SYSCON_AHBClockLineControl(SYSCON_AHBClockLine_GPIO, Enable);
    SYSCON_SetUARTClockDivider(1);


    UART_SetDivisor(UART, 26);
    UART_SetWordLength(UART, UART_WordLength_8b);
    UART_SetStopBits(UART, UART_StopBits_1);
    UART_SetParity(UART, UART_Parity_No);

    UART_EnableFifos(UART);
    UART_EnableTx(UART);
    //UART_EnableRX(UART);

    UART_FlushTxFifo(UART);
    UART_FlushRxFifo(UART);
    UART_EnableTx(UART);
    //UART_EnableIT(UART, UART_IT_RDA);

    GPIO_SetPinDirections(GPIO0, GPIO_Pin_5, GPIO_Direction_Out);
    GPIO_SetPinDirections(GPIO0, GPIO_Pin_4, GPIO_Direction_Out);
    GPIO_WritePins(GPIO0, GPIO_Pin_5, GPIO_Pin_5);
    GPIO_WritePins(GPIO0, GPIO_Pin_4, GPIO_Pin_4);

    SysTick_Config(SystemFrequency / 1000);

    while(1) {
        static int pinval = 0;

        if (UART_GetLineStatus(UART) & UART_RDR) {
            int a = UART_Recv(UART);
            if (a == ' ') {
                UART_Send(UART, '!');
            }
        }

        GPIO_WritePins(GPIO0, GPIO_Pin_5, pinval);
        GPIO_WritePins(GPIO0, GPIO_Pin_4, pinval);
        pinval ^= GPIO_Pin_5 | GPIO_Pin_4;

        delay(500);
    }

    return 0;
}
