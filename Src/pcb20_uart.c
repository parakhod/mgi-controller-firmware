#include "stm32f4xx_hal.h"

static UART_HandleTypeDef rs485;
static void RS485_Init(void);


/* RS 485 init function */
static void RS485_Init(void)
{

  rs485.Instance = USART2;
  rs485.Init.BaudRate = 115200;
  rs485.Init.WordLength = UART_WORDLENGTH_8B;
  rs485.Init.StopBits = UART_STOPBITS_1;
  rs485.Init.Parity = UART_PARITY_NONE;
  rs485.Init.Mode = UART_MODE_TX_RX;
  rs485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  rs485.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&rs485);

}

 // HAL_UART_Transmit(&rs485, (unsigned char *)aTxBuffer, strlen(aTxBuffer), 5000);