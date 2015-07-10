#include "stm32f4xx_hal.h"

static void System_Init(void);
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void GPIO_Update(void);

static void LED_ON(int led);
static void LED_OFF(int led);
static void LED_SW(int led);

static void GPIO_ClusterPowerOn(void);
static void GPIO_ClusterPowerOff(void);




static int CC_GPIO[10];

static int CTRL_SB;
static int CTRL_BR;
static int CTRL_IG;
static int CTRL_ST;

extern int CTRL_SB_INV;
extern int CTRL_BR_INV;
extern int CTRL_IG_INV;
extern int CTRL_ST_INV;

extern int CTRL_SB_PIN;
extern int CTRL_BR_PIN;
extern int CTRL_IG_PIN;
extern int CTRL_ST_PIN;

static int Updated = 0;

static void System_Init(void)
{
 /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
   
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
}



/** System Clock Configuration */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  
  
}



void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();

  __GPIOH_CLK_ENABLE();
  
     

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 
    
  
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_9|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); 
  
  
  
  

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  
  
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  
  

  LED_OFF(0);
  LED_OFF(1);
  LED_OFF(2);
  GPIO_ClusterPowerOff(); 
  
}


void GPIO_Update(void)
{
 int CTRL_SB_T;
 int CTRL_BR_T;
 int CTRL_IG_T;
 int CTRL_ST_T; 
 
 CC_GPIO[0] = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9) == GPIO_PIN_RESET; 
 CC_GPIO[1] = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10) == GPIO_PIN_RESET; 
 CC_GPIO[2] = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11) == GPIO_PIN_RESET; 
 CC_GPIO[3] = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12) == GPIO_PIN_RESET; 
 CC_GPIO[4] = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13) == GPIO_PIN_RESET; 
 CC_GPIO[5] = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14) == GPIO_PIN_RESET; 
 CC_GPIO[6] = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15) == GPIO_PIN_RESET; 
 CC_GPIO[7] = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == GPIO_PIN_RESET; 
 CC_GPIO[8] = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == GPIO_PIN_RESET; 
 CC_GPIO[9] = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == GPIO_PIN_RESET; 

 CTRL_SB_T = CC_GPIO[CTRL_SB_PIN] ^ CTRL_SB_INV;
 CTRL_BR_T = CC_GPIO[CTRL_BR_PIN] ^ CTRL_BR_INV;
 CTRL_IG_T = CC_GPIO[CTRL_IG_PIN] ^ CTRL_IG_INV;
 CTRL_ST_T = CC_GPIO[CTRL_ST_PIN] ^ CTRL_ST_INV;
 
 //put anti-bounce here
 if (CTRL_SB != CTRL_SB_T) {CTRL_SB = CTRL_SB_T;}
 if (CTRL_BR != CTRL_BR_T) {CTRL_BR = CTRL_BR_T;}
 if (CTRL_IG != CTRL_IG_T) {CTRL_IG = CTRL_IG_T;}
 if (CTRL_ST != CTRL_ST_T) {CTRL_ST = CTRL_ST_T;}
 
}



void LED_OFF(int led)
{
  switch(led) {
  case 0:  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
           break; 
  case 1:  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
           break; 
  case 2:  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
           break;    
    
  }
}


static void LED_ON(int led)
{
  switch(led) {
  case 0:  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
           break; 
  case 1:  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
           break; 
  case 2:  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
           break;      
    
  }
}

static void LED_SW(int led)
{
  switch(led) {
  case 0:  HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9);
           break; 
  case 1:  HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_13);
           break; 
  case 2:  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
           break;    
    
  }
}


static void GPIO_ClusterPowerOn(void)
{
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
  
}

static void GPIO_ClusterPowerOff(void)
{
   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
  
}

