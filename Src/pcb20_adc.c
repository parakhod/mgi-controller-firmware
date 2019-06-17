//pc0 pc2 pc3 adc1 in 10,12,13

#include "stm32f4xx_hal.h"
#include "pcb20_defines.c"
extern ADC_HandleTypeDef Adc1Handle;
extern ADC_HandleTypeDef Adc2Handle;
extern DMA_HandleTypeDef hdma_adc1;

extern uint32_t ADct;

extern __IO uint16_t ADC1_Buffer[256];

static uint16_t ADC1_Cache[256][4];
static uint8_t  ADC1_Cache_ct = 0;

static uint16_t ADC1_SuperCache[16][4];
static uint8_t  ADC1_SuperCache_ct = 0;

extern uint16_t PedFiltering;

static uint16_t Pedal_1 = 0;
static uint16_t Pedal_2 = 0;
static uint16_t Pedal_3 = 0;

static uint16_t N_Pedal_1;
static uint16_t N_Pedal_2;
static uint16_t N_Pedal_3;


static __IO uint16_t ButtonsR = 0;
static uint16_t Buttons = 0;
static uint32_t BtnCounter = 0;

extern uint16_t Pedal_1_Lo;
extern uint16_t Pedal_1_Hi;
extern uint16_t Pedal_2_Lo;
extern uint16_t Pedal_2_Hi;
extern uint16_t Pedal_3_Lo;
extern uint16_t Pedal_3_Hi;

static void ADC_Init(void);
static void GetPedals(void);
static uint16_t GetRes(int addr, uint8_t X, uint32_t pullup);
static void CheckButtons(void);
static float GetVbat(void);
static float GetMCTemp(void);
static void HandleAdcCycle(void);

extern void BtnLtPress(); 
extern void BtnRtPress(); 
extern void BtnOkPress(); 
extern void BtnMenuPress(); 

extern void OLED_Bright(unsigned char bright);
extern void OLED_ON(void);

extern int OledState;
extern unsigned char OledBrightness;
extern int OledDimCt;

#define PULL_1k   ((uint32_t)0x00000000)
#define PULL_50k  ((uint32_t)0x00000001)


static void ADC_Init(void)
{
   GPIO_InitTypeDef          GPIO_InitStruct;


  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  
  __ADC1_CLK_ENABLE();
  __ADC2_CLK_ENABLE(); 
  __DMA2_CLK_ENABLE();

  Adc1Handle.Instance = ADC1;  
  Adc1Handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
  Adc1Handle.Init.Resolution            = ADC_RESOLUTION12b;
  Adc1Handle.Init.ScanConvMode          = ENABLE;
  Adc1Handle.Init.ContinuousConvMode    = ENABLE;
  Adc1Handle.Init.DiscontinuousConvMode = DISABLE;
  Adc1Handle.Init.NbrOfDiscConversion   = 0;
  Adc1Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  Adc1Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  Adc1Handle.Init.NbrOfConversion       = 4;
  Adc1Handle.Init.DMAContinuousRequests = ENABLE;
  Adc1Handle.Init.EOCSelection          = EOC_SEQ_CONV;
      
  HAL_ADC_Init(&Adc1Handle);
  
  ADC_ChannelConfTypeDef sConfig;
  
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);
  
  

  hdma_adc1.Instance = DMA2_Stream0;    
  hdma_adc1.Init.Channel  = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
  hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_adc1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_adc1.Init.PeriphBurst = DMA_PBURST_SINGLE; 
  
  HAL_DMA_Init(&hdma_adc1);
 __HAL_LINKDMA(&Adc1Handle,DMA_Handle,hdma_adc1);
  

  //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
   // HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
   // HAL_NVIC_EnableIRQ(ADC_IRQn);  
    
  ADC1_Buffer[1] = 5;
  //HAL_ADC_Start(&Adc1Handle);
  HAL_ADC_Start_DMA(&Adc1Handle, (uint32_t*)(&ADC1_Buffer[0]), 8);
  
  
  Adc2Handle.Instance = ADC2;  
  Adc2Handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
  Adc2Handle.Init.Resolution            = ADC_RESOLUTION12b;
  Adc2Handle.Init.ScanConvMode          = DISABLE;
  Adc2Handle.Init.ContinuousConvMode    = DISABLE;
  Adc2Handle.Init.DiscontinuousConvMode = DISABLE;
  Adc2Handle.Init.NbrOfDiscConversion   = 0;
  Adc2Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  Adc2Handle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  Adc2Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  Adc2Handle.Init.NbrOfConversion       = 1;
  Adc2Handle.Init.DMAContinuousRequests = DISABLE;
  Adc2Handle.Init.EOCSelection          = EOC_SINGLE_CONV;
      
  HAL_ADC_Init(&Adc2Handle);
  }

void HandleAdcCycle(void)
{
  //Pedal_1 = ADC1_Buffer[2];
 // Pedal_2 = ADC1_Buffer[4];
  //Pedal_3 = ADC1_Buffer[6];  
  ADC1_Cache[ADC1_Cache_ct][0] = ADC1_Buffer[2];
  ADC1_Cache[ADC1_Cache_ct][1] = ADC1_Buffer[4];
  ADC1_Cache[ADC1_Cache_ct][2] = ADC1_Buffer[6];
  ADC1_Cache_ct++; 
  if (ADC1_Cache_ct >= PedFiltering) {
       
    ADC1_Cache_ct = 0;
    
    uint32_t acc0 = 0;
    uint32_t acc1 = 0;
    uint32_t acc2 = 0;
    if (PedFiltering > 0) {     
        for (int i = 0; i < PedFiltering; i++) 
        {
          acc0 += ADC1_Cache[i][0];
          acc1 += ADC1_Cache[i][1];
          acc2 += ADC1_Cache[i][2];
        }
        
        ADC1_SuperCache[ADC1_SuperCache_ct][0] = acc0 / PedFiltering;
        ADC1_SuperCache[ADC1_SuperCache_ct][1] = acc1 / PedFiltering;
        ADC1_SuperCache[ADC1_SuperCache_ct][2] = acc2 / PedFiltering;
    }
      else
      {
        ADC1_SuperCache[ADC1_SuperCache_ct][0] = ADC1_Buffer[2];
        ADC1_SuperCache[ADC1_SuperCache_ct][1] = ADC1_Buffer[4];
        ADC1_SuperCache[ADC1_SuperCache_ct][2] = ADC1_Buffer[6];
      
      }
    

    
    ADC1_SuperCache_ct++;
    if (ADC1_SuperCache_ct >= 16) {ADC1_SuperCache_ct = 0;  }
    
   
    
  }
  
  
  
}

static void GetPedals(void)
{  
  __IO uint16_t uhADCxConvertedValue = 0;
  
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset = 0;
  
  uint32_t acc0 = 0;
  uint32_t acc1 = 0;
  uint32_t acc2 = 0;
  
  for (int i = 0; i < 16; i++) 
  {
    acc0 += ADC1_SuperCache[i][0];
    acc1 += ADC1_SuperCache[i][1];
    acc2 += ADC1_SuperCache[i][2];
  }
  
  Pedal_1 = acc0 / 16;
  Pedal_2 = acc1 / 16;
  Pedal_3 = acc2 / 16;
  
 /*  sConfig.Channel = ADC_CHANNEL_10;                   //Pedal_1
   HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);   
   HAL_ADC_Start(&Adc1Handle);   
   HAL_ADC_PollForConversion(&Adc1Handle, 10);
   uhADCxConvertedValue = HAL_ADC_GetValue(&Adc1Handle);
   HAL_ADC_Stop(&Adc1Handle);
   Pedal_1 = uhADCxConvertedValue;
   
   
   sConfig.Channel = ADC_CHANNEL_12;                   //Pedal_2
   HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);   
   HAL_ADC_Start(&Adc1Handle);   
   HAL_ADC_PollForConversion(&Adc1Handle, 10);
   uhADCxConvertedValue = HAL_ADC_GetValue(&Adc1Handle);
   HAL_ADC_Stop(&Adc1Handle);
   Pedal_2 = uhADCxConvertedValue;
   
   sConfig.Channel = ADC_CHANNEL_13;                   //Pedal_3
   HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);   
   HAL_ADC_Start(&Adc1Handle);   
   HAL_ADC_PollForConversion(&Adc1Handle, 10);
   uhADCxConvertedValue = HAL_ADC_GetValue(&Adc1Handle);
   HAL_ADC_Stop(&Adc1Handle);
   Pedal_3 = uhADCxConvertedValue;*/
   
   
   
   
   if (Pedal_1 >= Pedal_1_Hi) {N_Pedal_1 = 1024;} else if 
      (Pedal_1 <= Pedal_1_Lo) {N_Pedal_1 = 0;} else
      {N_Pedal_1= ((Pedal_1-Pedal_1_Lo) * 1024) / (Pedal_1_Hi-Pedal_1_Lo);}
   
   if (Pedal_2 >= Pedal_2_Hi) {N_Pedal_2 = 1024;} else if 
      (Pedal_2 <= Pedal_2_Lo) {N_Pedal_2 = 0;} else
      {N_Pedal_2= ((Pedal_2-Pedal_2_Lo) * 1024) / (Pedal_2_Hi-Pedal_2_Lo);}   
   
   if (Pedal_3 >= Pedal_3_Hi) {N_Pedal_3 = 1024;} else if 
      (Pedal_3 <= Pedal_3_Lo) {N_Pedal_3 = 0;} else
      {N_Pedal_3= ((Pedal_3-Pedal_3_Lo) * 1024) / (Pedal_3_Hi-Pedal_3_Lo);}
   
}


void CheckButtons(void)
{
   //__IO uint16_t uhADCxConvertedValue = 0;
   
  ButtonsR = ADC1_Buffer[0];
  
   int BtnValue;
  
  /*  ADC_ChannelConfTypeDef sConfig;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfig.Offset = 0;

   sConfig.Channel = ADC_CHANNEL_8;                   //Buttons
   HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);   
   HAL_ADC_Start(&Adc1Handle);   
   HAL_ADC_PollForConversion(&Adc1Handle, 10);
   uhADCxConvertedValue = HAL_ADC_GetValue(&Adc1Handle);
   HAL_ADC_Stop(&Adc1Handle);
   ButtonsR = uhADCxConvertedValue; */
   
   if (ButtonsR <=250) {BtnValue = 1;}
   if ((ButtonsR > 250 ) && (ButtonsR <=800))  {BtnValue = 2;}
   if ((ButtonsR > 800 ) && (ButtonsR <=1500)) {BtnValue = 3;}
   if ((ButtonsR > 1500 )&& (ButtonsR <=3000)) {BtnValue = 4;}
   if (ButtonsR > 3000 ) {BtnValue = 0;}
   
   if (Buttons != BtnValue) {
     if (BtnCounter > 4) {BtnCounter = 0; }
     else if (BtnCounter > 2) {Buttons = BtnValue;
     switch (Buttons) {
     case 1: BtnMenuPress(); 
     break;
     case 2: BtnOkPress(); 
     break;
     case 3: BtnLtPress(); 
     break;
     case 4: BtnRtPress(); 
     break;  
       
     }

     //OLED Check and on
     if (OledState == 0)
     {
       OLED_ON();
     }
     
     if (OledBrightness < 0xFF)
     {
       OLED_Bright(0xFF);
     }
     OledDimCt = OLED_ON_TIME;
     
     }
       }
   
   BtnCounter++;
}

static uint16_t GetRes(int addr, uint8_t X, uint32_t pullup)
{
  __IO uint16_t uhADCxConvertedValue = 0;
  
  if (pullup == PULL_1k)   {ADG2128_ON(addr, X | 2);}
  if (pullup == PULL_50k)  {ADG2128_ON(addr, X | 3);}
  //HAL_Delay(1);

    ADC_ChannelConfTypeDef sConfig;
   sConfig.Rank = 1;
   sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
   sConfig.Offset = 0;
  

   if (pullup == PULL_1k)  {sConfig.Channel = ADC_CHANNEL_5;} 
   if (pullup == PULL_50k) {sConfig.Channel = ADC_CHANNEL_6;} 

   HAL_ADC_ConfigChannel(&Adc2Handle, &sConfig);   
   HAL_ADC_Start(&Adc2Handle);   
   HAL_ADC_PollForConversion(&Adc2Handle, 10);
   uhADCxConvertedValue = HAL_ADC_GetValue(&Adc2Handle);
   HAL_ADC_Stop(&Adc2Handle);
   
   if (pullup == PULL_1k)   {ADG2128_OFF(addr, X | 2);}
   if (pullup == PULL_50k)  {ADG2128_OFF(addr, X | 3);}
   
   return uhADCxConvertedValue;


}


float GetVbat(void)
{
  __IO uint16_t uhADCxConvertedValue = 0;
  __IO uint32_t ADC1ConvertedVoltage = 0;
  /*
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfig.Offset = 0;

   sConfig.Channel = ADC_CHANNEL_VBAT;                  
  HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);   
   HAL_ADC_Start(&Adc1Handle);   
   HAL_ADC_PollForConversion(&Adc1Handle, 10);
   uhADCxConvertedValue = HAL_ADC_GetValue(&Adc1Handle);
   HAL_ADC_Stop(&Adc1Handle);
   ADC1ConvertedVoltage = (uint32_t)(uhADCxConvertedValue * 2) * 3300 / 0xfff;*/
   return (float)(ADC1ConvertedVoltage / 1000.0);
  

}
                  
float GetMCTemp(void)
{
  __IO uint16_t uhADCxConvertedValue = 0;
  __IO uint32_t ADC1ConvertedVoltage = 0;
  __IO float Vsense = 0;
  __IO float TempX = 0;
  /*
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;

   sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;     
   
  
   HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);   
   for (int i=0; i<100; i++) {
   HAL_ADC_Start(&Adc1Handle);   
   HAL_ADC_PollForConversion(&Adc1Handle, 10);
   uhADCxConvertedValue = HAL_ADC_GetValue(&Adc1Handle);
   HAL_ADC_Stop(&Adc1Handle);
   ADC1ConvertedVoltage = (uint32_t)(uhADCxConvertedValue * 3300 / 0xfff);
   __IO float Vsense = (float)(ADC1ConvertedVoltage / 1000.0);
   TempX += ((Vsense - 0.760) / .0025) + 25.0 ;
   }
   */
   return TempX / 100.0;



}
      

