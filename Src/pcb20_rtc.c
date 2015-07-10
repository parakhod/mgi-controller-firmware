#include "stm32f4xx_hal.h"

static RTC_HandleTypeDef RtcHandle;

static void RTC_Init(void);
static void RTC_CalendarReset(void);
static void RTC_CalendarShow(void);

static void RTC_TimeSet(uint8_t hour, uint8_t min, uint8_t sec);
static void RTC_DateSet(uint8_t date, uint8_t month, uint8_t year);

static uint8_t Dec2HD(uint8_t dec);

static void RTC_Init(void)
{

  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
  /* To change the source clock of the RTC feature (LSE, LSI), You have to:
     - Enable the power clock using __PWR_CLK_ENABLE()
     - Enable write access using HAL_PWR_EnableBkUpAccess() function before to 
       configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and 
       __HAL_RCC_BACKUPRESET_RELEASE().
     - Configure the needed RTc clock source */
  
  /*##-1- Configue LSE as RTC clock soucre ###################################*/ 
  __PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
 // __HAL_RCC_BACKUPRESET_FORCE();
 // __HAL_RCC_BACKUPRESET_RELEASE();
  
    
     
     
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        // Connect LSE to RTC
        //__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
        return;
  }
    
  

  
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        //Error_Handler();
        return;
    }
  
  
  
  /*##-2- Enable RTC peripheral Clocks #######################################*/ 
  /* Enable RTC Clock */ 
 __HAL_RCC_RTC_ENABLE(); 
   
  RtcHandle.Instance = RTC; 
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = 0x7F;
  RtcHandle.Init.SynchPrediv = 0x00FF;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  HAL_RTC_Init(&RtcHandle);
   
  
  
   
  /*##-2- Check if Data stored in BackUp register0: No Need to reconfigure RTC#*/
  /* Read the BackUp Register 0 Data */
  if(HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0) != 0x32F2)
  {  
    
    RTC_CalendarReset();
  }
  

  
  }



static void RTC_CalendarReset(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Monday April 14th 2014 */
  sdatestructure.Year = 0x15;
  sdatestructure.Month = RTC_MONTH_JANUARY;
  sdatestructure.Date = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  HAL_RTC_SetDate(&RtcHandle,&sdatestructure,FORMAT_BCD);

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:00:00 */
  stimestructure.Hours = 0x00;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT_24;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  HAL_RTC_SetTime(&RtcHandle,&stimestructure,FORMAT_BCD);

  /*##-3- Writes a data in a RTC Backup data Register0 #######################*/
  HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR0,0x32F2);  
}

static void RTC_TimeSet(uint8_t hour, uint8_t min, uint8_t sec)
{
  RTC_TimeTypeDef stimestructure;
  stimestructure.Hours = Dec2HD(hour);
  stimestructure.Minutes = Dec2HD(min);
  stimestructure.Seconds = Dec2HD(sec);
  stimestructure.TimeFormat = RTC_HOURFORMAT_24;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  HAL_RTC_SetTime(&RtcHandle,&stimestructure,FORMAT_BCD);
  
  HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR0,0x32F2);  
  
}


/**
  * @brief  Display the current time and date.
  * @param  showtime: pointer to buffer
  * @param  showdate: pointer to buffer
  * @retval None
  */
static void RTC_CalendarShow(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  

  ///sprintf((char*)showtime,"%02d:%02d:%02d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  //sprintf((char*)showdate,"%02d-%02d-%02d",sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year);
  
  
  
}

static void RTC_DateSet(uint8_t date, uint8_t month, uint8_t year)
{
  
  RTC_DateTypeDef sdatestructure;
  

  sdatestructure.Year = Dec2HD(year);
  sdatestructure.Month = Dec2HD(month);
  sdatestructure.Date = Dec2HD(date);
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  HAL_RTC_SetDate(&RtcHandle,&sdatestructure,FORMAT_BCD); 
}



static uint8_t Dec2HD(uint8_t dec)
{
 return (dec % 10 ) + (dec / 10)*16;  
}