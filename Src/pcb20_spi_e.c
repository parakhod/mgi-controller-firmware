#include "stm32f4xx_hal.h"
#include "AD7193.h"    // AD7193 definitions.
#include "pcb20_defines.c"

static SPI_HandleTypeDef SpiEHandle;

static HAL_StatusTypeDef SPI_E_Init(void);   


static int AD7193_Cycle(void);

static int AD7193_Init(void);


/*! Writes data into a register. */
static void AD7193_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber);

/*! Reads the value of a register. */
static unsigned long AD7193_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber);

/*! Resets the device. */
static void AD7193_Reset(void);

/*! Set device to idle or power-down. */
static void AD7193_SetPower(unsigned char pwrMode);

/*! Selects the channel to be enabled. */
static void AD7193_ChannelSelect(unsigned short channel);

/*! Performs the given calibration*/
static void AD7193_Calibrate(unsigned char mode);

/*! Selects the polarity of the conversion and the ADC input range. */
static void AD7193_RangeSetup(unsigned char polarity, unsigned char range);

/*! Returns the result of a single conversion. */
static unsigned long AD7193_SingleConversion(void);

/*! Returns the average of several conversion results. */
static unsigned long AD7193_ContinuousReadAvg(unsigned char sampleNumber);

/*! Read data from temperature sensor and converts it to Celsius degrees. */
static float AD7193_TemperatureRead(void);

static int AD7193_WaitRDY();

/*! Converts 24-bit raw data to volts. */
static float AD7193_ConvertToVolts(unsigned long rawData, float vRef);

static unsigned long AD7193_ReadData(void);

static int AD7193_Ready(void);
static void AD7193_LED(uint8_t channel);
static void AD7193_SetMode();


static unsigned char currentPolarity = 0;
static unsigned char currentGain     = 1;

static int AD7193_CycleStatus = Cell_Reset;
static int AD7193_CurrentChannel = 0;

static unsigned char AD7193_Gain[4];
static unsigned char AD7193_Polarity[4];

static float AD7193_Zero[4];
static float AD7193_Scale[4];

static uint32_t Time_Reset = 0;

HAL_StatusTypeDef SPI_E_Init(void)                                                                   // SPI2 Init function
{
  
 
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  GPIO_InitStruct.Pin       = GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  
  SpiEHandle.Instance               = SPI2;
  
  SpiEHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  SpiEHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiEHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  SpiEHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiEHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiEHandle.Init.CRCPolynomial     = 7;
  SpiEHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiEHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiEHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiEHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
  SpiEHandle.Init.Mode = SPI_MODE_MASTER;
  
   __SPI2_CLK_ENABLE();

   HAL_StatusTypeDef result = HAL_SPI_Init(&SpiEHandle);
   return result;
    
}

int AD7193_Init(void)
{
    int          status = 0;
    unsigned char regVal = 0;
    
   // AD7193_Reset();
   // HAL_Delay(500);
    
    regVal = AD7193_GetRegisterValue(AD7193_REG_ID, 1);
    if((regVal & AD7193_ID_MASK) != ID_AD7193)
    {
       status = -1;
    }
    else 
    {
      
   // AD7193_ChannelSelect(AD7193_CH_1);
  //  AD7193_RangeSetup(0, AD7193_CONF_GAIN_128);
     
   /* unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
    oldRegValue  = AD7193_GetRegisterValue(AD7193_REG_CONF, 3);
    newRegValue  = oldRegValue | AD7193_CONF_CHOP;   
    AD7193_SetRegisterValue(AD7193_REG_CONF, newRegValue, 3);*/
     
     
     
     
      
      
    unsigned long command = 0x0;
    
    command = AD7193_MODE_SEL(AD7193_MODE_CONT) |
              AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2) |
              AD7193_MODE_RATE(0xFF); 
   
    AD7193_SetRegisterValue(AD7193_REG_MODE, command, 3);
    
   /* AD7193_Calibrate(AD7193_MODE_CAL_SYS_ZERO, AD7193_CH_1);
    while (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_SET) {} 
    */
   // AD7193_Calibrate(AD7193_MODE_CAL_SYS_FULL, AD7193_CH_1);
    // while (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_SET){}
    }
    return status;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue   - Data value to write.
 * @param bytesNumber     - Number of bytes to be written.
 * @param modifyCS        - Allows Chip Select to be modified.
 *
 * @return none.
*******************************************************************************/
void AD7193_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber)
{
    unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;
    
    writeCommand[0] = AD7193_COMM_WRITE |
                      AD7193_COMM_ADDR(registerAddress);
    while(bytesNr > 0)
    {
        writeCommand[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }
    //SPI_Write(AD7193_SLAVE_ID * modifyCS, writeCommand, bytesNumber + 1);
    HAL_SPI_Transmit(&SpiEHandle,writeCommand,bytesNumber + 1,1); 
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber     - Number of bytes that will be read.
 * @param modifyCS        - Allows Chip Select to be modified.
 *
 * @return buffer         - Value of the register.
*******************************************************************************/
unsigned long AD7193_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber)
{
    unsigned char registerWord[5] = {0, 0, 0, 0, 0}; 
    unsigned long buffer          = 0x0;
    unsigned char i               = 0;
    
    registerWord[0] = AD7193_COMM_READ |
                      AD7193_COMM_ADDR(registerAddress);
    //SPI_Read(AD7193_SLAVE_ID * modifyCS, registerWord, bytesNumber + 1);
    HAL_SPI_Transmit(&SpiEHandle,registerWord,1,1);
    registerWord[0] = 0;
    registerWord[1] = 0;
    registerWord[2] = 0;
    registerWord[3] = 0;
    registerWord[4] = 0;
    
    
    HAL_SPI_Receive(&SpiEHandle,registerWord,bytesNumber,1);
    for(i = 0; i < bytesNumber ; i++) 
    {
        buffer = (buffer << 8) + registerWord[i];
    }
    
    return buffer;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return none.
*******************************************************************************/
void AD7193_Reset(void)
{
    unsigned char registerWord[6] = {0, 0, 0, 0, 0, 0};
    
    registerWord[0] = 0xFF;
    registerWord[1] = 0xFF;
    registerWord[2] = 0xFF;
    registerWord[3] = 0xFF;
    registerWord[4] = 0xFF;
    registerWord[5] = 0xFF;
    //SPI_Write(AD7193_SLAVE_ID, registerWord, 6);
    HAL_SPI_Transmit(&SpiEHandle,registerWord,6,1); 
}

/***************************************************************************//**
 * @brief Set device to idle or power-down.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
 *
 * @return none.
*******************************************************************************/
void AD7193_SetPower(unsigned char pwrMode)
{
     unsigned long oldPwrMode = 0x0;
     unsigned long newPwrMode = 0x0; 
 
     oldPwrMode  = AD7193_GetRegisterValue(AD7193_REG_MODE, 3);
     oldPwrMode &= ~(AD7193_MODE_SEL(0x7));
     newPwrMode  = oldPwrMode | 
                   AD7193_MODE_SEL((pwrMode * (AD7193_MODE_IDLE)) |
                                  (!pwrMode * (AD7193_MODE_PWRDN)));
     AD7193_SetRegisterValue(AD7193_REG_MODE, newPwrMode, 3);  
}


/***************************************************************************//**
 * @brief Selects the channel to be enabled.
 *
 * @param channel - Selects a channel.
 *                  Example: AD7193_CH_0 - AIN1(+) - AIN2(-);  (Pseudo = 0)
 *                           AD7193_CH_1 - AIN3(+) - AIN4(-);  (Pseudo = 0)
 *                           AD7193_TEMP - Temperature sensor
 *                           AD7193_SHORT - AIN2(+) - AIN2(-); (Pseudo = 0)
 *  
 * @return none.
*******************************************************************************/
void AD7193_ChannelSelect(unsigned short channel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
    oldRegValue  = AD7193_GetRegisterValue(AD7193_REG_CONF, 3);
    oldRegValue &= ~(AD7193_CONF_CHAN(0x3FF));
    newRegValue  = oldRegValue | AD7193_CONF_CHAN(1 << channel);   
    AD7193_SetRegisterValue(AD7193_REG_CONF, newRegValue, 3);
    
    
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode    - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7193_Calibrate(unsigned char mode)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    //AD7193_ChannelSelect(channel);
    oldRegValue  = AD7193_GetRegisterValue(AD7193_REG_MODE, 3);
    oldRegValue &= ~AD7193_MODE_SEL(0x7);
    newRegValue  = oldRegValue | AD7193_MODE_SEL(mode);

    AD7193_SetRegisterValue(AD7193_REG_MODE, newRegValue, 3); // CS is not modified.


}

/***************************************************************************//**
 * @brief Selects the polarity of the conversion and the ADC input range.
 *
 * @param polarity - Polarity select bit. 
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
* @param range     - Gain select bits. These bits are written by the user to select 
                     the ADC input range.     
 *
 * @return none.
*******************************************************************************/
void AD7193_RangeSetup(unsigned char polarity, unsigned char range)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    oldRegValue  = AD7193_GetRegisterValue(AD7193_REG_CONF,3);
    oldRegValue &= ~(AD7193_CONF_UNIPOLAR |
                     AD7193_CONF_GAIN(0x7));
    newRegValue  = oldRegValue | 
                  (polarity * AD7193_CONF_UNIPOLAR) |
                   AD7193_CONF_GAIN(range); 
    AD7193_SetRegisterValue(AD7193_REG_CONF, newRegValue, 3);
    /* Store the last settings regarding polarity and gain. */
    currentPolarity = polarity;
    currentGain     = 1 << range;
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned long AD7193_SingleConversion(void)
{
    unsigned long command = 0x0;
    unsigned long regData = 0x0;
 
    command = AD7193_MODE_SEL(AD7193_MODE_SINGLE) |
              AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2) |
              AD7193_MODE_RATE(0x0FF);    
   
    AD7193_SetRegisterValue(AD7193_REG_MODE, command, 3);
    regData = AD7193_GetRegisterValue(AD7193_REG_DATA, 3);
  
    
    return regData;
}

unsigned long AD7193_ReadData(void)
{
    unsigned long regData = 0x0;
    regData = AD7193_GetRegisterValue(AD7193_REG_DATA, 3);
  
    
    return regData;
}


/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned long AD7193_ContinuousReadAvg(unsigned char sampleNumber)
{
    unsigned long samplesAverage = 0;
    unsigned long command        = 0;
    unsigned char count          = 0;
        
    command = AD7193_MODE_SEL(AD7193_MODE_CONT) | 
              AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2) |
              AD7193_MODE_RATE(0x060);

    AD7193_SetRegisterValue(AD7193_REG_MODE, command, 3);
    for(count = 0; count < sampleNumber; count++)
    {
        samplesAverage += AD7193_GetRegisterValue(AD7193_REG_DATA, 3); 
    }

    samplesAverage = samplesAverage / sampleNumber;
    
    return samplesAverage;
}

/***************************************************************************//**
 * @brief Read data from temperature sensor and converts it to Celsius degrees.
 *
 * @return temperature - Celsius degrees.
*******************************************************************************/
float AD7193_TemperatureRead(void)
{
    unsigned long dataReg     = 0;
    float temperature = 0;    
    
    AD7193_RangeSetup(0, AD7193_CONF_GAIN_1); // Bipolar operation, 0 Gain.
    AD7193_ChannelSelect(AD7193_CH_TEMP);
    dataReg      = AD7193_SingleConversion();
    dataReg     -= 0x800000;
    temperature  = (float) dataReg / 2815;   // Kelvin Temperature
    temperature -= 273;                      // Celsius Temperature
    
    return temperature;
}

/***************************************************************************//**
 * @brief Converts 24-bit raw data to milivolts.
 *
 * @param rawData  - 24-bit data sample.
 * @param vRef     - The value of the voltage reference used by the device.
 *
 * @return voltage - The result of the conversion expressed as volts.
*******************************************************************************/
float AD7193_ConvertToVolts(unsigned long rawData, float vRef)
{
    float voltage = 0;
    
    if(currentPolarity == 0 )   // Bipolar mode
    {
        voltage = 1000 * (((float)rawData / (1ul << 23)) - 1) * vRef / currentGain;
    }
    else                        // Unipolar mode
    {
        voltage = 1000 * ((float)rawData * vRef) / (1ul << 24) / currentGain;
    }
    
    return voltage;
}

int AD7193_Cycle(void)
{
  
int AD_StNext = AD7193_CycleStatus;  
  
if (HAL_GetTick() - Time_Reset > 500 ){ 
  
    
switch ( AD7193_CycleStatus ) 
 {
   
  case Cell_Reset:
   AD7193_Reset();
   AD_StNext = Cell_Init;  
   Time_Reset = HAL_GetTick();
   break;
   
  case Cell_Init: 
   if (AD7193_Init() == 0) 
   {
    AD_StNext = Cell_Prst0_0;  
   }  
   else
   {
    AD_StNext = Cell_Fail; 
   }
  break;
  case Cell_MsgReady:     
   AD_StNext = Cell_Ready;   
   AD7193_LED(0);
   AD7193_ChannelSelect(AD7193_CH_0);
   AD7193_SetMode();

   break;

  case Cell_Prst0_0:                              //init ch 0
   AD7193_LED(1);
   AD7193_ChannelSelect(AD7193_CH_0);
   AD7193_RangeSetup(0, AD7193_Gain[0]);
   AD_StNext = Cell_Prst0_1; 
   break;
  
  case Cell_Prst0_1:                             
   AD7193_Calibrate(AD7193_MODE_CAL_INT_ZERO);
   AD_StNext = Cell_Prst0_2; 
   break; 
   
  case Cell_Prst0_2:           
    if (AD7193_Ready()) 
    { 
     AD7193_Calibrate(AD7193_MODE_CAL_INT_FULL);
     AD_StNext = Cell_Prst1_0; 
    }
   break;    
   
  case Cell_Prst1_0:                              //init ch 1
    if (AD7193_Ready()) 
    { 
     AD7193_LED(2);
     AD7193_ChannelSelect(AD7193_CH_1);
     AD7193_RangeSetup(0, AD7193_Gain[1]);   
     AD_StNext = Cell_Prst1_1; 
    }
   break;
  
  case Cell_Prst1_1:                             
   AD7193_Calibrate(AD7193_MODE_CAL_INT_ZERO);
   AD_StNext = Cell_Prst1_2; 
   break; 
   
  case Cell_Prst1_2:
    if (AD7193_Ready()) 
    { 
     AD7193_Calibrate(AD7193_MODE_CAL_INT_FULL);
     AD_StNext = Cell_Prst2_0; 
    }
   break;       
   
  case Cell_Prst2_0:                              //init ch 2
    if (AD7193_Ready()) 
    { 
     AD7193_LED(3); 
     AD7193_ChannelSelect(AD7193_CH_2);
     AD7193_RangeSetup(0, AD7193_Gain[2]);
     AD_StNext = Cell_Prst2_1; 
    }
   break;
  
  case Cell_Prst2_1:  
     AD7193_Calibrate(AD7193_MODE_CAL_INT_ZERO);
     AD_StNext = Cell_Prst2_2; 
   break; 
   
  case Cell_Prst2_2:   
    if (AD7193_Ready()) 
    {   
     AD7193_Calibrate(AD7193_MODE_CAL_INT_FULL);
     AD_StNext = Cell_Prst3_0; 
    }
   break;          
   
   case Cell_Prst3_0:                              //init ch 3
    if (AD7193_Ready()) 
    { 
     AD7193_LED(4);  
     AD7193_ChannelSelect(AD7193_CH_3);
     AD7193_RangeSetup(0, AD7193_Gain[3]);
     AD_StNext = Cell_Prst3_1; 
    }
   break;
  
  case Cell_Prst3_1:                             
   AD7193_Calibrate(AD7193_MODE_CAL_INT_ZERO);
   AD_StNext = Cell_Prst3_2; 
   break; 
   
  case Cell_Prst3_2:   
    if (AD7193_Ready()) 
    {  
     AD7193_Calibrate(AD7193_MODE_CAL_INT_FULL);
     AD7193_WaitRDY();
     AD_StNext = Cell_MsgReady; 
    }
   break;        
   
  case Cell_SelCh1:
   AD7193_LED(1);  
   AD7193_ChannelSelect(AD7193_CH_0);
   AD7193_SetMode();
   AD_StNext = Cell_ContRead; 
   break;    
   
  case Cell_SelCh2:
   AD7193_LED(2);   
   AD7193_ChannelSelect(AD7193_CH_1);
   AD7193_SetMode();
   AD_StNext = Cell_ContRead; 
   break;   
   
  case Cell_SelCh3:
   AD7193_LED(3);   
   AD7193_ChannelSelect(AD7193_CH_2);
   AD7193_SetMode();
   AD_StNext = Cell_ContRead; 
   break;   
   
  case Cell_SelCh4:
   AD7193_LED(4);   
   AD7193_ChannelSelect(AD7193_CH_3);
   AD7193_SetMode();
   AD_StNext = Cell_ContRead; 
   break;   
   
  } 
   
 }
AD7193_CycleStatus = AD_StNext;
return AD7193_CycleStatus;

}

int AD7193_Ready(void) 
{
  return (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET);
}

void AD7193_LED(uint8_t channel)
{
  uint8_t regval = 0;
  
  switch (channel)
  {
  case 1:
    regval = 0x32;
    break;
  case 2:
    regval = 0x31;
    break;   
  case 3:
    regval = 0x34;
    break;
  case 4:
    regval = 0x38;
    break;    
  case 10:
    regval = 0x3F;
    break;    
    
  }
  
AD7193_SetRegisterValue(AD7193_REG_GPOCON,regval,1);
  //2 = ch 1
  //1 = ch 2
  //4 = ch 3
  //8 = ch 4
  
}

int AD7193_WaitRDY()
{
uint32_t Timer = HAL_GetTick(); 
  
 while ((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_SET) && (HAL_GetTick()-Timer < 1000) )
     {}

return HAL_GetTick()-Timer < 1000;
}

void AD7193_SetMode()
{
   unsigned long command = 0x0;
    
   command = AD7193_MODE_SEL(AD7193_MODE_CONT) |
              AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2) |
              AD7193_MODE_RATE(0x3FF); 
   
    AD7193_SetRegisterValue(AD7193_REG_MODE, command, 3);
}
