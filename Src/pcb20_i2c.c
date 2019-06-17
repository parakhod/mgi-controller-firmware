#include "stm32f4xx_hal.h"
#include "pca9685.h"


static I2C_HandleTypeDef hi2c_i;
static I2C_HandleTypeDef hi2c_e;

static void I2C_I_Init(void);
static void I2C_E_Init(void);

static uint8_t i2cTxBuffer[32];
static uint8_t i2cRxBuffer[32];

                                // ADT7410 - TempSensor
#define ADT7410_ADDRESS 0x48  

static int ADT7410_Init(void); 
static void ADT7410_Request(void);         

static float Temperature=-999.9;
static float newTemperature;
static float diff;

static int TestXX;



#define ADG2128_ADDRESS 0x70  

#define MX_GND 0
#define MX_12V 1
#define MX_ADC_1k 2
#define MX_ADC_50k 3
#define MX_CAN_L 4
#define MX_CAN_H 5
#define MX_LIN 6
#define MX_CLUSTER 7

#define CPIN_01 0x28 //X5
#define CPIN_02 0x40 //X6
#define CPIN_03 0x20 //X4
#define CPIN_04 0x48 //X7
#define CPIN_05 0x18 //X3
#define CPIN_06 0x50 //X8
#define CPIN_07 0x10 //X2
#define CPIN_08 0x58 //X9
#define CPIN_09 0x08 //X1
#define CPIN_10 0x60 //X10
#define CPIN_11 0x00 //X0
#define CPIN_12 0x68 //X11




static int ADG2128_Init(int addr);

static void ADG2128_ON (int addr, uint8_t XY);
static void ADG2128_OFF(int addr, uint8_t XY);
static void ADG2128_RESET(int addr);

#define EEPROM_ADDRESS 0x50 

static int EEPROM_ReadB(uint16_t addr);
static int EEPROM_ReadW(uint16_t addr);
static float EEPROM_ReadF(uint16_t addr);
static void EEPROM_WriteB(uint16_t addr, uint8_t data);
static void EEPROM_WriteW(uint16_t addr, uint16_t data);
static void EEPROM_WriteF(uint16_t addr, float fdata);


static HAL_StatusTypeDef init_pca9685(unsigned char address);
static void send_pca9685(uint16_t addr, uint8_t led_addr, uint16_t lev);
static void LedDrvSet(uint8_t ch, uint16_t lev);

static uint8_t init_LED_boards(uint8_t number);
static void update_LED_boards();

static int ActiveLEDs;

static void LedDrvSetNow(uint8_t number, uint16_t brightness);
static void LedDrvSetSmooth(uint8_t number, uint16_t brightness, uint16_t time);

static uint16_t LedPrev[32];
static uint16_t LedCurrent[32];
static uint16_t LedGoal[32];
static uint32_t LedInitTime[32];
static uint32_t LedFinTime[32];

static const uint16_t logtable[] = {
0,1,2,3,4,5,6,7,8,9,10,11,13,16,20,24,29,34,39,45,52,59,67,76,85,95,106,117,129,
142,156,170,186,202,219,237,255,275,296,317,340,363,388,414,440,468,497,527,558,
590,623,657,693,730,768,807,847,889,932,977,1022,1069,1117,1167,1218,1271,1324,
1380,1436,1494,1554,1615,1678,1742,1807,1874,1943,2013,2085,2159,2234,2310,2389,
2469,2550,2634,2719,2806,2894,2984,3076,3170,3266,3363,3462,3563,3666,3771,3878,3986,4096  
};




void I2C_I_Init(void)                                                                   // I2C-internal init function 
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  
  
  hi2c_i.Instance = I2C3;
  hi2c_i.Init.ClockSpeed = 100000;
  hi2c_i.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c_i.Init.OwnAddress1 = 0;
  hi2c_i.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c_i.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c_i.Init.OwnAddress2 = 0;
  hi2c_i.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c_i.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED; 
  
  
  __I2C3_CLK_ENABLE();
  
  HAL_I2C_Init(&hi2c_i);
  

}

void I2C_E_Init(void)                                                                   // I2C-external init function 
{

  

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  
  hi2c_e.Instance = I2C1;
  hi2c_e.Init.ClockSpeed = 100000;
  hi2c_e.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c_e.Init.OwnAddress1 = 0;
  hi2c_e.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c_e.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c_e.Init.OwnAddress2 = 0;
  hi2c_e.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c_e.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED; 
  
  __I2C1_CLK_ENABLE();
  
  HAL_I2C_Init(&hi2c_e);
  __HAL_I2C_CLEAR_FLAG(&hi2c_e, I2C_FLAG_BUSY);
  

}





int ADT7410_Init(void)                                                                 // ADT7410 INIT
{  
  i2cTxBuffer[0] = 0x03;
  i2cTxBuffer[1] = 0x80;
  return (HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)ADT7410_ADDRESS<<1, (uint8_t*)i2cTxBuffer, 2, 10)==HAL_OK);
  HAL_Delay(1);
}  

void ADT7410_Request(void)                                                                 // ADT7410 REQUEST
{  
  i2cTxBuffer[0] = 0x00;
  i2cTxBuffer[1] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)ADT7410_ADDRESS<<1, (uint8_t*)i2cTxBuffer, 1, 1);
  //HAL_Delay(1);
  HAL_I2C_Master_Receive(&hi2c_i, (uint16_t)ADT7410_ADDRESS<<1, (uint8_t *)i2cRxBuffer, 2, 1);
  newTemperature = (i2cRxBuffer[0] * 0x100 + i2cRxBuffer[1]) /128.0;
  diff = newTemperature-Temperature;
  if (diff<0) {diff=-diff;}  
  if (diff>0.05) {Temperature = newTemperature;}

  
}

int ADG2128_Init(int addr)
{
  ADG2128_RESET(addr);
  i2cTxBuffer[0] = 0x34;
  i2cTxBuffer[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)(ADG2128_ADDRESS+addr)<<1, (uint8_t*)i2cTxBuffer, 2, 1);
  HAL_Delay(1);
  return (HAL_I2C_Master_Receive(&hi2c_i, (uint16_t)(ADG2128_ADDRESS+addr)<<1, (uint8_t *)i2cRxBuffer, 2, 10)==HAL_OK);

}

void ADG2128_ON(int addr, uint8_t XY)
{
  i2cTxBuffer[0] = 0x80 + XY;
  i2cTxBuffer[1] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((ADG2128_ADDRESS+addr)<<1), (uint8_t*)i2cTxBuffer, 2, 1);
  
}

void ADG2128_OFF(int addr, uint8_t XY)
{
  i2cTxBuffer[0] = 0x00 +  XY;
  i2cTxBuffer[1] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((ADG2128_ADDRESS+addr)<<1), (uint8_t*)i2cTxBuffer, 2, 1);
}


void ADG2128_RESET(int addr)
{
 for (int i=0; i<0x80; i++)
 {ADG2128_OFF(addr,i);}
  
}

int EEPROM_ReadB(uint16_t addr)
{
  i2cTxBuffer[0] = addr >> 8;
  i2cTxBuffer[1] = addr & 0xFF;

  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((EEPROM_ADDRESS)<<1), (uint8_t*)i2cTxBuffer,2,1);
  HAL_I2C_Master_Receive(&hi2c_i, (uint16_t)(EEPROM_ADDRESS)<<1, (uint8_t *)i2cRxBuffer, 1, 1);
  return(i2cRxBuffer[0]);
 // return 0xFF;
}

int EEPROM_ReadW(uint16_t addr)
{
  i2cTxBuffer[0] = addr >> 8;
  i2cTxBuffer[1] = addr & 0xFF;

  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((EEPROM_ADDRESS)<<1), (uint8_t*)i2cTxBuffer,2,1);
  HAL_I2C_Master_Receive(&hi2c_i, (uint16_t)(EEPROM_ADDRESS)<<1, (uint8_t *)i2cRxBuffer, 2, 1);
  return(i2cRxBuffer[0]*0x100 + i2cRxBuffer[1]); 
 // return 0xFFFF;
}

void EEPROM_WriteB(uint16_t addr, uint8_t data)
{
  i2cTxBuffer[0] = addr >> 8;
  i2cTxBuffer[1] = addr & 0xFF;
  i2cTxBuffer[2] = data;
  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((EEPROM_ADDRESS)<<1), (uint8_t*)i2cTxBuffer,3,1);

}

void EEPROM_WriteW(uint16_t addr, uint16_t data)
{
  i2cTxBuffer[0] = addr >> 8;
  i2cTxBuffer[1] = addr & 0xFF;
  i2cTxBuffer[2] = data >> 8;
  i2cTxBuffer[3] = data & 0xFF;
  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((EEPROM_ADDRESS)<<1), (uint8_t*)i2cTxBuffer,4,1);

}

float EEPROM_ReadF(uint16_t addr)
{
  union {
    float a;
    uint8_t b[4];
  } data;
  
  i2cTxBuffer[0] = addr >> 8;
  i2cTxBuffer[1] = addr & 0xFF;

  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((EEPROM_ADDRESS)<<1), (uint8_t*)i2cTxBuffer,2,1);
  HAL_I2C_Master_Receive(&hi2c_i, (uint16_t)(EEPROM_ADDRESS)<<1, (uint8_t *)data.b, 4, 1);
  
  return(data.a);

}

void EEPROM_WriteF(uint16_t addr, float fdata)
{
   union {
    float a;
    uint8_t b[4];
  } data;
  data.a = fdata;
  
  i2cTxBuffer[0] = addr >> 8;
  i2cTxBuffer[1] = addr & 0xFF;
  i2cTxBuffer[2] = data.b[0];
  i2cTxBuffer[3] = data.b[1];
  i2cTxBuffer[4] = data.b[2];
  i2cTxBuffer[5] = data.b[3];
  HAL_I2C_Master_Transmit(&hi2c_i, (uint16_t)((EEPROM_ADDRESS)<<1), (uint8_t*)i2cTxBuffer,6,1);

}




uint8_t init_LED_boards(uint8_t number)
{
int ct=0;
  for (int i=0; i<number; i++)
{
 if (init_pca9685(0x40 + i*2)==HAL_OK)
 {
   init_pca9685(0x41 + i*2);
   ct++;
 }
  
  
}
return ct;
}


HAL_StatusTypeDef init_pca9685(unsigned char address)
{
    HAL_StatusTypeDef result;
    i2cTxBuffer[0] = PCA9685_MODE1;
    i2cTxBuffer[1] = PCA9685_AI;
   // i2cTxBuffer[2] = PCA9685_OUTDRV;
    i2cTxBuffer[2] = PCA9685_INVRT;
    

    

    result = HAL_I2C_Master_Transmit(&hi2c_e, (uint16_t)((address)<<1), (uint8_t*)i2cTxBuffer,3,50);

   
    
    if (result==HAL_OK) 
    {
       HAL_Delay(1);  
      i2cTxBuffer[0] = PCA9685_MODE1;
      i2cTxBuffer[1] = 0x31; // Setting mode to sleep so we can change teh default PWM frequency 
      HAL_I2C_Master_Transmit(&hi2c_e, (uint16_t)((address)<<1), (uint8_t*)i2cTxBuffer,2,10);    
    
      
      HAL_Delay(1); 
      
      i2cTxBuffer[0] = 0xfe;// PWM frequency PRE_SCALE address 
      i2cTxBuffer[1] = 0x04; 
      HAL_I2C_Master_Transmit(&hi2c_e, (uint16_t)((address)<<1), (uint8_t*)i2cTxBuffer,2,10); 
      HAL_Delay(1);  
      
      i2cTxBuffer[0] = PCA9685_MODE1;
      i2cTxBuffer[1] = 0xa1;
      HAL_I2C_Master_Transmit(&hi2c_e, (uint16_t)((address)<<1), (uint8_t*)i2cTxBuffer,2,10); 
      HAL_Delay(1);
      
      i2cTxBuffer[0] = PCA9685_MODE2;
      i2cTxBuffer[1] = PCA9685_INVRT;
      HAL_I2C_Master_Transmit(&hi2c_e, (uint16_t)((address)<<1), (uint8_t*)i2cTxBuffer,2,10); 
      HAL_Delay(1);      

      i2cTxBuffer[0] = 0xfa;
      i2cTxBuffer[2] = 0; 
      i2cTxBuffer[4] = PCA9685_LED_OFF;
      HAL_I2C_Master_Transmit(&hi2c_e, (uint16_t)((address)<<1), (uint8_t*)i2cTxBuffer,5,10);     
    }
    HAL_Delay(5);    
    return result;
}
static void LedDrvSet(uint8_t ch, uint16_t lev)
{
    uint8_t led = 0;
    uint16_t addr=0x41; 
      
    if (ch != 0) {
    
    if ((ch>0)&&(ch<=8))
    {
      addr=0x41; 
      led = ch-1;
    }
    
    if ((ch>8)&&(ch<=16))
    {
      addr=0x40; 
      led = ch-9;
    }
    
    if ((ch>16)&&(ch<=24))
    {
      addr=0x41; 
      led = 32 - ch;
    }
    
    if ((ch>24)&&(ch<=32))
    {
      addr=0x40; 
      led = 40 - ch;
    }
    
    
   send_pca9685(addr, led*4+6, lev);
  
    } else
    {
    send_pca9685(0x40, 0xfa, lev);
    send_pca9685(0x41, 0xfa, lev);   
    for (int i=0; i<32; i++) 
     LedCurrent[i] = LedGoal[i] = 0;

    }
  
}

void send_pca9685(uint16_t addr, uint8_t led_addr, uint16_t lev)
{
    unsigned int off;
    i2cTxBuffer[0] = led_addr;
  
    if (lev == 0) {
    i2cTxBuffer[2] = 0; 
    i2cTxBuffer[4] = PCA9685_LED_OFF;
    } else if (lev >= 4096) {
    i2cTxBuffer[2] = PCA9685_LED_ON;
    i2cTxBuffer[4] = 0;
    } else {
      
    i2cTxBuffer[1] = 0; 
    i2cTxBuffer[2] = 0;  
    i2cTxBuffer[3] = lev & 0xff;//2
    i2cTxBuffer[4] = (lev >> 8) & 0xf;//3
    }
    
    HAL_I2C_Master_Transmit(&hi2c_e, (uint16_t)((addr)<<1), (uint8_t*)i2cTxBuffer,5,3);


}


void LedDrvSetNow(uint8_t number, uint16_t brightness)
{
  if (number > 0) 
  {
      LedPrev[number-1] = LedCurrent[number-1];
    
      LedGoal[number-1] = logtable[brightness];
      LedCurrent[number-1] =  logtable[brightness];    
  }
    LedDrvSet(number,logtable[brightness]);


}

void LedDrvSetSmooth(uint8_t number, uint16_t brightness, uint16_t time)
{
    if (number > 0) 
  {
    LedPrev[number-1] = LedCurrent[number-1];    
    LedGoal[number-1] = logtable[brightness];       
    LedInitTime[number-1] = HAL_GetTick();
    LedFinTime[number-1]  = HAL_GetTick() + time;
    
  //  LedDrvSet(number,logtable[brightness]);
  }
}

void update_LED_boards()
{
  ActiveLEDs = 0;
  uint32_t time;
  
  time = HAL_GetTick();
 
  for (int i=0; i<32; i++) 
  {
    if (LedCurrent[i]>0) {ActiveLEDs++;}
    
    if (LedGoal[i] != LedCurrent[i])
    {
      if (time>LedFinTime[i])
      {
       LedCurrent[i] =  LedGoal[i];         
      } else
      {
        if (LedGoal[i] > LedPrev[i])  LedCurrent[i] = LedPrev[i] +  ((LedGoal[i] - LedPrev[i])* (time - LedInitTime[i]))/(LedFinTime[i] - LedInitTime[i]);      
        else LedCurrent[i] = LedPrev[i] - ((LedPrev[i] - LedGoal[i])* (time - LedInitTime[i]))/(LedFinTime[i] - LedInitTime[i]);   
      }
    LedDrvSet(i+1,LedCurrent[i]);
    }
      
  }
  
  
  
}