// PD2 - CS# ; PC10 - SPI2SCK ; PC12 - SPI2MOSI
// PD4 C/D#
#include "stm32f4xx_hal.h"
#include "font.c"

static void SPI_Init(void);    
static void OLED_Init();
static void OLED_CLS();
static void dly_oled(void);

static void SPI9_Transmit(uint8_t *pData, uint16_t Size, int dc);
static void OLED_OUTROW(int row);
static void OLED_Separator(int style,int line);
static void OLED_Print(char* buf,int line);
static void OLED_fPrint(char* buf,char* mask,int line);
static void OLED_Gauge(int begin, int end, int min, int max, char label, int line, int skin);
static void OLED_Bright(unsigned char bright);
static void OLED_ON(void);
static void OLED_RND(void);
static void OLED_OFF(void);
static void OLED_USB_Update(void);

static unsigned char str_buffer[128];
static unsigned char screen_buffer[128] [8];
static unsigned char COM_buffer[8][128];

static SPI_HandleTypeDef SpiHandle;

static int OledState = 0;
static unsigned char OledBrightness = 0xFF;

extern int ScreenDubMode;
extern void USB_Print(unsigned char* buffer);

void SPI_Init(void)                                                                   // SPI2 Init function
{
  
 // __SPI3_CLK_ENABLE();
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  GPIO_InitStruct.Pin       = GPIO_PIN_10|GPIO_PIN_12;
  //GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
 // GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  //GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;              // SIC!!!1111
  GPIO_InitStruct.Pin = GPIO_PIN_2;   
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 
  
  SpiHandle.Instance               = SPI3;
  
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
  SpiHandle.Init.Mode = SPI_MODE_MASTER;

 // HAL_SPI_Init(&SpiHandle);
    
}

void OLED_Init()
{
  
static const unsigned char init_seq[]= {
        0xAE, //DISPLAYOFF
        0xD5,0x80, /* clock divide ratio (0x00=1) and oscillator frequency (0x8) */
        0xAD,0x00,
        0xA8,0x3F,
        0xD3,0x00,
        0xD4, 190, //?
        0xD0, 2,
        0x40,/* start line */
        0x8D,0x14,/* [2] charge pump setting (p62): 0x014 enable, 0x010 disable */
        
        0xa1,/* segment remap a0/a1*/
        0xc8,/* c0: scan dir normal, c8: reverse */
        0xDA,0x12,/* com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5) */
        0x81,0xFF,/* [2] set contrast control */
        0xD9,82,//0x22,/* [2] pre-charge period 0x022/f1*/
        0xDB,0x20,/* vcomh deselect level */
 
        0xA4,/* output ram to display */
        0xA6,/* none inverted normal display mode */
        //0xAF /* display on */ 
        };

//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);    //digitalWrite(DC, LOW);  
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);    //digitalWrite(CS, LOW);  


//HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&init_seq,22,10);                  //SPI.transfer(c);
SPI9_Transmit((unsigned char*)&init_seq,28,0);

HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);     //digitalWrite(CS, HIGH);  

OLED_CLS();
  
}


void SPI9_Transmit(uint8_t *pData, uint16_t Size, int dc) 
{
 for (int i=0; i<Size; i++) {
  
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
  dly_oled();
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,(dc!=0));
  dly_oled();
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
  dly_oled();
  for (int j=0; j<8; j++) {
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);  
  dly_oled();
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,(((1 << (7-j)) & pData[i]) !=0 )); 
  dly_oled();
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);  
  dly_oled();
  } 
 }
  
  
}

void dly_oled() {
 //for (int i=0; i<10000000; i++) { }
  //HAL_Delay(5);
  
}

void OLED_CLS()
{
   for (int i=0; i <= 127; i++){
     str_buffer[i] = 0x00;
   }
   for (int i=0; i <= 7; i++){
     OLED_OUTROW(i);
      
   }
  if (ScreenDubMode==1){
      USB_Print("OLED: TY='CLS'\r\n");
      }
  
}


void OLED_Separator(int style,int line)
{
   for (int i=0; i <= 127; i++){
     switch(style) {
     case 0:
     str_buffer[i] = 0x04; 
     break;  
     case 1:
       if (i != 87) {str_buffer[i] = 0x04;} else {str_buffer[i] = 0x07;} 
     break;         
       
     }
     
     
     
   }
     
   
    if (ScreenDubMode==1){
      sprintf(COM_buffer[line],"OLED: L=%d; TY='SEP'; SS='%d'\r\n",line, style);
      }
   OLED_OUTROW(line);
  
}



void OLED_OUTROW(int row)
{
  

  int DrFlag = 0;
  
  for (int i=0; i < 128; i++)
  {
    if (str_buffer[i] != screen_buffer[i][row])
    {
      screen_buffer[i][row] = str_buffer[i];
      DrFlag = 1;
    }
  }
  
  if (DrFlag) {
  unsigned char cmd1[]= {0xb0+row,0x00,0x10};
  //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);    //digitalWrite(DC, LOW);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);    //digitalWrite(CS, LOW);  
  //HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&cmd1,3,10);                  //SPI.transfer(c);
  SPI9_Transmit((unsigned char*)&cmd1,2,0);  
 // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);     //digitalWrite(CS, HIGH);  
  

  
  //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);      
 // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);   
  //HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&cmd1,2,10);   
  //HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&str_buffer,128,10);
 // SPI9_Transmit((unsigned char*)&cmd1,2,1);
  SPI9_Transmit((unsigned char*)&str_buffer,128,1);
  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);     

  
 if (ScreenDubMode==1){ USB_Print(COM_buffer[row]);}
  }
  

}
   
void OLED_Bright(unsigned char bright)
{
  unsigned char cmd1[]= {0x81,bright};
  //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);    //digitalWrite(DC, LOW);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);    //digitalWrite(CS, LOW);  
 // HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&cmd1,2,10);                  //SPI.transfer(c);
  SPI9_Transmit((unsigned char*)&cmd1,2,0);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);     //digitalWrite(CS, HIGH);  
  OledBrightness = bright;
} 


void OLED_ON(void)
{
  unsigned char cmd1[]= {0xAF};
  //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);    //digitalWrite(DC, LOW);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);    //digitalWrite(CS, LOW);  
 // HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&cmd1,1,10);                  //SPI.transfer(c);
  SPI9_Transmit((unsigned char*)&cmd1,1,0);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);     //digitalWrite(CS, HIGH);  
  OledState = 1;
  
} 
   
void OLED_OFF(void)
{
  unsigned char cmd1[]= {0xAE};
  //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);    //digitalWrite(DC, LOW);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);    //digitalWrite(CS, LOW);  
 // HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&cmd1,1,10);                  //SPI.transfer(c);
  SPI9_Transmit((unsigned char*)&cmd1,1,0);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);     //digitalWrite(CS, HIGH);  
  OledState = 0;
}


void OLED_RND(void)
{
  unsigned char x =  0+ (rand() % 256);
  if ((x != 0xAE) && (x != 0x8D) && (x >= 0xF0) && (x < 0x100)) {   //A0 !!!!D0!!!!D1br  D2 D4br D9br
  unsigned char cmd1[]= {x, rand() % 256};
  //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);    //digitalWrite(DC, LOW);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);    //digitalWrite(CS, LOW);  
 // HAL_SPI_Transmit(&SpiHandle,(unsigned char*)&cmd1,1,10);                  //SPI.transfer(c);
  SPI9_Transmit((unsigned char*)&cmd1,1,0);  
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);     //digitalWrite(CS, HIGH);  
  }
} 


void OLED_Print(char* buf,int line)
{
  for (int i=0; i<21; i++){
    for (int j=0; j<5; j++){
    str_buffer[i*6+j] = font[buf[i]*5+j];      
    }
  str_buffer[i*6+5] = 0; 
   }
  
  for (int i=125; i<128; i++){
  str_buffer[i] = 0;
  
  }
  
  if (ScreenDubMode==1){
      sprintf(COM_buffer[line],"OLED: L=%d; TY='TEXT'; TX='%s'\r\n",line, buf);
      // USB_Print(COM_buffer[line]);
      }
   OLED_OUTROW(line);
}  

void OLED_fPrint(char* buf,char* mask,int line)
{
  for (int i=0; i<21; i++){
    for (int j=0; j<5; j++){
    str_buffer[i*6+j] = font[buf[i]*5+j];    
    if (mask[i]=='I') {str_buffer[i*6+j] = ~str_buffer[i*6+j]; }
    if (mask[i]=='U') {str_buffer[i*6+j] = str_buffer[i*6+j] ^ 0x80; }
    }
    str_buffer[i*6+5] = 0x00;
    if ((mask[i]=='I') || (mask[i+1]=='I')) {str_buffer[i*6+5] = 0xFF;} 
    if ((mask[i]=='U') || (mask[i+1]=='U')) {str_buffer[i*6+5] = 0x80;}  
   }
  
  for (int i=125; i<128; i++){
  str_buffer[i] = 0;
  
  }
  
  if (ScreenDubMode==1){
      sprintf(COM_buffer[line],"OLED: L=%d; TY='TEXT'; TX='%s'; MS='%s'\r\n",line, buf,mask);
       //USB_Print(TextBuffer);
      }
  
   OLED_OUTROW(line);
}  




void OLED_Gauge(int begin, int end, int min, int max, char label, int line, int skin)
{
 unsigned char c_fi=0x7F;
 unsigned char c_ed=0x7F;
 unsigned char c_em=0x41;

 if (skin==1) {
 c_fi=0x0f;
 c_ed=0x0f;
 c_em=0x08;
 }

  for (int j=0; j<5; j++){
   str_buffer[j] = font[label*5+j];   
  }
  
  int px_b = ((begin-min)*117) / (max-min) + 10;
  int px_e = ((end-min)*117) / (max-min) + 10;
  
  for (int j=5; j<128; j++){
  
    if(j<10) {str_buffer[j] = 0x00;}
    if((j==10)||(j==127)) {str_buffer[j] = c_ed;}   
    if((j>10)&&(j<127)) {if((j>px_b)&&(j<px_e)) {str_buffer[j] = c_fi;} else {str_buffer[j] = c_em;}}  
    if((j==px_b)||(j==px_e)) {str_buffer[j] = c_ed;}   
    //if(j>126) {str_buffer[j] = 0x00;}
  }
  
    if (ScreenDubMode==1){
      sprintf(COM_buffer[line],"OLED: L=%d; TY='GAUG'; SS='%d'; TX='%c'; BG='%d'; EN='%d'; MI='%d'; MX='%d'\r\n",line, skin, label, begin, end, min, max);
       //USB_Print(TextBuffer);
      } 
    OLED_OUTROW(line);   
     
} 

static void OLED_USB_Update(void)
{
   if (ScreenDubMode==1){ 
     
  OLED_CLS();
   
   }
}



