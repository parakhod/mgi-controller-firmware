/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pcb20_defines.c"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>


static uint8_t TVDS;
static uint8_t TVHS;
static uint32_t TVRR;

static uint32_t ADct = 0;

float CellVoltage;
extern float Temperature;

static int ClusterType;
static int ClusterAutonomy;

static int SimulatorType;
static int PedType;

static int LCellEN;
static int LEDDriverEN;

static int OledInitEN;
static int OledDimmerEN;

static int ButtonsEN;

static int SpecMode=0;
static int ScreenDubMode=0;

static char TextBuffer[256];
static char TmpBuffer[256];

static __IO uint16_t ADC1_Buffer[256];

char RCVBuffer[256];
uint32_t RCVpos=0;

char COM_In_Buffer[256];
uint32_t COM_In_Length=0;
uint32_t COM_In_Flag=0;

static int ADG2128_Inited[4];
static int ADT7410_Inited;

USBD_HandleTypeDef  USBD_Device;
PCD_HandleTypeDef hpcd;
ADC_HandleTypeDef Adc1Handle;
ADC_HandleTypeDef Adc2Handle;
DMA_HandleTypeDef hdma_adc1;

static uint32_t CT_10ms;
static uint32_t CT_20ms;
static uint32_t CT_50ms;
static uint32_t CT_100ms;
static uint32_t CT_250ms;
static uint32_t CT_500ms;
static uint32_t CT_1000ms;
static uint32_t CT_10000ms;

static void Evt_10ms(void);
static void Evt_20ms(void);
static void Evt_50ms(void);
static void Evt_100ms(void);
static void Evt_250ms(void);
static void Evt_500ms(void);
static void Evt_1000ms(void);
static void Evt_10000ms(void);

static void ParseCom(void);

uint16_t USB_SN;

static int d_rpm = 0;        //RPM
static int d_speed = 0;      //Speed in km/h
static int d_fuel = 50;      //Fuel level (%)
static int d_temp = 0;       //Temperature
static int d_park_dst = 15;  //Park distance 0-15
static int d_odo = 0;      

int d_cluster_pwr=1;

int d_high_beam=0;  //indicator LEDs
int d_low_beam=0;
int d_low_beam_lev=0;
int d_front_fog=0;
int d_rear_fog=0;
int d_left=0;
int d_right=0;
int d_emerg=0;
int d_brake=0;
int d_steering_e=0;
int d_backlight_c=1;
int d_backlight_b=0;
int d_liq_fl_e=0;
int d_fault=0;
int d_br_pads_e=0;
uint8_t d_doors=0;
int d_lfd=0;
int d_rfd=0;
int d_lrd=0;
int d_rrd=0;
int d_boot=0;
int d_bonnet=0;
int d_fuel_need=0;
int d_lock=0;
int d_lock_err=0;
int d_fuel_cut=0;
int d_city=0;
int d_eco=0;
int d_ab2_fl=0;
int d_ab2=0;
int d_ab1_fl=0;
int d_ab_fail=0;
int d_b_fl_1=1;
int d_b_fl_2=1;
int d_batt=1;
int d_batt_fl=1;
int d_ss_c=0;
int d_ss_u=0;
int d_ss_ind=0;
int d_check=0;
int d_check_fl=0;
int d_overheat=0;
int d_oil=0;
int d_oil_fl=0;
int d_ch_oil=0;
uint8_t d_asr=0;
int d_esc=0;
int d_ebd_f=0;
int d_abs_u=0;
int d_esc_u=0;
int d_shift_up=0;
int d_shift_dn=0;
int d_park=0;
int d_tco_perf=0;
int d_hb=0;
int d_gear=0;
int d_adblue=100;
int d_deccel=0;
int d_eb=0;
int d_btn_up=0;
int d_btn_esc=0;
int d_btn_page=0;
int d_btn_menu=0;
int d_btn_minus=0;
int d_btn_plus=0;
int d_btn_down=0;
uint32_t d_trid=0;
uint32_t d_totd=0;
uint32_t d_hours=0;
uint8_t d_press_1=100;
uint8_t d_press_2=100;
uint8_t d_cruise_lcd=0;
uint8_t d_cruise_led=0;
uint8_t d_cruise_pto=0;
uint8_t d_cruise_speed=0;
uint8_t d_starter=0;
uint8_t d_mirror=0;
uint8_t d_beam_pos=0;
uint8_t d_preheat=0;
uint8_t d_speed_limit_led=0;
uint8_t d_speed_limit_value=0;
uint8_t d_speed_limit_set=0;
uint8_t d_cab_tilted=0;
uint8_t d_oil_press_low=0;
uint8_t d_oil_temp_high=0;
uint8_t d_tlr_lcd=0;
uint8_t d_tlr_led=0;
uint8_t d_br_err_1=0;
uint8_t d_br_err_2=0;
uint8_t d_br_err_3=0;
uint8_t d_tr_el_con=0;
uint8_t d_etc7_mode=0x4C;
uint8_t d_airsusp=0;
uint8_t d_susp_lcm=0;
uint8_t d_susp_lift=0;
uint8_t d_1pto=0;
uint8_t d_2pto=0;
uint8_t d_3pto=0;
uint8_t d_esp=0;
uint8_t d_no_abs=0;
uint8_t d_no_asr=0;
uint8_t d_warning_1=0;
uint8_t d_warning_2=0;
uint8_t d_lock_front=0;
uint8_t d_lock_center=0;
uint8_t d_lock_rear=0;
uint8_t d_oil_low=0;
uint8_t d_oil_high=0;
uint8_t d_coolant_low=0;
uint8_t d_br_fluid_low=0;
uint8_t d_st_fluid_low=0;
uint8_t d_ws_fluid_low=0;
uint8_t d_steer_error=0;
uint8_t d_air_filter=0;
uint8_t d_fuel_filter=0;
uint8_t d_oil_filter=0;
uint8_t d_water_tank=0;
uint8_t d_oil_too_low=0;
uint8_t d_coolant_too_low=0;
uint8_t d_oil_level=128;
uint8_t d_oil_pressure=128;
uint16_t d_ext_temp=9536;
uint8_t d_edc_warning=0;
uint8_t d_edc_error=0;

uint8_t d_tt[9];



//EEPROM





static uint8_t CtrlTypes[4];// = {CT_VW_LEV_L,CT_VW_LIGHT,CT_VW_LEV_R,0};

static int TurnDelay;// =  0;
static int TurnPeriod;// = 14;
static int TurnON;// =     5;

static int RtcEN;
static int TempEN;

static uint32_t btnStartMask = 0x0F;

static int x_left=0;
static int x_right=0;
static int x_pos_light=0;
static int x_high_beam=0;
static int x_low_beam=0;
static int x_front_fog=0;
static int x_rear_fog=0;


static int x_wiper_f=0;
static int x_washer_f=0;
static int x_wiper_r=0;
static int x_washer_r=0;

static int x_cruise=0;
static int x_cc_set=0;
static int x_cc_res=0;

static int x_interval=-1;
static int x_reset=0;
static int x_up=0;
static int x_dn=0;

static int G_motor_key = 0;
static int G_wiper_key = 0;

static int G_motor_key_ct = 0;
static int G_wiper_key_ct = 0;

static int G_motor_value = 0;
static int G_motor_value_prev = 0;

static int G_wiper_value = 0;
static int G_wiper_value_prev = 0;

static int x_blink=0;

static int LED_blink_1;

static int USB_started = 0;

int blink = 0;
int DirBlink = 0;
int DirBlinkCt = 0;
uint32_t RedLedTm = 0;
int RedLed = 0;

static int LED_Boards = 0;
static int CELL_Board = -1;

static int eeprom_ID;

static int CTRL_SB_INV;
static int CTRL_BR_INV;
static int CTRL_IG_INV;
static int CTRL_ST_INV;

static int CTRL_SB_PIN;
static int CTRL_BR_PIN;
static int CTRL_IG_PIN;
static int CTRL_ST_PIN;

static uint16_t Pedal_1_Lo;
static uint16_t Pedal_1_Hi;
static uint16_t Pedal_2_Lo;
static uint16_t Pedal_2_Hi;
static uint16_t Pedal_3_Lo;
static uint16_t Pedal_3_Hi;

extern uint16_t N_Pedal_1;
extern uint16_t N_Pedal_2;
extern uint16_t N_Pedal_3;

int tval1 = 0;
int tval2 = 0;
int tval3 = 0;
int tval4 = 0;

int dw[6];
int dw_before[6];
int dw_ch[6];

static uint16_t PedFiltering = 255;

unsigned char bright = 0;

static uint32_t VWIntns[4]={165,310,565,980}; //load-50k !
static uint32_t WhBtns[6]={2830,400,1100,1630,2200,2600};
static uint32_t OnOff[2]={2831,360};

static uint32_t FIPl[2]={1000,360};
static uint32_t FIDir[3]={350,1750,2500}; 
static uint32_t FIWs[4]={2600, 1800, 1010, 350};
static uint32_t FIWash[3]={2600,2090,350};

static uint32_t MBuffer[12][12];
static uint32_t MBuffer_old[12][12];

extern unsigned char COM_buffer[8][128];

static int PollR(int addr, uint8_t X, uint32_t pullup, uint32_t* values, int number);
static int CmpStr(const char *s1, const char *s2, const char delimiter);
static void TestMatrix(int addr);
static void PresetControls(int addr, int type);
static void PollControls(int addr, int type);
static void Blinker(void);
static void RedLedBlast(uint32_t length);

static void USB_Print(unsigned char* buffer);
extern void OLED_USB_Update(void);

extern void BtnLtPress(); 
extern void BtnRtPress(); 
extern void BtnOkPress(); 
extern void BtnMenuPress(); 


extern void LedDrvSetNow(uint8_t number, uint16_t brightness);
extern void LedDrvSetSmooth(uint8_t number, uint16_t brightness, uint16_t time);

static float Vbat;

static int AD_CalCt = 0;
static float AD_CalAcc = 0;
static float AD_CalVal = 1;



static int OledDimCt = OLED_ON_TIME;

/* Private function prototypes -----------------------------------------------*/


  int main(void)
{

  System_Init(); //SysClock & GPIO

  LED_ON(0);
    

  I2C_I_Init();  //Internal I2C
       
     //  if(ADT7410_Inited) {ADT7410_Request();}
   SPI_Init();
     
       
       
       
       ADG2128_Inited[3] = ADG2128_Init(3);       
       ADG2128_Inited[2] = ADG2128_Init(2);
       ADG2128_Inited[1] = ADG2128_Init(1);
       ADG2128_Inited[0] = ADG2128_Init(0);
  
  HAL_Delay(10);
  eeprom_ID = EEPROM_ReadW(EE_SN);
  
  ClusterType =     EEPROM_ReadB(EE_ClusterType);
  if (ClusterType == 0xFF) {ClusterType == 0;}
  
  ClusterAutonomy = EEPROM_ReadB(EE_ClusterAutonomy);
  if (ClusterAutonomy == 0xFF) {ClusterAutonomy == 0;}
  
  CtrlTypes[0] = EEPROM_ReadB(EE_CtrlTypes0);
  CtrlTypes[1] = EEPROM_ReadB(EE_CtrlTypes1);
  CtrlTypes[2] = EEPROM_ReadB(EE_CtrlTypes2);
  CtrlTypes[3] = EEPROM_ReadB(EE_CtrlTypes3);
  
  TurnDelay =   EEPROM_ReadB(EE_TurnDelay);
  if (TurnDelay == 0xFF) {TurnDelay =0;}
  TurnPeriod =  EEPROM_ReadB(EE_TurnPeriod);
  if (TurnPeriod == 0xFF) {TurnPeriod = 14;}
  TurnON =  EEPROM_ReadB(EE_TurnON);
  if (TurnON == 0xFF) {TurnON = 5;}

  CTRL_SB_INV = EEPROM_ReadB(EE_CTRL_SB_INV);
  if (CTRL_SB_INV == 0xFF) {CTRL_SB_INV =0;}
  
  CTRL_BR_INV = EEPROM_ReadB(EE_CTRL_BR_INV);
  if (CTRL_BR_INV == 0xFF) {CTRL_BR_INV =0;}
  
  CTRL_IG_INV = EEPROM_ReadB(EE_CTRL_IG_INV);
  if (CTRL_IG_INV == 0xFF) {CTRL_IG_INV =0;}
  
  CTRL_ST_INV = EEPROM_ReadB(EE_CTRL_ST_INV);
  if (CTRL_ST_INV == 0xFF) {CTRL_ST_INV =1;}

  
  CTRL_SB_PIN = EEPROM_ReadB(EE_CTRL_SB_PIN);
  if (CTRL_SB_PIN == 0xFF) {CTRL_SB_PIN =0;}
  
  CTRL_BR_PIN = EEPROM_ReadB(EE_CTRL_BR_PIN);
  if (CTRL_BR_PIN == 0xFF) {CTRL_BR_PIN =1;} 
  
  CTRL_IG_PIN = EEPROM_ReadB(EE_CTRL_IG_PIN);
  if (CTRL_IG_PIN == 0xFF) {CTRL_IG_PIN =2;}
  
  CTRL_ST_PIN = EEPROM_ReadB(EE_CTRL_ST_PIN);
  if (CTRL_ST_PIN == 0xFF) {CTRL_ST_PIN =3;}  
  
  
  SimulatorType = EEPROM_ReadB(EE_SimulatorType);
  if (SimulatorType == 0xFF) {SimulatorType = 0;} 
  
  PedType = EEPROM_ReadB(EE_PedType);
  if (PedType == 0xFF) {PedType = 0;} 
  
  PedFiltering = EEPROM_ReadB(EE_PedFiltering);
  
  Pedal_1_Lo = EEPROM_ReadW(EE_Pedal_1_Lo);
  if (Pedal_1_Lo  > 4096) {Pedal_1_Lo  = 0;} 
  
  Pedal_1_Hi = EEPROM_ReadW(EE_Pedal_1_Hi);
  if (Pedal_1_Hi  > 4096) {Pedal_1_Hi  = 4096;} 
  
  Pedal_2_Lo = EEPROM_ReadW(EE_Pedal_2_Lo);
  if (Pedal_2_Lo  > 4096) {Pedal_2_Lo  = 0;} 
  
  Pedal_2_Hi = EEPROM_ReadW(EE_Pedal_2_Hi);
  if (Pedal_2_Hi  > 4096) {Pedal_2_Hi  = 4096;}  
  
  Pedal_3_Lo = EEPROM_ReadW(EE_Pedal_3_Lo);
  if (Pedal_3_Lo  > 4096) {Pedal_3_Lo  = 0;} 
  
  Pedal_3_Hi = EEPROM_ReadW(EE_Pedal_3_Hi);
  if (Pedal_3_Hi  > 4096) {Pedal_3_Hi  = 4096;} 
  
  
  RtcEN = EEPROM_ReadB(EE_RtcEN);
  if (RtcEN == 0xFF) {RtcEN = 0;} 
  
  TempEN = EEPROM_ReadB(EE_TempEN);
  if (TempEN == 0xFF) {TempEN = 0;} 
  
  LCellEN = EEPROM_ReadB(EE_LCellEN);
  if (LCellEN == 0xFF) {LCellEN = 0;} 
  
  LEDDriverEN = EEPROM_ReadB(EE_LEDDriverEN);
  if (LEDDriverEN == 0xFF) {LEDDriverEN = 0;} 
  
  OledInitEN = EEPROM_ReadB(EE_OledInitEN);
  if (OledInitEN == 0xFF) {OledInitEN = 1;} 
  
  OledDimmerEN = EEPROM_ReadB(EE_OledDimmerEN);
  if (OledDimmerEN == 0xFF) {OledDimmerEN = 0;} 
  
  ButtonsEN = EEPROM_ReadB(EE_ButtonsEN);
  if (ButtonsEN == 0xFF) {ButtonsEN = 1;} 
  
  OLED_Init(); 
   
  ADC_Init();    
  
  HAL_Delay(5); 
  
  Vbat = GetVbat();
  

  
  if (TempEN) {  
     ADT7410_Inited = ADT7410_Init();  //Temperature sensor
      }
     else
     {
       Temperature =  GetMCTemp();
     }
 if (RtcEN) {  
  RTC_Init();
 }
 
   I2C_E_Init();                              //External I2C
  
   CAN_Init();    //CANbus

   
if (LEDDriverEN)                     
{
  HAL_Delay(5);  
  LED_Boards = init_LED_boards(1);              //Init LED Driver board(s)
}

SPI_E_Init();                                  //External SPI

if (LCellEN) 
{
  CELL_Board = 0; 
  AD7193_CycleStatus = Cell_Reset;
  
  AD7193_Gain[0] = AD7193_CONF_GAIN_128;
  AD7193_Gain[1] = AD7193_CONF_GAIN_128;
  AD7193_Gain[2] = AD7193_CONF_GAIN_128;
  AD7193_Gain[3] = AD7193_CONF_GAIN_128;
  
  AD7193_Polarity[0] = 0;
  AD7193_Polarity[1] = 0;
  AD7193_Polarity[2] = 0;
  AD7193_Polarity[3] = 0;
  
  AD7193_Zero[0] = EEPROM_ReadF(EE_CellZero0);
  if (AD7193_Zero[0] != AD7193_Zero[0]) {AD7193_Zero[0] = 0;}
  
  AD7193_Zero[1] = EEPROM_ReadF(EE_CellZero1);
  if (AD7193_Zero[1] != AD7193_Zero[1]) {AD7193_Zero[1] = 0;}
  
  AD7193_Zero[2] = EEPROM_ReadF(EE_CellZero2);
  if (AD7193_Zero[2] != AD7193_Zero[2]) {AD7193_Zero[2] = 0;}
  
  AD7193_Zero[3] = EEPROM_ReadF(EE_CellZero3);
  if (AD7193_Zero[3] != AD7193_Zero[3]) {AD7193_Zero[3] = 0;}

  AD7193_Scale[0] = EEPROM_ReadF(EE_CellScale0);
  if (AD7193_Scale[0] != AD7193_Scale[0]) {AD7193_Scale[0] = 1;}

  AD7193_Scale[1] = EEPROM_ReadF(EE_CellScale1);
  if (AD7193_Scale[1] != AD7193_Scale[1]) {AD7193_Scale[1] = 1;}

  AD7193_Scale[2] = EEPROM_ReadF(EE_CellScale2);
  if (AD7193_Scale[2] != AD7193_Scale[2]) {AD7193_Scale[2] = 1;}

  AD7193_Scale[3] = EEPROM_ReadF(EE_CellScale3);
  if (AD7193_Scale[3] != AD7193_Scale[3]) {AD7193_Scale[3] = 1;}  
 
  
  
}

    USB_SN = eeprom_ID;

  
    /* Init Device Library */
  if ( USBD_Init(&USBD_Device, &COMBO_Desc, 0)==HAL_OK)
    
  {
     /* Add Supported Class */
     USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  
    /* Add CDC Interface Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  
    /* Start Device Process */
    USB_started = USBD_Start(&USBD_Device)==HAL_OK;
  }
  if (USB_started) {LED_OFF(0);}
  
  OledPage = 0;
  
  LED_blink_1 = 10;
  
  
  
  
  if(ADT7410_Inited) {ADT7410_Request();}
      
  UpdateOled();
  
  if (OledInitEN == 1) {OLED_ON();}
  OLED_Bright(0xFF);
  
  
  CT_10ms = HAL_GetTick();
  CT_20ms = HAL_GetTick();
  CT_50ms = HAL_GetTick();
  CT_100ms = HAL_GetTick();
  CT_250ms = HAL_GetTick();
  CT_500ms = HAL_GetTick();
  CT_1000ms = HAL_GetTick();
  
  /* Infinite loop */
  while (1)
  {
    
    if (COM_In_Flag)
    {
      ParseCom();   
      
    }
  
    
    
    if ((HAL_GetTick() - CT_10ms) >= 10)    //10ms
    {
      CT_10ms = HAL_GetTick();  
      Evt_10ms();
    }
    
    if ((HAL_GetTick() - CT_20ms) >= 20)    //20ms
    {
      CT_20ms = HAL_GetTick();  
      Evt_20ms();
    }
    
    if ((HAL_GetTick() - CT_50ms) >= 50)    //50ms
    {
      CT_50ms = HAL_GetTick();  
      Evt_50ms();
    }
    
    if ((HAL_GetTick() - CT_100ms) >= 100)    //100ms
    {
      CT_100ms = HAL_GetTick();  
      Evt_100ms();
    }
    
    if ((HAL_GetTick() - CT_250ms) >= 250)    //250ms
    {
      CT_250ms = HAL_GetTick();  
      Evt_250ms();
    }
    
    if ((HAL_GetTick() - CT_500ms) >= 500)    //500ms
    {
      CT_500ms = HAL_GetTick();  
      Evt_500ms();
    }
   
    if ((HAL_GetTick() - CT_1000ms) >= 1000)    //1sec
    {
      CT_1000ms = HAL_GetTick();  
      Evt_1000ms();
    }
    
    if ((HAL_GetTick() - CT_10000ms) >= 10000)    //10sec
    {
      CT_10000ms = HAL_GetTick();  
      Evt_10000ms();
    }
    
    
  }

}

void Evt_10ms(void)      // ====================== Event 10 ms =============================
{
    CAN_Cluster_10ms();
    
    GetPedals();
    
    if (SpecMode==2) {
      if(Pedal_1_Lo > Pedal_1) {Pedal_1_Lo = Pedal_1;}
      if(Pedal_1_Hi < Pedal_1) {Pedal_1_Hi = Pedal_1;}
      if(Pedal_2_Lo > Pedal_2) {Pedal_2_Lo = Pedal_2;}
      if(Pedal_2_Hi < Pedal_2) {Pedal_2_Hi = Pedal_2;}
      if(Pedal_3_Lo > Pedal_3) {Pedal_3_Lo = Pedal_3;}
      if(Pedal_3_Hi < Pedal_3) {Pedal_3_Hi = Pedal_3;}      
    }
    
    
    
    GPIO_Update();
   // CAN_Send_Dummy();
    
    
    
   
}

void Evt_20ms(void)      // ====================== Event 20 ms =============================
{
  if (ButtonsEN == 1)
  { 
    CheckButtons();
  }
  
  if (LEDDriverEN)
  { 
    update_LED_boards();
  }   
    
    
 // tval1 = PollR(3, CPIN_1, PULL_1k, OnOff, 2);
  
    if (SpecMode == 0) {
     
      
   for (int i=0; i<4 ; i++) 
  {
    if (CtrlTypes[i]) {PollControls(i,CtrlTypes[i]); }
  }   
      

  for (int i=0; i<6 ; i++) 
  {
   dw_ch[i] = dw[i] != dw_before[i]; 
    dw_before[i] = dw[i];
   }
  }
  

  
  
  
  struct gamepad_report_t
{
    uint32_t buttons;
    int16_t accelerator;
    int16_t brake;
    int16_t clutch;
};
   struct gamepad_report_t report;
   
   int xx_high_beam = (x_high_beam>0);
   int xx_wiper_f =(x_wiper_f>0);

   
   if (SimulatorType==0) {
       report.buttons = x_pos_light*0x01 + x_low_beam*0x02 + xx_high_beam*0x04 + x_left*0x08 + x_right*0x10 + x_front_fog*0x20 + x_rear_fog*0x40 + xx_wiper_f*0x80
        +x_washer_f*0x100 + x_wiper_r*0x200 + x_washer_r*0x400 + x_cruise*0x800 + x_cc_set*0x1000 +  x_cc_res*0x2000 + x_reset*0x4000 + x_up*0x8000 + x_dn*0x10000;
   
     for (int i=0; i<10 ; i++) 
     {
       report.buttons |= CC_GPIO[i] << (22+i);
     }
   }
   
   if (SimulatorType==SIM_CCDE) {
     int s_wiper_int =(x_wiper_f==1);
     int s_wiper_low =(x_wiper_f==2);
     int s_wiper_high=(x_wiper_f==3);
     int s_low_beam = (x_low_beam && ! xx_high_beam);
     
      
     
     uint32_t trep  = CTRL_SB     * 0x0001 +  //0_1but_j_p   SafetyBelt 
                      CTRL_BR     * 0x0002 +  //0_2but_j_p   HandBrake
                      CTRL_IG     * 0x0004 +  //0_3but_j_p   Ignition
                      CTRL_ST     * 0x0008 +  //0_4but_j_p   Starter
                      x_left      * 0x0010 +  //0_5but_j_p   ToggleLeftSteerLight
                      x_right     * 0x0020 +  //0_6but_j_p   ToggleRightSteerLight   
                      x_pos_light * 0x0040 +  //0_7but_j_p   HWD
                      s_low_beam  * 0x0080 +  //0_8but_j_p   NearLight   
                      xx_high_beam* 0x0100 +  //0_9but_j_p   FarLight 
                      s_wiper_int * 0x0200 +  //0_10but_j_p  WipersOnIntermittent   
                      s_wiper_low * 0x0400 +  //0_11but_j_p  WipersOnLow   
                      s_wiper_high* 0x0800 +  //0_12but_j_p  WipersOnHigh                          
                      CC_GPIO[4]  * 0x1000 +
                        0;
     btnStartMask |= ~ trep;
       
     report.buttons =  trep & btnStartMask;
     
   }
   
      if (SimulatorType==SIM_CCDH) {
     
     int s_low_beam = (x_low_beam && ! xx_high_beam);
     
     if (G_motor_key_ct == 0) {
       
        G_motor_value_prev = G_motor_value;
     
        if (G_motor_value_prev == 0) 
          {
            if (CTRL_IG && CTRL_ST)
            {
              G_motor_value = 2;      
            }  //Starter
          }
     
         if (G_motor_value_prev == 1) 
         {
           if (CTRL_IG == 0)
           {
              G_motor_value = 3;      
           } 
         }
        
        if ((G_motor_value_prev == 2) && (CTRL_ST == 0))
         {
          if (CTRL_IG == 0)
           {
              G_motor_value = 3;      
           } 
           else
           {
              G_motor_value = 1;    
           }
         }
        
        if (G_motor_value_prev == 3) 
         {
              G_motor_value = 0;    
         } 
        
        G_motor_key_ct = 20;
        }

    
    if (G_motor_key_ct > 0)
      {
        G_motor_key_ct--;
      }
     
     
     if (G_wiper_key_ct == 0) {
     
       if (G_wiper_key)
       {
        G_wiper_key = 0;
        G_wiper_key_ct = 3;
       }
       else
       {
         if (G_wiper_value != x_wiper_f)
         {
          G_wiper_key = 1;
          if (++G_wiper_value == 4) {G_wiper_value = 0;}
          G_wiper_key_ct = 3;
         }       
       }
        //int WiperGoal =  x_wiper_f;
     
     
       
     
       
       
     }
      else
      {
        
       G_wiper_key_ct--;
       
      }
     
     
      
     G_motor_key = (G_motor_value == 2) || (G_motor_value == 3);
     
     
     
     
     
     
     
     uint32_t trep  = CTRL_SB     * 0x0001 +  //0_1but_j_p   SafetyBelt 
                      CTRL_BR     * 0x0002 +  //0_2but_j_p   HandBrake
                      CTRL_IG     * 0x0004 +  //0_3but_j_p   Ignition
                      CTRL_ST     * 0x0008 +  //0_4but_j_p   Starter                  
 
                      x_left      * 0x0010 +  //0_5but_j_p   ToggleLeftSteerLight
                      x_right     * 0x0020 +  //0_6but_j_p   ToggleRightSteerLight   
                      x_pos_light * 0x0040 +  //0_7but_j_p   HWD
                      s_low_beam  * 0x0080 +  //0_8but_j_p   NearLight   
                      xx_high_beam* 0x0100 +  //0_9but_j_p   FarLight 
                        
                      G_motor_key * 0x1000 +  //0_13but_j_p   Motor on/off key  
                      G_wiper_key * 0x2000 +  //0_14but_j_p  Wipers key
    
     
                        0;
     
     btnStartMask |= ~ trep;
       
     report.buttons =  trep & btnStartMask;
   }
   
   
   
   
   
   report.clutch = N_Pedal_1;
   report.brake = N_Pedal_2;
   report.accelerator = N_Pedal_3;
   

  
  
  USBD_HandleTypeDef *pdev = &USBD_Device;
  USBD_COMBO_HandleTypeDef  *hhid = pdev->pClassData;
  
  TVDS = pdev->dev_state;
  TVRR = pdev->dev_connection_status;
  TVHS = hhid->state;
   
 //  if (CC_GPIO[9] == 0)      
   {
     USBD_HID_SendReport (&USBD_Device,(unsigned char*)&report, 10);
   }
   
   
}

void Evt_50ms(void)      // ====================== Event 50 ms =============================
{
 
  CAN_Cluster_50ms();

  
  Blinker();
  if (USB_started) {
    if ((RedLed == 1) && (HAL_GetTick()>RedLedTm))
     {
       RedLed = 0; 
       LED_OFF(0);
     }
  }
  
/*   if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET){
 
// AD7193_ChannelSelect(AD7193_CH_0);
 unsigned long didata = AD7193_ReadData();
 //volt0 = AD7193_ConvertToVolts(didata, -19.53);
 volt0 = (((float)didata / (1ul << 23)) - 1)*100000;
 //volt0 = -volt0*3.68;
   volt0 = -volt0*12.8;
   
   }*/

  if (CELL_Board == 0)
     {
      switch (AD7193_Cycle())
      {
      case Cell_Fail:
        CELL_Board = -1;
        USB_Print("CELL: MSG='FAIL'\r\n");
        break;
      case Cell_MsgReady:
        USB_Print("CELL: MSG='READY'\r\n");
        break;   
        
      case Cell_ContRead:
        if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
        {
          unsigned long didata = AD7193_ReadData();
          CellVoltage = (((float)didata / (1ul << 23)) - 1)*100000;
          float CaldValue = (CellVoltage - AD7193_Zero[AD7193_CurrentChannel])*AD7193_Scale[AD7193_CurrentChannel];       
          
          sprintf(TextBuffer,"CELL: TY='D';CHANNEL=%d;GAIN=%d;DATA='%.1f'\r\n",AD7193_CurrentChannel+1, AD7193_Gain[AD7193_CurrentChannel], CaldValue);
          
          USB_Print(TextBuffer);
        }
        break;   
        
        case Cell_CalZero:
        if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
        {
          unsigned long didata = AD7193_ReadData();
          AD_CalAcc += (((float)didata / (1ul << 23)) - 1)*100000;          
          
          if (++AD_CalCt == CellCalCycles)
          {
            AD7193_Zero[AD7193_CurrentChannel] = AD_CalAcc / CellCalCycles;
            EEPROM_WriteF(EE_CellZero0 + AD7193_CurrentChannel*4,AD7193_Zero[AD7193_CurrentChannel]);
            
            
            sprintf(TextBuffer,"CELL: TY='Z';ZERO=%f\r\n", AD7193_Zero[AD7193_CurrentChannel]);
            USB_Print(TextBuffer);
            AD7193_CycleStatus = Cell_ContRead;
          }
        }
        break;  
        
        case Cell_CalWeight:
        if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
        {
          unsigned long didata = AD7193_ReadData();
          AD_CalAcc += (((float)didata / (1ul << 23)) - 1)*100000;          
          
          if (++AD_CalCt == CellCalCycles)
          {
            float AD_CalTmp = AD_CalAcc / CellCalCycles - AD7193_Zero[AD7193_CurrentChannel];
            
            AD7193_Scale[AD7193_CurrentChannel] = AD_CalVal / AD_CalTmp;
            EEPROM_WriteF(EE_CellScale0 + AD7193_CurrentChannel*4,AD7193_Scale[AD7193_CurrentChannel]);
            
            sprintf(TextBuffer,"CELL: TY='SC';SCALE=%f\r\n", AD7193_Scale[AD7193_CurrentChannel]);
            USB_Print(TextBuffer);
            AD7193_CycleStatus = Cell_ContRead;
          }
        }
        break;          
        
      }
     
     }
  
  
  
    
}

void Evt_100ms(void)      // ====================== Event 100 ms =============================
{
   CAN_Cluster_100ms(); 
  
 UpdateOled();
   // LED_SW(0);
     //OLED_Bright(bright++);
   //  OLED_RND();
 
 if (SpecMode != 0) { LED_SW(1);}
   else { 
      if (LED_blink_1 > 0)
      {LED_blink_1--;
      LED_SW(1);} else
      {LED_OFF(1);}
     }
 
 
 
}

void Evt_250ms(void)      // ====================== Event 250 ms =============================
{
   CAN_Cluster_250ms(); 
 
}


void Evt_500ms(void)      // ====================== Event 500 ms =============================
{
 // if (CC_GPIO[0]) 
    CAN_Cluster_500ms();
    
  if (blink) {blink=0;} else {blink=1;}
  

  LED_SW(2);
  
  //if (blink) {ADG2128_ON(0,MX_GND | CPIN_1);} else {ADG2128_OFF(0,MX_GND | CPIN_1);} 
  //USB_COM_Print("12345/n");
  
 if (SpecMode == 1) { 
   TestMatrix(3);
 };
 

}

void Evt_1000ms(void)      // ====================== Event 1 sec =============================
{
    CAN_Cluster_1000ms(); 
  
  if(ADT7410_Inited) {ADT7410_Request();}
  

   // sprintf(TextBuffer,"Resistor = %d     \r\n", 1);
   // USB_Print(TextBuffer);
    // USBD_CDC_SetTxBuffer(&USBD_Device, (unsigned char*)TextBuffer, strlen((char const *)TextBuffer));    
    // USBD_CDC_TransmitPacket(&USBD_Device);  
    

  if ( USB_started == 0 ) 
  {
  if ( USBD_Init(&USBD_Device, &COMBO_Desc, 0)==HAL_OK)
    
  {
     /* Add Supported Class */
     USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  
    /* Add CDC Interface Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  
    /* Start Device Process */
    USB_started = USBD_Start(&USBD_Device)==HAL_OK;
  }
  
  
  }
  

 if ((OledDimmerEN == 1) && (OledState == 1) && (SpecMode==0))
  {
   OledDimCt--;
   
   if ((OledDimCt < (OLED_ON_TIME - OLED_DIM_TIME)) && (OledBrightness == 0xFF))
   {
     OLED_Bright(0);
   }
   
   if (OledDimCt == 0) {
   OLED_OFF();
   
   }
 
  } 
  
  
}

void Evt_10000ms(void)      // ====================== Event 10 sec =============================
{
  if(TempEN == 0) 
  {
    Temperature =  GetMCTemp();
  }
  
   Vbat = GetVbat();  
}

int PollR(int addr, uint8_t X, uint32_t pullup, uint32_t* values, int number)
{
  int value = GetRes(addr, X ,pullup);
  int mindistance=5000;
  int minnum=99;
  for (int i=0; i<number; i++)
  {
    int distance = abs(value-values[i]);
    if (distance<mindistance)
    {
      mindistance = distance;
      minnum = i;     
    }
  }
  return minnum;  
  
}

void ParseCom(void)
{
 RedLedBlast(150); 
 //LED_SW(0); 
 tval1 = 0; 
 char *CmdString;
 char *ValPos;
 char *p;
 int vv[4];
 
 
 if (CmpStr(COM_In_Buffer, "RESET", 0)) 
     {
       NVIC_SystemReset();
     } 
 
if (CmpStr(COM_In_Buffer, "STATUS", 0)) 
     {
       //STATUS PRINT
     } 

if (CmpStr(COM_In_Buffer, "INFO", 0)) 
     {
       //INFO PRINT
       sprintf(TextBuffer,"MESSAGE: Firmware version='"FW_VERSION"'; Serial Number=%04x\r\n", eeprom_ID);
       USB_Print(TextBuffer);
       
       sprintf(TextBuffer,"MESSAGE: MATRIX='----'\r\n");
       for (int i=0; i<4; i++) {if (ADG2128_Inited[i]) {TextBuffer[17+i]='1'+i; }}
       USB_Print(TextBuffer);
       
       if(ADT7410_Inited) {sprintf(TextBuffer,"MESSAGE: Temperature=%.1f C \r\n", Temperature);
       USB_Print(TextBuffer); }
       if (RtcEN) { 
        RTC_TimeTypeDef stimestructureget;
        HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);
        RTC_DateTypeDef sdatestructureget;
        HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  
          sprintf(TextBuffer,"MESSAGE: Time=%02d.%02d.%02d; Date=%02d/%02d/%02d \r\n",
                     stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds
                    ,sdatestructureget.Date, sdatestructureget.Month, 2000 + sdatestructureget.Year);
          
          
          USB_Print(TextBuffer);  
         }
          sprintf(TextBuffer,"MESSAGE: Ctrl1 = %d; Ctrl2 = %d; Ctrl3 = %d; Ctrl4 = %d\r\n",CtrlTypes[0],CtrlTypes[1],CtrlTypes[2],CtrlTypes[3]);
          USB_Print(TextBuffer);   
          
         sprintf(TextBuffer,"MESSAGE: ClusterType = %d; ClusterAutonomy = %d\r\n",ClusterList[ClusterType],ClusterAutonomy);
          USB_Print(TextBuffer);   
          
          sprintf(TextBuffer,"MESSAGE: TurnDelay = %d; TurnPeriod = %d; TurnON = %d\r\n",TurnDelay,TurnPeriod,TurnON);
          USB_Print(TextBuffer);   
          
            
          sprintf(TextBuffer,"MESSAGE: SB_PIN = %d; SB_INV = %d; BR_PIN = %d; BR_INV = %d; IG_PIN = %d; IG_INV = %d; ST_PIN = %d; ST_INV = %d;\r\n",
                            CTRL_SB_PIN,CTRL_SB_INV,CTRL_BR_PIN,CTRL_BR_INV,CTRL_IG_PIN,CTRL_IG_INV,CTRL_ST_PIN,CTRL_ST_INV);
          USB_Print(TextBuffer);     
          
          sprintf(TextBuffer,"MESSAGE: SimulatorType = %d\r\n",SimulatorType );
          USB_Print(TextBuffer);  
          sprintf(TextBuffer,"MESSAGE: PedType = %d\r\n",PedType );
          USB_Print(TextBuffer);  
          
     }  
 
 
     
p = strchr(COM_In_Buffer, ':');

 
 if (p != 0) {
    CmdString = p+1;

    p = strchr(CmdString, '=');

    if (p != 0) {
    ValPos = p+1;
    
    vv[0]= GetIntValue(ValPos,0);
    vv[1]= GetIntValue(ValPos,1);
    vv[2]= GetIntValue(ValPos,2);
    vv[3]= GetIntValue(ValPos,3);
   
    }
    
   if (CmpStr(COM_In_Buffer, "SET", ':'))    //============ "SET"===========
     {
              
       if (CmpStr(CmdString, "TIME", '='))   // SET TIME
       {
         
        if ((vv[0] != -1) && (vv[1] != -1) &&(vv[2] != -1) )
        {
          RTC_TimeSet(vv[0],vv[1],vv[2]);
        }              
       } 
       if (CmpStr(CmdString, "DATE", '='))  // SET DATE
       {
        if ((vv[0] != -1) && (vv[1] != -1) &&(vv[2] != -1) )
        {
          RTC_DateSet(vv[0],vv[1],vv[2]%100);
        }  
       } 
       
       if (CmpStr(CmdString, "SERIALNUMBER", '='))  // SETSERIAL
       {
        if ((vv[0] != -1))
        {
         EEPROM_WriteB(EE_SN+1,(vv[0]&0xFF));
         HAL_Delay(10);
         EEPROM_WriteB(EE_SN,(vv[0]&0xFF00)>>8);
         HAL_Delay(10);
         eeprom_ID = EEPROM_ReadW(EE_SN);

        }  
       } 
        if (CmpStr(CmdString, "RTC", '='))  // RTC
       {
        if ((vv[0] != -1))
        {
         EEPROM_WriteB(EE_RtcEN,vv[0]);
         //HAL_Delay(10);
         RtcEN = vv[0];
         if (RtcEN) {  
            RTC_Init();
         }
        }  
       }
       
        if (CmpStr(CmdString, "TEMP", '='))  // TEMP
       {
        if ((vv[0] != -1))
        {
         EEPROM_WriteB(EE_TempEN,vv[0]);
        
         TempEN = vv[0];

        }  
       }
       
       if (CmpStr(CmdString, "CTRL", '='))  // CONTROLS
       {
       
        
         if (vv[0] == 1) 
           {
            CtrlTypes[0] = vv[1];
             EEPROM_WriteB(EE_CtrlTypes0, CtrlTypes[0]);
            
            
          //  sprintf(TextBuffer,"Control 1 is set to %d\r\n",vv[1] );
        //    USB_Print(TextBuffer); 
           }
         if (vv[0] == 2) 
           {
            CtrlTypes[1] = vv[1];
             EEPROM_WriteB(EE_CtrlTypes1, CtrlTypes[1]);
        //    sprintf(TextBuffer,"Control 2 is set to %d\r\n",vv[1] );
         //   USB_Print(TextBuffer); 
           } 
         if (vv[0] == 3) 
           {
            CtrlTypes[2] = vv[1];
            EEPROM_WriteB(EE_CtrlTypes2, CtrlTypes[2]);
        //    sprintf(TextBuffer,"Control 3 is set to %d\r\n",vv[1] );
        //    USB_Print(TextBuffer); 
           }
         if (vv[0] == 4) 
           {
            CtrlTypes[3] = vv[1];
             EEPROM_WriteB(EE_CtrlTypes3, CtrlTypes[3]);
        //    sprintf(TextBuffer,"Control 4 is set to %d\r\n",vv[1] );
        //    USB_Print(TextBuffer); 
           } 
          
    //     HAL_Delay(10);
       }
       
        if (CmpStr(CmdString, "CLUSTERTYPE", '='))  // ClusterType
       {
        if ((vv[0] != -1))
        {
         ClusterType = vv[0];
         EEPROM_WriteB(EE_ClusterType,vv[0]);
     //    HAL_Delay(10);
        }  
       }
       
       if (CmpStr(CmdString, "CLUSTERAUTONOMY", '='))  // ClusterAutonomy
       {
        if ((vv[0] != -1))
        {
         ClusterAutonomy = vv[0];
         EEPROM_WriteB(EE_ClusterAutonomy,vv[0]);
      //   HAL_Delay(10);
        }  
       }
       

       
      if (CmpStr(CmdString, "TURNDELAY", '='))  // TurnDelay
       {
        if ((vv[0] != -1))
        {
         TurnDelay = vv[0];
         EEPROM_WriteB(EE_TurnDelay,vv[0]);
      //   HAL_Delay(10);
        }  
       } 
       
        if (CmpStr(CmdString, "TURNPERIOD", '='))  // TurnPeriod
       {
        if ((vv[0] != -1))
        {
         TurnPeriod = vv[0];
         EEPROM_WriteB(EE_TurnPeriod,vv[0]);
       //  HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "TURNON", '='))  // TurnON
       {
        if ((vv[0] != -1))
        {
         TurnON = vv[0];
         EEPROM_WriteB(EE_TurnON,vv[0]);
        // HAL_Delay(10);
        }  
       } 
   
       
       if (CmpStr(CmdString, "SBPIN", '='))  // CTRL_SB_PIN
       {
        if ((vv[0] != -1))
        {
         CTRL_SB_PIN = vv[0];
         EEPROM_WriteB(EE_CTRL_SB_PIN,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "SBINV", '='))  // CTRL_SB_INV
       {
        if ((vv[0] != -1))
        {
         CTRL_SB_INV = vv[0];
         EEPROM_WriteB(EE_CTRL_SB_INV,vv[0]);
       //  HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "BRPIN", '='))  // CTRL_BR_PIN
       {
        if ((vv[0] != -1))
        {
         CTRL_BR_PIN = vv[0];
         EEPROM_WriteB(EE_CTRL_BR_PIN,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
        if (CmpStr(CmdString, "BRINV", '='))  // CTRL_BR_INV
       {
        if ((vv[0] != -1))
        {
         CTRL_BR_INV = vv[0];
         EEPROM_WriteB(EE_CTRL_BR_INV,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
        if (CmpStr(CmdString, "IGPIN", '='))  // CTRL_IG_PIN
       {
        if ((vv[0] != -1))
        {
         CTRL_IG_PIN = vv[0];
         EEPROM_WriteB(EE_CTRL_IG_PIN,vv[0]);
       //  HAL_Delay(10);
        }  
       } 
       
        if (CmpStr(CmdString, "IGINV", '='))  // CTRL_IG_INV
       {
        if ((vv[0] != -1))
        {
         CTRL_IG_INV = vv[0];
         EEPROM_WriteB(EE_CTRL_IG_INV,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "STPIN", '='))  // CTRL_ST_PIN
       {
        if ((vv[0] != -1))
        {
         CTRL_ST_PIN = vv[0];
         EEPROM_WriteB(EE_CTRL_ST_PIN,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "STINV", '='))  // CTRL_ST_INV
       {
        if ((vv[0] != -1))
        {
         CTRL_ST_INV = vv[0];
         EEPROM_WriteB(EE_CTRL_ST_INV,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
   
        if (CmpStr(CmdString, "SIMULATORTYPE", '='))  // SimulatorType
       {
        if ((vv[0] != -1))
        {
         SimulatorType = vv[0];
         EEPROM_WriteB(EE_SimulatorType,vv[0]);
       //  HAL_Delay(10);
        }  
       } 
       
        if (CmpStr(CmdString, "PEDTYPE", '='))  // PedType
       {
        if ((vv[0] != -1))
        {
         PedType = vv[0];
         EEPROM_WriteB(EE_PedType,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "PEDFILTER", '='))  // PedFilter
       {
        if ((vv[0] != -1))
        {
         PedFiltering = vv[0];
         EEPROM_WriteB(EE_PedFiltering,vv[0]);
        // HAL_Delay(10);
        }  
       } 
        if (CmpStr(CmdString, "BUTTONS_EN", '='))  // PedType
       {
        if ((vv[0] != -1))
        {
         ButtonsEN = vv[0];
         EEPROM_WriteB(EE_ButtonsEN,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "OLED_INIT", '='))  // OledInit
       {
        if ((vv[0] != -1))
        {
         PedType = vv[0];
         EEPROM_WriteB(EE_OledInitEN,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
       if (CmpStr(CmdString, "OLED_DIMMER", '='))  // OledDimmer 
       {
        if ((vv[0] != -1))
        {
         PedType = vv[0];
         EEPROM_WriteB(EE_OledDimmerEN,vv[0]);
        // HAL_Delay(10);
        }  
       } 
       
       
       if (CmpStr(CmdString, "SPECMODE", '='))  // Specmode
       {
        if (vv[0] != -1)
        {
         SpecMode = vv[0];
         
         if (SpecMode == 1) {
           
           PrintMsg("Controls 4 poll");
           ADG2128_RESET(3);
         }
         
        }  
       } 
       if (CmpStr(CmdString, "SCREENDUB", '='))  // SCREEN DUB MODE
       {
        if (vv[0] != -1)
        {
         ScreenDubMode = vv[0];
         if (ScreenDubMode==1){
           OLED_USB_Update();
           for (int i=0; i <= 7; i++){
              USB_Print(COM_buffer[i]);
           }
         }
        }  
       }
    
     }

    
    if (CmpStr(COM_In_Buffer, "VALUE", ':')) 
     {
              
       if (CmpStr(CmdString, "RPM", '=')) 
       {
        d_rpm = vv[0];
       }
       
       if (CmpStr(CmdString, "SPEED", '=')) 
       {
        d_speed = vv[0];
       }
       
       if (CmpStr(CmdString, "ODO", '=')) 
       {
        d_odo = vv[0];
       }      
       
       if (CmpStr(CmdString, "FUEL", '=')) 
       {
        d_fuel = vv[0];
       }        

       if (CmpStr(CmdString, "TEMP", '=')) 
       {
        d_temp = vv[0];
       }   
       
       if (CmpStr(CmdString, "LEFT", '=')) 
       {
        d_left = vv[0];
       } 
       
       if (CmpStr(CmdString, "RIGHT", '=')) 
       {
        d_right = vv[0];
       } 
       
       if (CmpStr(CmdString, "BOTH", '=')) 
       {
        d_left = vv[0];
        d_right = vv[0];
       } 
       
       if (CmpStr(CmdString, "EMERG", '=')) 
       {
        d_emerg = vv[0];
       } 
       
       if (CmpStr(CmdString, "B_FL_1", '=')) 
       {
        d_b_fl_1 = vv[0];
       }
       
       if (CmpStr(CmdString, "B_FL_2", '=')) 
       {
        d_b_fl_2 = vv[0];
       }
       
       if (CmpStr(CmdString, "TURN", '=')) 
       {
        d_left = 0;
        d_right = 0;
        if ((vv[0]==1) || (vv[0]==3)) {d_left = 1;}
        if ((vv[0]==2) || (vv[0]==3)) {d_right = 1;}
       } 
       
       if (CmpStr(CmdString, "HIGH_BEAM", '=')) 
       {
        d_high_beam = vv[0];
       } 
       
        if (CmpStr(CmdString, "LOW_BEAM", '=')) 
       {
        d_low_beam = vv[0];
       } 
       
       if (CmpStr(CmdString, "LOW_BEAM_LEV", '=')) 
       {
        d_low_beam_lev = vv[0];
       } 
       
        if (CmpStr(CmdString, "FRONT_FOG", '=')) 
       {
        d_front_fog = vv[0];
       } 
       
       if (CmpStr(CmdString, "REAR_FOG", '=')) 
       {
        d_rear_fog = vv[0];
       } 
       
       if (CmpStr(CmdString, "BRAKE", '=')) 
       {
        d_brake = vv[0];
       } 
       
       if (CmpStr(CmdString, "CLUSTER_PWR", '=')) 
       {
        d_cluster_pwr = vv[0];
        
        if (d_cluster_pwr) 
         { 
          GPIO_ClusterPowerOn();
          CAN_ClusterPowerOn();
           
         }
         else
         {
          GPIO_ClusterPowerOff();
          CAN_ClusterPowerOff();  
         }
        
       } 
       
       if (CmpStr(CmdString, "STEERING", '=')) 
       {
        d_steering_e = vv[0];
       } 
       
       if (CmpStr(CmdString, "BACKLIGHT_C", '=')) 
       {
        d_backlight_c = vv[0];
       } 
       
       if (CmpStr(CmdString, "BACKLIGHT_B", '=')) 
       {
        d_backlight_c = vv[0];
       } 
       
       if (CmpStr(CmdString, "NEED_FUEL", '=')) 
       {
        d_fuel_need = vv[0];
       } 
       
       if (CmpStr(CmdString, "TCO_PERF", '=')) 
       {
        d_tco_perf = vv[0];
       } 
       
       if (CmpStr(CmdString, "HB", '=')) 
       {
        d_hb = vv[0];
       } 
       
       if (CmpStr(CmdString, "GEAR", '=')) 
       {
        d_gear = vv[0];
       } 
       
       if (CmpStr(CmdString, "ADBLUE", '=')) 
       {
        d_adblue = vv[0]*2.55;
       } 
       
       if (CmpStr(CmdString, "DECCEL", '=')) 
       {
        d_deccel = vv[0];
       } 
       
       if (CmpStr(CmdString, "EB", '=')) 
       {
        d_eb = vv[0];
       } 
       
       if (CmpStr(CmdString, "BTN_UP", '=')) 
       {
        d_btn_up = vv[0];
       } 
       
       if (CmpStr(CmdString, "BTN_ESC", '=')) 
       {
        d_btn_esc = vv[0];
       } 
       
       if (CmpStr(CmdString, "BTN_PAGE", '=')) 
       {
        d_btn_page = vv[0];
       } 
       
       if (CmpStr(CmdString, "BTN_MENU", '=')) 
       {
        d_btn_menu = vv[0];
       } 
       
       if (CmpStr(CmdString, "BTN_MINUS", '=')) 
       {
        d_btn_minus = vv[0];
       } 
       
       if (CmpStr(CmdString, "BTN_PLUS", '=')) 
       {
        d_btn_plus = vv[0];
       } 
       
       if (CmpStr(CmdString, "BTN_DOWN", '=')) 
       {
        d_btn_down = vv[0];
       } 
       
       if (CmpStr(CmdString, "TRIP_DST", '=')) 
       {
        d_trid = vv[0];
       } 
       
       if (CmpStr(CmdString, "TOTAL_DST", '=')) 
       {
        d_totd = vv[0];
       } 
       
       if (CmpStr(CmdString, "HOURS", '=')) 
       {
        d_hours = vv[0];
       } 
       
              
       if (CmpStr(CmdString, "PRESS_1", '=')) 
       {
        d_press_1 = vv[0]*1.4;
       } 
       
       if (CmpStr(CmdString, "PRESS_2", '=')) 
       {
        d_press_2 = vv[0]*1.4;
       } 
       
       if (CmpStr(CmdString, "CRUISE_LED", '=')) 
       {
        d_cruise_led = vv[0];
       } 
       
       if (CmpStr(CmdString, "CRUISE_LCD", '=')) 
       {
        d_cruise_lcd = vv[0];
       } 
       
       if (CmpStr(CmdString, "CRUISE_PTO", '=')) 
       {
        d_cruise_pto = vv[0];
       } 
       
       if (CmpStr(CmdString, "CRUISE_SPEED", '=')) 
       {
        d_cruise_speed = vv[0];
       } 
       
       if (CmpStr(CmdString, "STARTER", '=')) 
       {
        d_starter = vv[0];
       } 
       
       if (CmpStr(CmdString, "MIRROR", '=')) 
       {
        d_mirror = vv[0];
       } 
       
       if (CmpStr(CmdString, "DOORS", '=')) 
       {
        d_doors = vv[0];
       } 
       
       if (CmpStr(CmdString, "BEAM_POS", '=')) 
       {
        d_beam_pos = vv[0];
       } 
       
       if (CmpStr(CmdString, "PREHEAT", '=')) 
       {
        d_preheat = vv[0];
       } 
       
       if (CmpStr(CmdString, "SP_LIMIT_LED", '=')) 
       {
        d_speed_limit_led = vv[0];
       } 
       
       if (CmpStr(CmdString, "SP_LIMIT_VALUE", '=')) 
       {
        d_speed_limit_value = vv[0];
       } 
       
       if (CmpStr(CmdString, "SP_LIMIT_SET", '=')) 
       {
        d_speed_limit_set = vv[0];
       } 
       
       if (CmpStr(CmdString, "CHECK_ENGINE", '=')) 
       {
        d_check = vv[0];
       } 
       
       if (CmpStr(CmdString, "FL_CHECK_ENGINE", '=')) 
       {
        d_check_fl = vv[0];
       } 
       
       if (CmpStr(CmdString, "CAB_TILTED", '=')) 
       {
        d_cab_tilted = vv[0];
       } 
       
       if (CmpStr(CmdString, "OIL_PRESS_LOW", '=')) 
       {
        d_oil_press_low = vv[0];
       } 
       
       if (CmpStr(CmdString, "OIL_TEMP_HIGH", '=')) 
       {
        d_oil_temp_high = vv[0];
       } 
       
       if (CmpStr(CmdString, "TLR_LCD", '=')) 
       {
        d_tlr_lcd = vv[0];
       } 
       
       if (CmpStr(CmdString, "TLR_LED", '=')) 
       {
        d_tlr_led = vv[0];
       } 
       
       if (CmpStr(CmdString, "BR_ERR_1", '=')) 
       {
        d_br_err_1 = vv[0];
       } 
       
       if (CmpStr(CmdString, "BR_ERR_2", '=')) 
       {
        d_br_err_2 = vv[0];
       } 
       
       if (CmpStr(CmdString, "BR_ERR_3", '=')) 
       {
        d_br_err_3 = vv[0];
       } 
       

       
       if (CmpStr(CmdString, "PTO1", '=')) 
       {
        d_1pto = vv[0];
       } 
       
       if (CmpStr(CmdString, "PTO2", '=')) 
       {
        d_2pto = vv[0];
       } 
       
       if (CmpStr(CmdString, "PTO3", '=')) 
       {
        d_3pto = vv[0];
       } 
       
       if (CmpStr(CmdString, "TR_EL_CON", '=')) 
       {
        d_tr_el_con = vv[0];
       } 
       
        if (CmpStr(CmdString, "ETC7_MODE", '=')) 
       {
        d_etc7_mode = vv[0];
       }
       
       if (CmpStr(CmdString, "AIRSUSP", '=')) 
       {
        d_airsusp = vv[0];
       }
       
       if (CmpStr(CmdString, "SUSP_LCM", '=')) 
       {
        d_susp_lcm = vv[0];
       }
       
       if (CmpStr(CmdString, "SUSP_LIFT", '=')) 
       {
        d_susp_lift = vv[0];
       }
       
       if (CmpStr(CmdString, "ESP", '=')) 
       {
        d_esp = vv[0];
       }
       
       if (CmpStr(CmdString, "NO_ABS", '=')) 
       {
        d_no_abs = vv[0];
       }
       
       if (CmpStr(CmdString, "NO_ASR", '=')) 
       {
        d_no_asr = vv[0];
       }
       
       if (CmpStr(CmdString, "ASR", '=')) 
       {
        d_asr = vv[0];
       }
       
       if (CmpStr(CmdString, "WARNING_1", '=')) 
       {
        d_warning_1 = vv[0];
       }
       
       if (CmpStr(CmdString, "WARNING_2", '=')) 
       {
        d_warning_2 = vv[0];
       }
       
       if (CmpStr(CmdString, "LOCK_FRONT", '=')) 
       {
        d_lock_front = vv[0];
       }
       
       if (CmpStr(CmdString, "LOCK_CENTER", '=')) 
       {
        d_lock_center = vv[0];
       }
       
       if (CmpStr(CmdString, "LOCK_REAR", '=')) 
       {
        d_lock_rear = vv[0];
       }
       
       if (CmpStr(CmdString, "OIL_LOW", '=')) 
       {
        d_oil_low = vv[0];
       }
       
       if (CmpStr(CmdString, "OIL_HIGH", '=')) 
       {
        d_oil_high = vv[0];
       }
       
       if (CmpStr(CmdString, "COOLANT_LOW", '=')) 
       {
        d_coolant_low = vv[0];
       }

       if (CmpStr(CmdString, "BR_FLUID_LOW", '=')) 
       {
        d_br_fluid_low = vv[0];
       }
       
       if (CmpStr(CmdString, "ST_FLUID_LOW", '=')) 
       {
        d_st_fluid_low = vv[0];
       }

       if (CmpStr(CmdString, "WS_FLUID_LOW", '=')) 
       {
        d_ws_fluid_low = vv[0];
       }
       
       if (CmpStr(CmdString, "STEER_ERROR", '=')) 
       {
        d_steer_error = vv[0];
       }
       
       if (CmpStr(CmdString, "FILTER_AIR", '=')) 
       {
        d_air_filter = vv[0];
       }

       if (CmpStr(CmdString, "FILTER_FUEL", '=')) 
       {
        d_fuel_filter = vv[0];
       }

       if (CmpStr(CmdString, "FILTER_OIL", '=')) 
       {
        d_oil_filter = vv[0];
       }       
       
       if (CmpStr(CmdString, "WATER_TANK", '=')) 
       {
        d_water_tank = vv[0];
       }       

       if (CmpStr(CmdString, "OIL_TOO_LOW", '=')) 
       {
        d_oil_too_low = vv[0];
       }  
       
       if (CmpStr(CmdString, "COOLANT_TOO_LOW", '=')) 
       {
        d_coolant_too_low = vv[0];
       }
       
       if (CmpStr(CmdString, "OIL_LEVEL", '=')) 
       {
        d_oil_level = vv[0];
       }      
       
       if (CmpStr(CmdString, "OIL_PRESSURE", '=')) 
       {
        d_oil_pressure = vv[0];
       }      
       
       if (CmpStr(CmdString, "EXT_TEMP", '=')) 
       {
        d_ext_temp = 0x2220 + (int)((float)(vv[0])*3.2f);
       }   
       
       if (CmpStr(CmdString, "TEMP", '=')) 
       {
        d_temp = vv[0];
       }   
       
       if (CmpStr(CmdString, "EDC_WARNING", '=')) 
       {
        d_edc_warning = vv[0];
       }          
       
       if (CmpStr(CmdString, "EDC_ERROR", '=')) 
       {
        d_edc_error = vv[0];
       }          
       
       //=========================================================
       
       if (CmpStr(CmdString, "TT", '=')) 
       {
        d_tt[vv[1]] = vv[0];
       }
       
       
       
       //=========================================================
       if (CmpStr(CmdString, "LED_NOW", '=')) 
       {
        LedDrvSetNow( vv[0],vv[1] );
       } 
   
       if (CmpStr(CmdString, "LED_SMOOTH", '=')) 
       {
        LedDrvSetSmooth( vv[0],vv[1],vv[2] );
       }     
       
       
       
     }
   
   if (CmpStr(COM_In_Buffer, "CLICK", ':') )
     {
     if (CmpStr(CmdString, "BTN", '=')) 
       {
         if (vv[0]==1) {BtnOkPress();};
         if (vv[0]==2) {BtnMenuPress();};
         if (vv[0]==3) {BtnLtPress();};
         if (vv[0]==4) {BtnRtPress();};
       } 
     }
   
   if (CmpStr(COM_In_Buffer, "EEPROM", ':') )
     {
     // tval1 = 3; 
     }
   
   if (CmpStr(COM_In_Buffer, "CELL", ':') )
     {
     if (CmpStr(CmdString, "RESET", 0)) 
       { 
         CELL_Board = 0; 
         AD7193_CycleStatus = Cell_Reset;  
       }
     if (CmpStr(CmdString, "ZERO", 0)) 
       { 
         AD_CalAcc = 0;
         AD_CalCt = 0;
         AD7193_CycleStatus = Cell_CalZero;  
       }
     
     if (CmpStr(CmdString, "CAL", '=')) 
       { 
         AD_CalAcc = 0;
         AD_CalCt = 0;
         AD_CalVal = vv[0]; 
         AD7193_CycleStatus = Cell_CalWeight;  
       }
     
     if (CmpStr(CmdString, "CHANNEL", '=')) 
       { 
         if (vv[0]==0) 
           {
             AD7193_CycleStatus = Cell_Ready;
             AD7193_LED(0);
           }
         if (vv[0]==1) 
           {
             AD7193_CurrentChannel = 0;
             AD7193_CycleStatus = Cell_SelCh1;  
           }
         if (vv[0]==2) 
           {
             AD7193_CurrentChannel = 1;
             AD7193_CycleStatus = Cell_SelCh2;  
           }
         if (vv[0]==3) 
           {
             AD7193_CurrentChannel = 2;
             AD7193_CycleStatus = Cell_SelCh3;  
           }         
         if (vv[0]==4) 
           {
             AD7193_CurrentChannel = 3;
             AD7193_CycleStatus = Cell_SelCh4;  
           }   
       }
     } 
    
    
    
   
    if (CmpStr(COM_In_Buffer, "TEST", ':') )
     {
       if (CmpStr(CmdString, "LOOP", '=')) 
       {
       uint8_t MPins[]= {CPIN_01,CPIN_02,CPIN_03,CPIN_04,CPIN_05,CPIN_06,CPIN_07,CPIN_08,CPIN_09,CPIN_10,CPIN_11,CPIN_12};
      
       uint8_t MatrA = vv[0] - 1;
       uint8_t MatrB = vv[1] - 1;
       
       int PinCt = 12;
       
       int tst;
       int fail = 0;
       
       if (vv[2]==8 ) {PinCt = 8;}

       
       
       for (int i=0; i<12; i++) 
       {
        ADG2128_OFF(MatrA, MX_GND + MPins[i]); 
        ADG2128_OFF(MatrB, MX_GND + MPins[i]); 
       }
  
      for (int i=0; i<12; i++)
      for (int j=0; j<12; j++)
    {
      int pro=1;
      if (PinCt == 8) {
        if ((i==5)||(j==5)) { pro=0;}
        if ((i==7)||(j==7)) { pro=0;}
        if ((i==9)||(j==9)) { pro=0;}
        if ((i==11)||(j==11)) { pro=0;}
      }
      if (pro==1)
      {  ADG2128_ON(MatrA, MX_GND + MPins[i]);
        
        tst = PollR(MatrB, MPins[j], PULL_1k, OnOff, 2);
        
        
        if ( (i != j) && (tst == 1))
        {
         sprintf(TextBuffer,"Short %d -> %d\r\n",i+1,j+1 );
          USB_Print(TextBuffer);  
          fail = 1;
         }
        
        if ( (i == j) && (tst == 0))
        {
         sprintf(TextBuffer,"Break %d -> %d\r\n",i+1,j+1 );
          USB_Print(TextBuffer);  
          fail = 1;
         }
        
      /*  if (tst==1)
        {
         sprintf(TextBuffer,"XX %d -> %d\r\n",i+1,j+1 );
          USB_Print(TextBuffer);  
          //fail = 1;
         }*/
        
        ADG2128_OFF(MatrA, MX_GND + MPins[i]);}
    
    
        HAL_Delay(1);
     
     
   }
   
   if (fail)
   {USB_Print("Test FAILED\r\n");  }
    else
   {USB_Print("Test PASSED\r\n");  }
       
       
       } 
       
     }
    
    
   
      }
 
 COM_In_Flag = 0; 
}


int CmpStr(const char *s1, const char *s2, const char delimiter)
{
  int result = 1;
  int eos = 1;
  
  for (int i=0; eos; i++) {
    char c1 = toupper(s1[i]);
    char c2 = toupper(s2[i]);
 
    if ((c1 == 0) || (c2 == 0) || (c1 == delimiter) || (c2 == delimiter))
       {
        eos = 0;
       }
      else if (c1 != c2)
        {
        result = 0;
        eos = 0;
        }  
          
  }
  return result;
}


int GetIntValue(const char *s, int number)
{
  int result = -1;
  char buf[16];
  int eos = 1;
  int digital = 1;
  
  int pct = 0;
  int posi = -1;
  if (number > 0){
   for (int i=0; (pct != number) && (s[i] != 0); i++)
    if (( s[i] == 0) || (s[i] == '/') || (s[i] == ':'))
    {
     pct ++;
     posi = i+1;
    }
      
   } 
  else 
    {
      posi = 0;
    }
  
  
 if (pct == number) {
  
  for (int i=0; (i<16) && eos; i++) {
    char c = s[i+posi];
    if (( c == 0) || (c == '/') || (c == ':'))
    {
      eos=0;
      buf[i]=0;
    }
    else
    {
     buf[i] = c;
     if ((isdigit(buf[i])==0) && (buf[i]!='-')) {digital = 0;}
    }
    
    
  }
  
  if (digital) {result = atoi(buf);}
  else {
     if (CmpStr(buf, "OFF", 0) )
     {
       result = 0;
     }
     
     if (CmpStr(buf, "ON", 0) )
     {
       result = 1;
     }
     
     if (CmpStr(buf, "LEFT", 0) )
     {
       result = 1;
     }
     
     if (CmpStr(buf, "RIGHT", 0) )
     {
       result = 2;
     }
     
     if (CmpStr(buf, "BOTH", 0) )
     {
       result = 3;
     }
     
  }
 }

  
  return result;
}


void TestMatrix(int addr)
{
  uint8_t MPins[] = {CPIN_01,CPIN_02,CPIN_03,CPIN_04,CPIN_05,CPIN_06,CPIN_07,CPIN_08,CPIN_09,0x60,0x00,0x68};
  static char PrintBuffer[32];
  
   for (int i=0; i<12; i++)
   for (int j=0; j<12; j++)
   {
     if (i!=j) { 
    ADG2128_ON(addr, MX_GND + MPins[i]);
    MBuffer[i][j] = GetRes(addr, MPins[j] ,PULL_1k);
    ADG2128_OFF(addr, MX_GND + MPins[i]);
    HAL_Delay(1);
     }
   }
  
  for (int i=0; i<12; i++)
   for (int j=0; j<i; j++)
   {
     if (abs(MBuffer[i][j]-MBuffer_old[i][j]) > 100) 
     {
       sprintf(PrintBuffer,"%i_%i -> %i_%i = %i  ",addr+1,i+1,addr+1,j+1,MBuffer[i][j]);
       PrintMsg(PrintBuffer);
       if (abs(MBuffer[i][j]-MBuffer[j][i])>100) {
       sprintf(PrintBuffer,"Opposite = %i  ",MBuffer[j][i]);
       PrintMsg(PrintBuffer);
       }
       
       
     }
   }
  
   for (int i=0; i<12; i++)
    for (int j=0; j<12; j++)
   {
     MBuffer_old[i][j] = MBuffer[i][j]; 
   }
  
  
  /*ADG2128_ON(addr, MX_GND + CPIN_11);
  sprintf(PrintBuffer,"11-12 = %i  ",GetRes(addr, CPIN_12 ,PULL_1k));
  ADG2128_OFF(addr, MX_GND + CPIN_11);
  PrintMsg(PrintBuffer);*/
}

void PresetControls(int addr, int type)
{
  if (type== CT_DAEWOO_LEV_L) {
  ADG2128_ON(addr, MX_GND + CPIN_02);
  ADG2128_ON(addr, MX_GND + CPIN_04);
  ADG2128_ON(addr, MX_GND + CPIN_12);  
  }
}

void PollControls(int addr, int type)
{
  
  if (type== CT_DAEWOO_LEV_L) {
  
  ADG2128_ON(addr, MX_GND + CPIN_02);
  ADG2128_ON(addr, MX_GND + CPIN_07);
  ADG2128_ON(addr, MX_GND + CPIN_08);  
  x_high_beam = PollR(addr, CPIN_05, PULL_1k, OnOff, 2) + PollR(addr, CPIN_04, PULL_1k, OnOff, 2)*2 ;
  x_low_beam  = PollR(addr, CPIN_11, PULL_1k, OnOff, 2);
  x_pos_light = PollR(addr, CPIN_09, PULL_1k, OnOff, 2);
  x_left   = PollR(addr, CPIN_03, PULL_1k, OnOff, 2);
  x_right  = PollR(addr, CPIN_01, PULL_1k, OnOff, 2);
  
  
 
  }
  
 
  if (type== CT_DAEWOO_LEV_R) {
  
  ADG2128_ON(addr, MX_GND + CPIN_03);
  ADG2128_ON(addr, MX_GND + CPIN_09);  
  x_wiper_f=0;
  if ( PollR(addr, CPIN_02, PULL_1k, OnOff, 2)) {x_wiper_f=1;}
  if ( PollR(addr, CPIN_01, PULL_1k, OnOff, 2)) {x_wiper_f=2;}
  if ( PollR(addr, CPIN_04, PULL_1k, OnOff, 2)) {x_wiper_f=3;}
  x_washer_f = PollR(addr, CPIN_11, PULL_1k, OnOff, 2);
  x_wiper_r  = PollR(addr, CPIN_05, PULL_1k, OnOff, 2);
  x_washer_r = PollR(addr, CPIN_07, PULL_1k, OnOff, 2);
  }
  
  if (type== CT_FIAT_CONTROLS) {
  
   //ADG2128_ON(addr, MX_GND + CPIN_11);
  ADG2128_ON(addr, MX_GND + CPIN_05);
  /* 
   int a12 = GetRes(addr,CPIN_12 ,PULL_1k);
   int a11 = GetRes(addr,CPIN_11 ,PULL_1k);
   int a10 = GetRes(addr,CPIN_10 ,PULL_1k);
   int a9 = GetRes(addr,CPIN_09 ,PULL_1k);
   int a8 = GetRes(addr,CPIN_08 ,PULL_1k);
   int a7 = GetRes(addr,CPIN_07 ,PULL_1k);
   int a6 = GetRes(addr,CPIN_06 ,PULL_1k);
   int a5 = GetRes(addr,CPIN_05 ,PULL_1k);
   int a4 = GetRes(addr,CPIN_04 ,PULL_1k);
   int a3 = GetRes(addr,CPIN_03 ,PULL_1k);
   int a2 = GetRes(addr,CPIN_02 ,PULL_1k);
   int a1 = GetRes(addr,CPIN_01 ,PULL_1k);
   
   x_washer_r = a12 + a11 + a10 + a9 + a8 + a7 + a6 + a5 + a4 + a3 + a2 + a1;
   
   x_washer_f = x_washer_r + GetRes(addr, CPIN_05 ,PULL_1k);
   */
   x_low_beam  = PollR(addr, CPIN_08, PULL_1k, FIPl, 2);   
   int resDir = PollR(addr, CPIN_12, PULL_1k, FIDir, 3);
   x_left = resDir == 1;
   x_right  = resDir == 0; 
   
   x_high_beam = GetRes(addr, CPIN_10 ,PULL_1k) < 2400;
   
   x_cc_set  = PollR(addr, CPIN_07, PULL_1k, OnOff, 2);
   
   ADG2128_ON(addr, MX_GND + CPIN_05);
   x_wiper_r = PollR(addr, CPIN_06, PULL_1k, OnOff, 2);
   int washers = PollR(addr, CPIN_04, PULL_1k, FIWash, 3);
   x_washer_f = washers == 1;
   x_washer_r = washers == 2;
   
  // ADG2128_OFF(addr, MX_GND + CPIN_05);
   
   x_wiper_f = PollR(addr, CPIN_11, PULL_1k, FIWs, 4);
   

  }
  
  
  if (type== CT_VW_LIGHT) {
  
  ADG2128_ON(addr, MX_GND + CPIN_01);
  ADG2128_ON(addr, MX_GND + CPIN_04);
  ADG2128_ON(addr, MX_GND + CPIN_05);  
  x_low_beam  = PollR(addr, CPIN_02, PULL_1k, OnOff, 2);
  x_pos_light = PollR(addr, CPIN_03, PULL_1k, OnOff, 2);
  x_rear_fog = PollR(addr, CPIN_07, PULL_1k, OnOff, 2);

  
  if (d_backlight_b) {
     ADG2128_ON(addr, MX_12V + CPIN_09);
     } else
     {
     ADG2128_OFF(addr, MX_12V + CPIN_09);
     } 
  
  
  }
  
  if (type== CT_VW_LEV_L) {
  
  ADG2128_ON(addr, MX_GND + CPIN_01);
  ADG2128_ON(addr, MX_GND + CPIN_06);
  ADG2128_ON(addr, MX_GND + CPIN_12);  
  x_high_beam = PollR(addr, CPIN_04, PULL_1k, OnOff, 2) + PollR(addr, CPIN_05, PULL_1k, OnOff, 2)*2 ;
  x_left   = PollR(addr, CPIN_03, PULL_1k, OnOff, 2);
  x_right  = PollR(addr, CPIN_02, PULL_1k, OnOff, 2);
  x_cruise  = PollR(addr, CPIN_11, PULL_1k, OnOff, 2);
  x_cc_set  = PollR(addr, CPIN_07, PULL_1k, OnOff, 2);
  x_cc_res  = PollR(addr, CPIN_09, PULL_1k, OnOff, 2);
  
  }
  
  
  if (type== CT_VW_LEV_R) {
    
  ADG2128_ON(addr, MX_GND + CPIN_01);
  ADG2128_ON(addr, MX_GND + CPIN_04);
  x_wiper_f=0;
  if ( PollR(addr, CPIN_07, PULL_1k, OnOff, 2)) {x_wiper_f=1;}
  if ( PollR(addr, CPIN_12, PULL_1k, OnOff, 2)) {x_wiper_f=2;}
  if ( PollR(addr, CPIN_08, PULL_1k, OnOff, 2)) {x_wiper_f=3;}
  ADG2128_ON(addr, MX_GND + CPIN_09);
  x_washer_f = PollR(addr, CPIN_11, PULL_1k, OnOff, 2);
  ADG2128_OFF(addr, MX_GND + CPIN_09);
  x_washer_r = PollR(addr, CPIN_09, PULL_1k, OnOff, 2);
  
  x_interval = PollR(addr, CPIN_02, PULL_50k, VWIntns, 4);

  x_reset = PollR(addr, CPIN_05, PULL_1k, OnOff, 2);
  x_up = PollR(addr, CPIN_06, PULL_1k, OnOff, 2);
  x_dn = PollR(addr, CPIN_03, PULL_1k, OnOff, 2);
  
  }
  
    
    //Autonomy section
  if (ClusterAutonomy == 1) 
  {
    d_backlight_c = x_pos_light;
    d_high_beam = x_high_beam;
    d_low_beam = x_low_beam;
    d_rear_fog = x_rear_fog;
    
    if ((x_left || x_right) != x_blink) {DirBlinkCt = 0;}
    x_blink = x_left || x_right;
    
    d_right = x_right && DirBlink;
    d_left  = x_left && DirBlink;
  }
}

void Blinker(void)
{
DirBlink = DirBlinkCt < 6;

if (DirBlinkCt++ > 14) {DirBlinkCt = 0;}
  
}

void USB_Print(unsigned char* buffer)

{
  USBD_CDC_SetTxBuffer(&USBD_Device, (unsigned char*)buffer, strlen((char const *)buffer));    
  while (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_BUSY) {};
  
}

static void RedLedBlast(uint32_t length)
{
  
  if (USB_started) {  
    RedLedTm = HAL_GetTick() + length;
    RedLed = 1;
    LED_ON(0);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
  //LED_SW(0);
  
}

void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  HandleAdcCycle();
}