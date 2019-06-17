#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "pcb20_defines.c"

extern uint8_t TVDS;
extern uint8_t TVHS;
extern uint32_t TVRR;

extern uint32_t ADct;

extern float CellVoltage;


static char OledBuffer[32];
static char MaskBuffer[32];
static int OledPage=0;             
static int OledSubPage=0;

extern int SpecMode;

extern int AD7193_CurrentChannel;
extern float AD7193_Zero[4];
extern float AD7193_Scale[4];
extern unsigned char AD7193_Gain[4];
extern unsigned char AD7193_Polarity[4];


static char MsgBuffer[8][32];

const static int MaxPages = Menu_Pages_Count;


static float CL_Speed();
static float CL_RPM();

static int ChValue=0;

extern char COM_In_Buffer[256];

extern float Temperature;
extern uint16_t Pedal_1;
extern uint16_t Pedal_2;
extern uint16_t Pedal_3;

extern uint16_t N_Pedal_1;
extern uint16_t N_Pedal_2;
extern uint16_t N_Pedal_3;

extern uint16_t Pedal_1_Lo;
extern uint16_t Pedal_1_Hi;
extern uint16_t Pedal_2_Lo;
extern uint16_t Pedal_2_Hi;
extern uint16_t Pedal_3_Lo;
extern uint16_t Pedal_3_Hi;

extern uint32_t d_trid;
extern uint32_t d_totd;

extern uint16_t Buttons;
extern int ct;
extern RTC_HandleTypeDef RtcHandle;

extern uint8_t CtrlTypes[4];
extern const int ControlsList[];
extern const int ClusterList[];
extern int ClusterAutonomy;
extern int SimulatorType;

extern int ADG2128_Inited[4];
extern int ADT7410_Inited;

extern int CC_GPIO[10];

extern int CTRL_SB;
extern int CTRL_BR;
extern int CTRL_IG;
extern int CTRL_ST;

extern int CTRL_SB_PIN;
extern int CTRL_BR_PIN;
extern int CTRL_IG_PIN;
extern int CTRL_ST_PIN;

extern int CTRL_SB_INV;
extern int CTRL_BR_INV;
extern int CTRL_IG_INV;
extern int CTRL_ST_INV;

extern int tval1;
extern int tval2;
extern int tval3;
extern int tval4;

extern int RtcEN;

extern int LCellEN;
extern int LEDDriverEN;

extern int OledInitEN;
extern int OledDimmerEN;
extern int ButtonsEN;
extern uint16_t PedFiltering;

extern int d_rpm;        //RPM
extern int d_speed;      //Speed in km/h
extern int d_fuel;      //Fuel level (%)
extern int d_temp;       //Temperature
extern int d_park_dst;  //Park distance 0-15
extern int d_odo; 
extern int d_high_beam;  //indicator LEDs
extern int d_low_beam;
extern int d_front_fog;
extern int d_rear_fog;
extern int d_left;
extern int d_right;
extern int d_emerg;
extern int d_brake;
extern int d_cluster_pwr;

extern int x_high_beam;  //controls LEDs
extern int x_low_beam;
extern int x_front_fog;
extern int x_rear_fog;
extern int x_pos_light;
extern int x_left;
extern int x_right;
extern int x_wiper_f;
extern int x_washer_f;
extern int x_wiper_r;
extern int x_washer_r;
extern int x_cruise;
extern int x_cc_set;
extern int x_cc_res;
extern int x_interval;
extern int x_reset;
extern int x_up;
extern int x_dn;


extern int TurnDelay;
extern int TurnPeriod;
extern int TurnON;

extern int LED_Boards;
extern int ActiveLEDs;

extern int CELL_Board;

static void UpdateOled(void);
static void MenuHeader(char* buf,int numpage);
static void CtlPrint(char* ccode, int c_pos, int a1_pos, int a2_pos, int a3_pos, int a4_pos, int active);

static void PrintMsg(char* msg);
static char* ControlName(uint8_t index);
static char* ClusterName(uint8_t index);
static char* SimulatorName(uint8_t index);
static char* PedName(uint8_t index);
static char* MsgOnOff(int msg);
static char* MsgNoNc(int msg);

extern int ClusterType;

extern int PedType;

static void BtnLtPress(); 
static void BtnRtPress(); 
static void BtnOkPress(); 
static void BtnMenuPress(); 



extern int xresult;

extern int eeprom_ID;

extern float Vbat;
extern int TempEN;


void UpdateOled(void)
{
 if (SpecMode==0) 
 {
 switch ( OledPage ) {
  case Menu_General_Info:                                                       //General info
  MenuHeader("General Info         ",Menu_General_Info);
  sprintf(OledBuffer,"Serial Number: %04x     ", eeprom_ID);
  OLED_Print(OledBuffer,2);
  OLED_Print("Firmware: "FW_VERSION"       ",3);
  sprintf(OledBuffer,"Controls: ----         ");
  for (int i=0; i<4; i++) {if (ADG2128_Inited[i]) {OledBuffer[10+i]='1'+i; }}
  if (ButtonsEN == 0) {OledBuffer[20] = 1;}
  OLED_Print(OledBuffer,4);
 OLED_Separator(0,5);
  //sprintf(OledBuffer,"-DS:%i--HS:%i--RR:%i--      ",TVDS,TVHS,TVRR);
  //for (int i=0; i<21; i++) {OledBuffer[i] = i+235;}
 // sprintf(OledBuffer,"%i %i %i %i           ",ADC1_Buffer[0],ADC1_Buffer[2],ADC1_Buffer[4],ADC1_Buffer[6]);
  //OLED_Print(OledBuffer,5);   
  sprintf(OledBuffer,"Temperature: %.1f C        ", Temperature);
  if (TempEN == 0) {OledBuffer[19]='*';}
  
  OLED_fPrint(OledBuffer,"UUUUUUUU------------",6);  
  if (RtcEN) { 
  RTC_TimeTypeDef stimestructureget;
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);
  RTC_DateTypeDef sdatestructureget;
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  
  sprintf(OledBuffer,"%02d:%02d:%02d   %02d/%02d/%02d    ",
                     stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds
                    ,sdatestructureget.Date, sdatestructureget.Month, 2000 + sdatestructureget.Year);
  } else {sprintf(OledBuffer,"                      ");}
  
  OLED_fPrint(OledBuffer,"IIIIIIII------------",7);
  
  
  break;
  
    case Menu_Controls_Status:                                                            //Controls status
    MenuHeader("Controls         ",Menu_Controls_Status);
    
   
    sprintf(MaskBuffer,"                     ");
    if (x_left)         {MaskBuffer[0] = MaskBuffer[1] = 'I';}
    if (x_right)        {MaskBuffer[3] = MaskBuffer[4] = 'I';}
    if (x_pos_light)     {MaskBuffer[6] = MaskBuffer[7] = 'I';}
    if (x_low_beam)    {MaskBuffer[9] = MaskBuffer[10] = 'I';}
    if (x_high_beam)    {MaskBuffer[12] = MaskBuffer[13] = 'I';}
    if (x_front_fog)     {MaskBuffer[15] = MaskBuffer[16] = 'I';}
    if (x_rear_fog)        {MaskBuffer[18] = MaskBuffer[19] = 'I';}
    OLED_fPrint("<L R> PL LB HB FF RF",MaskBuffer,2);
    
    sprintf(MaskBuffer,"                     ");
    char s[2];
    switch (x_wiper_f)
    {
    case 0:
      strcpy(s,"--");
      break;
    case 1:
      strcpy(s,"IN");
      break;
    case 2:
      strcpy(s,"LO");
      break;
    case 3:
      strcpy(s,"HI");
      break;
    }
    if (x_wiper_f)  {MaskBuffer[4] = MaskBuffer[5] = 'I';}
    if (x_washer_f)  {MaskBuffer[7] = MaskBuffer[8] = MaskBuffer[9] = 'I';}
    if (x_wiper_r)  {MaskBuffer[11] = MaskBuffer[12] = MaskBuffer[13] = 'I';}
    if (x_washer_r)  {MaskBuffer[15] = MaskBuffer[16] = MaskBuffer[17] = 'I';}
   
    
    sprintf(OledBuffer,"WpF:%s WsF WpR WsR     ", s);
     if (x_interval!=-1) {OledBuffer[19] = '1'+x_interval;}
    OLED_fPrint(OledBuffer,MaskBuffer,3);
    
    //OLED_Print("                     ",3);
    
    sprintf(MaskBuffer,"                     ");
    if (x_cruise)         {MaskBuffer[0] = MaskBuffer[1]= MaskBuffer[2]= MaskBuffer[3]= MaskBuffer[4]= MaskBuffer[5] = 'I';}
    if (x_cc_set)        {MaskBuffer[7] = MaskBuffer[8]= MaskBuffer[9] = 'I';}
    if (x_cc_res)     {MaskBuffer[11] = MaskBuffer[12]= MaskBuffer[13] = 'I';}
    OLED_fPrint("Cruise SET RES      ",MaskBuffer,4);

    //OLED_Print("                     ",4);
   
    sprintf(MaskBuffer,"                     ");
    if (x_reset)         {MaskBuffer[0] = MaskBuffer[1]= 'I';}
    if (x_up)        {MaskBuffer[3] = MaskBuffer[4] = 'I';}
    if (x_dn)     {MaskBuffer[6] = MaskBuffer[7]= MaskBuffer[8]= MaskBuffer[9]= MaskBuffer[10] = 'I';}
    OLED_fPrint("Up Dn Reset         ",MaskBuffer,5);
   
   
   
   
   
   
   
   // OLED_Print("                     ",5);
    
    
    
    
    sprintf(MaskBuffer,"                     ");
    if (CTRL_SB) {MaskBuffer[0] = MaskBuffer[1] = 'I';}
    if (CTRL_BR) {MaskBuffer[3] = MaskBuffer[4] = 'I';}
    if (CTRL_IG) {MaskBuffer[6] = MaskBuffer[7] = 'I';}
    if (CTRL_ST) {MaskBuffer[9] = MaskBuffer[10] = 'I';}
    OLED_fPrint("SB BR IG ST          ",MaskBuffer,6);
    
    
    OLED_Print("                     ",7);
    break;
  
    case Menu_Cluster_Status:                                                            //Cluster status
    MenuHeader("Cluster              ",Menu_Cluster_Status);
    sprintf(OledBuffer,"Speed:%03d Tacho:%04d  ",d_speed, d_rpm);
    OLED_Print(OledBuffer,2);   
    sprintf(OledBuffer," Fuel:%03d  Temp:%03d  ",d_fuel, d_temp);
    OLED_Print(OledBuffer,3);   
    
    sprintf(MaskBuffer,"                     ");
    if (d_left)         {MaskBuffer[0] = MaskBuffer[1] = 'I';}
    if (d_right)        {MaskBuffer[3] = MaskBuffer[4] = 'I';}
    if (d_low_beam)     {MaskBuffer[6] = MaskBuffer[7] = 'I';}
    if (d_high_beam)    {MaskBuffer[9] = MaskBuffer[10] = 'I';}
    if (d_front_fog)    {MaskBuffer[12] = MaskBuffer[13] = 'I';}
    if (d_rear_fog)     {MaskBuffer[15] = MaskBuffer[16] = 'I';}
    if (d_brake)        {MaskBuffer[18] = MaskBuffer[19] = 'I';}
    OLED_fPrint("<L R> LB HB FF RF BR",MaskBuffer,4);
    //OLED_Print("                     ",5);
    sprintf(OledBuffer,"O:%08d T:%08d ",d_totd, d_trid);
    OLED_Print(OledBuffer,5);   
    OLED_Print("                     ",6);
    
    sprintf(MaskBuffer,"                     ");
    if (d_cluster_pwr)         {MaskBuffer[0] = MaskBuffer[1] = MaskBuffer[2] = 'I';}
    OLED_fPrint("PWR BL               ",MaskBuffer,7);
    break;
    
    case Menu_GPIO_In:                                                            //GPIO in
    MenuHeader("GPIO status           ",Menu_GPIO_In);
    sprintf(OledBuffer,"                     ");
    sprintf(MaskBuffer,"                     ");
    for (int i=0; i<10; i++) {
     OledBuffer[i*2+1] = 'A'+i; 
     if (CC_GPIO[i]) {MaskBuffer[i*2+1] = 'I';}
     }
    
    OLED_fPrint(OledBuffer,MaskBuffer,2);
    CtlPrint("  ", -1, CTRL_SB_PIN, CTRL_BR_PIN, CTRL_IG_PIN, CTRL_ST_PIN, 0);
    OLED_fPrint(OledBuffer,MaskBuffer,3);
    CtlPrint("SB", CTRL_SB_PIN, CTRL_BR_PIN, CTRL_IG_PIN, CTRL_ST_PIN, -1, CTRL_SB);
    OLED_fPrint(OledBuffer,MaskBuffer,4);
    CtlPrint("BR", CTRL_BR_PIN, CTRL_IG_PIN, CTRL_ST_PIN, -1, -1 , CTRL_BR);
    OLED_fPrint(OledBuffer,MaskBuffer,5);
    CtlPrint("IG", CTRL_IG_PIN, CTRL_ST_PIN, -1, -1, -1 , CTRL_IG);
    OLED_fPrint(OledBuffer,MaskBuffer,6);
    CtlPrint("ST", CTRL_ST_PIN, -1, -1, -1, -1 , CTRL_ST);
    OLED_fPrint(OledBuffer,MaskBuffer,7);
    break;
  
    case Menu_Pedals_Status:                                                    //Pedals status
    MenuHeader("Pedals status          ",Menu_Pedals_Status);
    
    OLED_Gauge(0,N_Pedal_1,0,1024,'F',2,0);
    OLED_Gauge(0,N_Pedal_2,0,1024,'B',4,0);
    OLED_Gauge(0,N_Pedal_3,0,1024,'A',6,0);
    OLED_Gauge(0,Pedal_1,0,4096,' ',3,1);
    OLED_Gauge(0,Pedal_2,0,4096,' ',5,1);
    OLED_Gauge(0,Pedal_3,0,4096,' ',7,1);
    break;  
  
    case Menu_Controls_Setup:                                                   //Controls setup
    MenuHeader("Controls setup       ",Menu_Controls_Setup);
    
    for (int i=0; i<4; i++) 
    {

   
      if (OledSubPage != (i+1)) {
      sprintf(OledBuffer,"%i: %s             ",i+1,ControlName(CtrlTypes[i]) );
      OLED_Print(OledBuffer,2+i); 
      } else {
        
      sprintf(OledBuffer,"%i: %s             ",i+1,ControlName(ControlsList[ChValue]));  
      sprintf(MaskBuffer,"   IIIIIIIIIIIIIIIII ");
      OLED_fPrint(OledBuffer,MaskBuffer,2+i);    
      }
      
      
    }
    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
  
    case Menu_Cluster_Setup:                                                    //Cluster setup
    MenuHeader("Cluster setup        ",Menu_Cluster_Setup);
    
    if (OledSubPage != 1){
      
      sprintf(OledBuffer,"Type: %s           ",ClusterName(ClusterList[ClusterType]) );
      OLED_Print(OledBuffer,2); 
    }
     else
     {
     sprintf(OledBuffer,"Type: %s           ",ClusterName(ClusterList[ChValue]));  
     sprintf(MaskBuffer,"      IIIIIIIIIIIIIII");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     }
    
     if (OledSubPage != 2){
      
      sprintf(OledBuffer,"Autonomy: %s        ",MsgOnOff(ClusterAutonomy) );
      OLED_Print(OledBuffer,3); 
    }
     else
     {
     sprintf(OledBuffer,"Autonomy: %s         ",MsgOnOff(ChValue));  
     sprintf(MaskBuffer,"          IIII       ");
      OLED_fPrint(OledBuffer,MaskBuffer,3);  
     } 

    OLED_Print("                     ",4);
    OLED_Print("                     ",5);
    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
    
    case Menu_Turn_Setup:                                                       //Turn setup
    MenuHeader("Turn indicator       ",Menu_Turn_Setup);
    
    if (OledSubPage != 1){
    sprintf(OledBuffer,"Delay:   %i ms         ",TurnDelay*50);
    OLED_Print(OledBuffer,2);  
    }
     else
     {
     sprintf(OledBuffer,"Delay:   %i ms         ",ChValue*50);  
     sprintf(MaskBuffer,"         IIIIIII        ");
     OLED_fPrint(OledBuffer,MaskBuffer,2);  
     } 
    
    
    if (OledSubPage != 2){
    sprintf(OledBuffer,"Period:  %i ms         ",TurnPeriod*50);
    OLED_Print(OledBuffer,3);  
    }
    else
     {
     sprintf(OledBuffer,"Period:  %i ms         ",ChValue*50);  
     sprintf(MaskBuffer,"         IIIIIII        ");
     OLED_fPrint(OledBuffer,MaskBuffer,3);  
     } 
    
    if (OledSubPage != 3){
    sprintf(OledBuffer,"ON time: %i ms         ",TurnON*50);
    OLED_Print(OledBuffer,4);  
    }
    else
     {
     sprintf(OledBuffer,"ON time: %i ms         ",ChValue*50);  
     sprintf(MaskBuffer,"         IIIIIII        ");
     OLED_fPrint(OledBuffer,MaskBuffer,4);  
     } 
    
    
    
    OLED_Print("                     ",5);
    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
    case Menu_GPIO_Setup_1:                                                     //GPIO setup 1
    MenuHeader("GPIO setup 1     ",Menu_GPIO_Setup_1);
    
      sprintf(OledBuffer,"SeatBelt: GPIO_X        ");
      if (OledSubPage != 1){
      OledBuffer[15]='A'+CTRL_SB_PIN;      
      OLED_Print(OledBuffer,2); 
    }
     else
     {
     OledBuffer[15]='A'+ChValue;  
      sprintf(MaskBuffer,"          IIIIII       ");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     } 
     
      sprintf(OledBuffer,"Brake:    GPIO_X        ");
      if (OledSubPage != 2){
      OledBuffer[15]='A'+CTRL_BR_PIN;      
      OLED_Print(OledBuffer,3); 
    }
     else
     {
     OledBuffer[15]='A'+ChValue;  
     sprintf(MaskBuffer,"          IIIIII       ");
      OLED_fPrint(OledBuffer,MaskBuffer,3);  
     } 
     
      sprintf(OledBuffer,"Ignition: GPIO_X        ");
      if (OledSubPage != 3){
      OledBuffer[15]='A'+CTRL_IG_PIN;      
      OLED_Print(OledBuffer,4); 
    }
     else
     {
     OledBuffer[15]='A'+ChValue;  
     sprintf(MaskBuffer,"          IIIIII       ");
      OLED_fPrint(OledBuffer,MaskBuffer,4);  
     } 
     
     sprintf(OledBuffer,"Starter:  GPIO_X        ");
      if (OledSubPage != 4){
      OledBuffer[15]='A'+CTRL_ST_PIN;      
      OLED_Print(OledBuffer,5); 
    }
     else
     {
     OledBuffer[15]='A'+ChValue;  
     sprintf(MaskBuffer,"          IIIIII       ");
      OLED_fPrint(OledBuffer,MaskBuffer,5);  
     } 
     
     
     

    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
    
    
    case Menu_GPIO_Setup_2:                                                     //GPIO setup 2
    MenuHeader("GPIO setup 2     ",Menu_GPIO_Setup_2);
    
      
      if (OledSubPage != 1){
      sprintf(OledBuffer,"SeatBelt: %s      ",MsgNoNc(CTRL_SB_INV));
      OLED_Print(OledBuffer,2); 
    }
     else
     {
      sprintf(OledBuffer,"SeatBelt: %s        ",MsgNoNc(ChValue));
      sprintf(MaskBuffer,"          IIIIIIIIII ");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     } 
     
      if (OledSubPage != 2){
      sprintf(OledBuffer,"Brake:    %s      ",MsgNoNc(CTRL_BR_INV));
      OLED_Print(OledBuffer,3); 
    }
     else
     {
      sprintf(OledBuffer,"Brake:    %s        ",MsgNoNc(ChValue));
      sprintf(MaskBuffer,"          IIIIIIIIII ");
      OLED_fPrint(OledBuffer,MaskBuffer,3);  
     }
              
              
     if (OledSubPage != 3){
      sprintf(OledBuffer,"Ignition: %s        ",MsgNoNc(CTRL_IG_INV));
      OLED_Print(OledBuffer,4); 
    }
     else
     {
      sprintf(OledBuffer,"Ignition: %s      ",MsgNoNc(ChValue));
      sprintf(MaskBuffer,"          IIIIIIIIII ");
      OLED_fPrint(OledBuffer,MaskBuffer,4);  
     } 
     
     if (OledSubPage != 4){
      sprintf(OledBuffer,"Starter:  %s      ",MsgNoNc(CTRL_ST_INV));
      OLED_Print(OledBuffer,5); 
    }
     else
     {
      sprintf(OledBuffer,"Starter:  %s       ",MsgNoNc(ChValue));
      sprintf(MaskBuffer,"          IIIIIIIIII ");
      OLED_fPrint(OledBuffer,MaskBuffer,5);  
     } 
     
     

    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
    case Menu_Simulator:                                                        //Simulator
    MenuHeader("Simulator            ",Menu_Simulator);
    
    if (OledSubPage != 1){
      
      sprintf(OledBuffer,"Type: %s           ",SimulatorName(SimulatorList[SimulatorType]) );
      OLED_Print(OledBuffer,2); 
    }
     else
     {
     sprintf(OledBuffer,"Type: %s           ",SimulatorName(SimulatorList[ChValue]));  
     sprintf(MaskBuffer,"      IIIIIIIIIIIIIII");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     }
    OLED_Print("                     ",3);
    OLED_Print("                     ",4);
    OLED_Print("                     ",4);
    OLED_Print("                     ",5);
    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
     case Menu_Pedals_Setup:                                                    //Pedals setup 
    MenuHeader("Pedals setup         ",Menu_Pedals_Setup);
    
    if (OledSubPage != 1){
      
      sprintf(OledBuffer,"Type: %s           ",PedName(PedList[PedType]) );
      OLED_Print(OledBuffer,2); 
    }
     else
     {
     sprintf(OledBuffer,"Type: %s           ",PedName(PedList[ChValue]));  
     sprintf(MaskBuffer,"      IIIIIIIIIIIIIII");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     }
     if (OledSubPage != 2){
      
      sprintf(OledBuffer,"Calibrate            ");
      OLED_Print(OledBuffer,3); 
    }
     else
     {
     sprintf(OledBuffer,"Calibrate            ");
     sprintf(MaskBuffer,"IIIIIIIII            ");
      OLED_fPrint(OledBuffer,MaskBuffer,3);  
     }
    
     if (OledSubPage != 3){
      
      sprintf(OledBuffer,"Filtering: %i         ",PedFiltering);
      OLED_Print(OledBuffer,4); 
    }
     else
     {
     sprintf(OledBuffer,"Filtering: %i         ",ChValue);  
     sprintf(MaskBuffer,"           III        ");
      OLED_fPrint(OledBuffer,MaskBuffer,4);  
     }
 
    //OLED_Print("                     ",4);
    //OLED_Print("                     ",4);
    OLED_Print("                     ",5);
    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
    case Menu_LCell_Status:                                                     //Load cell status
    MenuHeader("Load cells             ",Menu_LCell_Status);
    
    if (CELL_Board == -1) {
    OLED_Print("ADC Board not found  ",2);
    OLED_Print("                     ",3);
    OLED_Print("                     ",4);
    OLED_Print("                     ",5);
    OLED_Print("                     ",6);
    OLED_Print("                     ",7);    
    } else
    {
    OLED_Print("ADC ok               ",2);

    sprintf(OledBuffer,"Channel: %d              ",AD7193_CurrentChannel+1 );
      OLED_Print(OledBuffer,3);  
    sprintf(OledBuffer,"Voltage: %.5f            ",CellVoltage );
      OLED_Print(OledBuffer,4);  
    sprintf(OledBuffer,"Zero: %.5f               ",AD7193_Zero[AD7193_CurrentChannel] );
      OLED_Print(OledBuffer,5);    
    sprintf(OledBuffer,"Scale: %.5f              ",AD7193_Scale[AD7193_CurrentChannel] );
      OLED_Print(OledBuffer,6);      
      sprintf(OledBuffer,"Gain: %d, polarity %d  ",AD7193_Gain[AD7193_CurrentChannel], AD7193_Polarity[AD7193_CurrentChannel]);
      OLED_Print(OledBuffer,7);    
 
    }   
      
    break;
    case Menu_LCell_Setup:                                                     //Load cell setup
    MenuHeader("Load cells setup     ",Menu_LCell_Setup);
    //OLED_Print("                     ",2);
    if (OledSubPage != 1){
      
      sprintf(OledBuffer,"Enabled: %s         ",MsgOnOff(LCellEN) );
      OLED_Print(OledBuffer,2); 
    }
     else
     {
     sprintf(OledBuffer,"Enabled: %s          ",MsgOnOff(ChValue));  
     sprintf(MaskBuffer,"         IIII       ");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     } 
    
    OLED_Print("                     ",3);
    OLED_Print("                     ",4);
    OLED_Print("                     ",5);
    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break; 
    case Menu_LED_Status:                                                       //LED status
    MenuHeader("LED driver           ",Menu_LED_Status);
    sprintf(OledBuffer,"Boards found: %d      ",LED_Boards );
    OLED_Print(OledBuffer,2); 
    sprintf(OledBuffer,"LEDs on: %d           ",ActiveLEDs );
    OLED_Print(OledBuffer,3); 

    OLED_Print("                     ",4);
    OLED_Print("                     ",5);
    OLED_Print("                     ",6);
    OLED_Print("                     ",7);    
    break;
    case Menu_LED_Setup:                                                        //LED setup
    MenuHeader("LED setup            ",Menu_LED_Setup);
    //OLED_Print("                     ",2);
    if (OledSubPage != 1){
      
      sprintf(OledBuffer,"Enabled: %s         ",MsgOnOff(LEDDriverEN) );
      OLED_Print(OledBuffer,2); 
    }
     else
     {
     sprintf(OledBuffer,"Enabled: %s          ",MsgOnOff(ChValue));  
     sprintf(MaskBuffer,"         IIII       ");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     } 
    
    OLED_Print("                     ",3);
    OLED_Print("                     ",4);
    OLED_Print("                     ",5);
    OLED_Separator(0,6);
    if (OledSubPage == 0){ OLED_fPrint("Press MENU to change ",
                                       "------IIII-----------",7);}
                        else
                         { OLED_fPrint(" OK save \xB2 MENU skip ",
                                       "-II--------IIII------",7);} 
    break;
    case Menu_Board_Setup:                                                       //Board setup
    MenuHeader("Board                ",Menu_Board_Setup);
   
    
    if (OledSubPage != 1){                         
      sprintf(OledBuffer,"Oled on init: %s    ",MsgOnOff(OledInitEN) );
      OLED_Print(OledBuffer,2); 
    }
     else
     {          
     sprintf(OledBuffer,"Oled on init: %s    ",MsgOnOff(ChValue));  
     sprintf(MaskBuffer,"              IIII  ");
      OLED_fPrint(OledBuffer,MaskBuffer,2);  
     }
    if (OledSubPage != 2){  
      
      sprintf(OledBuffer,"Oled dimmer: %s      ",MsgOnOff(OledDimmerEN) );
      OLED_Print(OledBuffer,3); 
    }
     else
     {      
     sprintf(OledBuffer,"Oled dimmer: %s      ",MsgOnOff(ChValue));  
     sprintf(MaskBuffer,"             IIII    ");
      OLED_fPrint(OledBuffer,MaskBuffer,3);  
     }   

    if (OledSubPage != 3){  
      
      sprintf(OledBuffer,"Buttons enabled: %s    ",MsgOnOff(ButtonsEN) );
      OLED_Print(OledBuffer,4); 
    }
     else
     {      
     sprintf(OledBuffer,"Buttons enabled: %s    ",MsgOnOff(ChValue));  
     sprintf(MaskBuffer,"                 IIII  ");
      OLED_fPrint(OledBuffer,MaskBuffer,4);  
     }       
    
    
    OLED_Print("                     ",5);
    OLED_Separator(0,6);  
    sprintf(OledBuffer,"Vbat: %.2f V         ",Vbat );
    OLED_Print(OledBuffer,7); 
    break;
 }
 }
 
  if (SpecMode==1) 
 {
   for (int i=0; i<8; i++) {
   OLED_Print(MsgBuffer[i],i);
   }   
 }
 
   if (SpecMode==2) 
 {
  OLED_Print("  Calibration Mode   ",0); 
  OLED_Separator(0,1);
  OLED_Gauge(Pedal_1_Lo,Pedal_1_Hi,0,4096,'F',2,0);
  OLED_Gauge(Pedal_2_Lo,Pedal_2_Hi,0,4096,'B',3,0);
  OLED_Gauge(Pedal_3_Lo,Pedal_3_Hi,0,4096,'A',4,0);
  OLED_Separator(0,5);
  OLED_fPrint("Press OK to confirm   ",
              "------II-------------",6);
  OLED_fPrint("Press MENU to cancel ",
              "------IIII-----------",7);
 }
 
}  

void MenuHeader(char* buf,int numpage)
{
  sprintf(OledBuffer,"%.*s\xB2\x11 %02i \x10",14,buf, numpage+1);
  if (OledPage == 0) {OledBuffer[15] = ' ';}
  if (OledPage == MaxPages) {OledBuffer[20] = ' ';}
  
  OLED_Print(OledBuffer,0);
  OLED_Separator(1,1);
}

void CtlPrint(char* ccode, int c_pos, int a1_pos, int a2_pos, int a3_pos, int a4_pos, int active)
{
    sprintf(OledBuffer,"                     ");
    sprintf(MaskBuffer,"                     ");
    if (a1_pos >= 0) {OledBuffer[a1_pos*2+1]='|';} 
    if (a2_pos >= 0) {OledBuffer[a2_pos*2+1]='|';} 
    if (a3_pos >= 0) {OledBuffer[a3_pos*2+1]='|';} 
    if (a4_pos >= 0) {OledBuffer[a4_pos*2+1]='|';} 
    
    if (c_pos >= 0) {OledBuffer[c_pos*2+1]=ccode[0];
                     OledBuffer[c_pos*2+2]=ccode[1]; 
                     if (active) {
                     MaskBuffer[c_pos*2+1]='I';
                     MaskBuffer[c_pos*2+2]='I'; 
                     
                     
                     
                     }
    
    
                    } 
    
}  

void BtnMenuPress()
{
  if (SpecMode==0) {   
switch ( OledPage ) {
  case Menu_Controls_Setup:                                                     // Controls setup
   if (OledSubPage++<4)
   {
     for (int i=0; i<=CONTROL_ITEMS; i++)
     {
       if(CtrlTypes[OledSubPage-1] == ControlsList[i]) 
       {
         ChValue = i;
         break;
       }
     }
    } else
   {
   OledSubPage = 0;   
   }     
  break;  
  case Menu_Cluster_Setup:                                                      // Cluster status 
    switch (++OledSubPage)
    {
      case 1:
      for (int i=0; i<=CLUSTER_ITEMS; i++)
      {
       if(ClusterType == ClusterList[i]) 
       {
         ChValue = i;
         break;
       }
      } 
        
      break;
      case 2:
      ChValue = ClusterAutonomy; 
      
      break;
      case 3:
      OledSubPage = 0;
      break;
    }
     break; 
    
    case Menu_Turn_Setup:                                                       // Turn
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = TurnDelay;
        break;
      case 2: 
        ChValue = TurnPeriod;
        break;
      case 3: 
        ChValue = TurnON;
        break;
      case 4: 
        OledSubPage = 0;
        break; 
    }
     break; 
    
   case Menu_GPIO_Setup_1:                                                      // GPIO_SETUP_1
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = CTRL_SB_PIN;
        break;
      case 2: 
        ChValue = CTRL_BR_PIN;
        break;
      case 3: 
        ChValue = CTRL_IG_PIN;
        break;
      case 4: 
        ChValue = CTRL_ST_PIN;
        break;
      case 5: 
        OledSubPage = 0;
        break; 
    }
     break; 
    case Menu_GPIO_Setup_2:                                                     // GPIO_SETUP_2
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = CTRL_SB_INV;
        break;
      case 2: 
        ChValue = CTRL_BR_INV;
        break;
      case 3: 
        ChValue = CTRL_IG_INV;
        break;
      case 4: 
        ChValue = CTRL_ST_INV;
        break;
      case 5: 
        OledSubPage = 0;
        break; 
    }
     break; 
    case Menu_Simulator:                                                        // SIMULATOR
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = SimulatorType;
        break;
      case 2: 
        OledSubPage = 0;
        break; 
    }
     break; 
     case Menu_Pedals_Setup:                                                    // Pedal setup
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = PedType;
        break;
      case 2: 
        break; 
      case 3: 
        ChValue = PedFiltering;
        break;  
      case 4: 
        OledSubPage = 0;
        break;   
    }
     break; 
    case Menu_LCell_Setup:                                                      // Load cell setup
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = LCellEN;
        break;
      case 2: 
        OledSubPage = 0;
        break; 
    }
    break; 
    case Menu_LED_Setup:                                                        // LED driver setup
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = LEDDriverEN;
        break;
      case 2: 
        OledSubPage = 0;
        break; 
    }
    break; 
    
    case Menu_Board_Setup:                                                      //  Board Setup
    switch (++OledSubPage)
    {
      case 1: 
        ChValue = OledInitEN;
        break;
      case 2: 
        ChValue = OledDimmerEN;
        break;  
      case 3: 
        ChValue = ButtonsEN;
        break;  
      case 4: 
        OledSubPage = 0;
        break; 
    }
    break; 
    
  }
 } 

if (SpecMode==2) 
   {   
    SpecMode=0;
     //Load calibration defaults
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
   }
}

void BtnOkPress()
{
  
    if (SpecMode==2) 
   {   
    SpecMode=0;
     //Save calibration
    EEPROM_WriteW(EE_Pedal_1_Lo,Pedal_1_Lo);
    HAL_Delay(5);
    EEPROM_WriteW(EE_Pedal_1_Hi,Pedal_1_Hi);
    HAL_Delay(5);
    EEPROM_WriteW(EE_Pedal_2_Lo,Pedal_2_Lo);
    HAL_Delay(5);
    EEPROM_WriteW(EE_Pedal_2_Hi,Pedal_2_Hi);
    HAL_Delay(5);
    EEPROM_WriteW(EE_Pedal_3_Lo,Pedal_3_Lo);
    HAL_Delay(5);
    EEPROM_WriteW(EE_Pedal_3_Hi,Pedal_3_Hi);
    HAL_Delay(5);
   } 
   
   
   if (SpecMode==0) {    

  
    if ((OledPage==Menu_Controls_Setup)&&(OledSubPage!=0))
  {
    CtrlTypes[OledSubPage-1] = ControlsList[ChValue];
    
   switch (OledSubPage-1)
    {
      case 0:  
      EEPROM_WriteB(EE_CtrlTypes0, CtrlTypes[0]);
      break;
      case 1:  
      EEPROM_WriteB(EE_CtrlTypes1, CtrlTypes[1]);
      break;
      case 2:  
      EEPROM_WriteB(EE_CtrlTypes2, CtrlTypes[2]);
      break;
      case 3:  
      EEPROM_WriteB(EE_CtrlTypes3, CtrlTypes[3]);
      break;
    }
   OledSubPage = 0;
  }
  
  if ((OledPage==Menu_Cluster_Setup)&&(OledSubPage==1))
  {
   ClusterType = ClusterList[ChValue];
   EEPROM_WriteB(EE_ClusterType, ClusterType);
   OledSubPage = 0;
  }
  
  if ((OledPage==Menu_Cluster_Setup)&&(OledSubPage==2))
  {
   ClusterAutonomy = ChValue;
   EEPROM_WriteB(EE_ClusterAutonomy, ClusterAutonomy);
   OledSubPage = 0;
  }
  
  if ((OledPage==Menu_Turn_Setup)&&(OledSubPage!=0))
  {
    switch (OledSubPage)
    {
      case 1: 
        TurnDelay = ChValue ;
        EEPROM_WriteB(EE_TurnDelay,ChValue);
        break;
      case 2: 
        TurnPeriod = ChValue ;
         EEPROM_WriteB(EE_TurnPeriod,ChValue);
        break;
      case 3: 
         TurnON = ChValue ;
         EEPROM_WriteB(EE_TurnON,ChValue);
        break;

    }
    OledSubPage = 0;
  }
  
  
  if ((OledPage==Menu_GPIO_Setup_1)&&(OledSubPage!=0))
  {
     switch (OledSubPage)
    {
      case 1: 
        CTRL_SB_PIN = ChValue ;
        EEPROM_WriteB(EE_CTRL_SB_PIN, CTRL_SB_PIN);
        break;
      case 2: 
        CTRL_BR_PIN = ChValue ;
        EEPROM_WriteB(EE_CTRL_BR_PIN, CTRL_BR_PIN);
        break;
      case 3: 
        CTRL_IG_PIN = ChValue ;
        EEPROM_WriteB(EE_CTRL_IG_PIN, CTRL_IG_PIN);
        break;
      case 4: 
        CTRL_ST_PIN = ChValue ;
        EEPROM_WriteB(EE_CTRL_ST_PIN, CTRL_ST_PIN);
        break;
    }
      OledSubPage = 0;
    
  }
  
    if ((OledPage==Menu_GPIO_Setup_2)&&(OledSubPage!=0))
  {
     switch (OledSubPage)
    {
      case 1: 
        CTRL_SB_INV = ChValue ;
        EEPROM_WriteB(EE_CTRL_SB_INV, CTRL_SB_INV);
        break;
      case 2: 
        CTRL_BR_INV = ChValue ;
        EEPROM_WriteB(EE_CTRL_BR_INV, CTRL_BR_INV);
        break;
      case 3: 
        CTRL_IG_INV = ChValue ;
        EEPROM_WriteB(EE_CTRL_IG_INV, CTRL_IG_INV);
        break;
      case 4: 
        CTRL_ST_INV = ChValue ;
        EEPROM_WriteB(EE_CTRL_ST_INV, CTRL_ST_INV);
        break;
    }
      OledSubPage = 0;
    
  }
      if ((OledPage==Menu_Simulator)&&(OledSubPage!=0))
  {
     switch (OledSubPage)
    {
      case 1: 
        SimulatorType = ChValue ;
        EEPROM_WriteB(EE_SimulatorType, SimulatorType);
        break;
    }
      OledSubPage = 0;
    
  }
  
    if ((OledPage==Menu_Pedals_Setup)&&(OledSubPage!=0))
  {
     switch (OledSubPage)
    {
      case 1: 
        PedType = ChValue ;
        EEPROM_WriteB(EE_PedType, PedType);
        break;
      case 2: 
        SpecMode = 2;
        
        Pedal_1_Lo = Pedal_1;
        Pedal_1_Hi = Pedal_1;
        Pedal_2_Lo = Pedal_2;
        Pedal_2_Hi = Pedal_2;
        Pedal_3_Lo = Pedal_3;
        Pedal_3_Hi = Pedal_3;        
        
        break;  
      case 3: 
        PedFiltering = ChValue ;
        EEPROM_WriteB(EE_PedFiltering, PedFiltering);
        break;
        
        
        
    }
      OledSubPage = 0;
    
   }
   
     if ((OledPage==Menu_LCell_Setup)&&(OledSubPage!=0))
  {
     switch (OledSubPage)
    {
      case 1: 
        LCellEN = ChValue ;
        EEPROM_WriteB(EE_LCellEN, LCellEN);
        break;
        
    }
      OledSubPage = 0;
    
   }
    
   if ((OledPage==Menu_LED_Setup)&&(OledSubPage!=0))
  {
     switch (OledSubPage)
    {
      case 1: 
        LEDDriverEN = ChValue ;
        EEPROM_WriteB(EE_LEDDriverEN, LEDDriverEN);
        break;
        
    }
      OledSubPage = 0;
    
   }
   
   if ((OledPage==Menu_Board_Setup)&&(OledSubPage!=0))
  {
     switch (OledSubPage)
    {
      case 1: 
        OledInitEN = ChValue ;
        EEPROM_WriteB(EE_OledInitEN, OledInitEN);
        break;
      case 2: 
        OledDimmerEN = ChValue ;
        EEPROM_WriteB(EE_OledDimmerEN, OledDimmerEN);
        break;  
      case 3: 
        ButtonsEN = ChValue ;
        EEPROM_WriteB(EE_ButtonsEN, ButtonsEN);
        break;  
        
    }
      OledSubPage = 0;
    
   }
   
   
 } 

 
 
}

void BtnLtPress()
{
  if ( (OledPage>0)&&(OledSubPage==0) ) {OledPage--;}
  
  if ((OledPage==Menu_Controls_Setup)&&(OledSubPage!=0))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
  if ((OledPage==Menu_Cluster_Setup)&&(OledSubPage==1))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
  if ((OledPage==Menu_Cluster_Setup)&&(OledSubPage==2))
  {
    ChValue = 0;
  }
  
  if ((OledPage==Menu_Turn_Setup)&&(OledSubPage==1))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
    if ((OledPage==Menu_Turn_Setup)&&(OledSubPage==2))
  {
    if (--ChValue< 2 ) {ChValue = 2;}
  }
  
    if ((OledPage==Menu_Turn_Setup)&&(OledSubPage==3))
  {
    if (--ChValue< 1 ) {ChValue = 1;}
  }
  
    if ((OledPage==Menu_GPIO_Setup_1)&&(OledSubPage!=0))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
   if ((OledPage==Menu_GPIO_Setup_2)&&(OledSubPage!=0))
  {
    ChValue = 0;
  }
    if ((OledPage==Menu_Simulator)&&(OledSubPage==1))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
   if ((OledPage==Menu_Pedals_Setup)&&(OledSubPage==1))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
     if ((OledPage==Menu_Pedals_Setup)&&(OledSubPage==3))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
     if ((OledPage==Menu_LCell_Setup)&&(OledSubPage==1))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
     if ((OledPage==Menu_LED_Setup)&&(OledSubPage==1))
  {
    if (--ChValue< 0 ) {ChValue = 0;}
  }
  
     if ((OledPage==Menu_Board_Setup)&&(OledSubPage!=0))
  {
    ChValue = 0;
  }
  
}



void BtnRtPress()
{
  if ((OledPage<MaxPages)&&(OledSubPage==0) ) {OledPage++;}
  
  if ((OledPage==Menu_Controls_Setup)&&(OledSubPage!=0))
  {
    if (++ChValue >= CONTROL_ITEMS) {ChValue = CONTROL_ITEMS;}
  }
  
   if ((OledPage==Menu_Cluster_Setup)&&(OledSubPage==1))
  {
    if (++ChValue >= CLUSTER_ITEMS) {ChValue = CLUSTER_ITEMS;}
  }

   if ((OledPage==Menu_Cluster_Setup)&&(OledSubPage==2))
  {
    ChValue = 1;
  }
  
   if ((OledPage==Menu_Turn_Setup)&&(OledSubPage!=0))
  {
    if (++ChValue> 40 ) {ChValue = 40;}
  }
  
    if ((OledPage==Menu_GPIO_Setup_1)&&(OledSubPage!=0))
  {
    if (++ChValue> 9 ) {ChValue = 9;}
  }
  
     if ((OledPage==Menu_GPIO_Setup_2)&&(OledSubPage!=0))
  {
    ChValue = 1;
  }
     if ((OledPage==Menu_Simulator)&&(OledSubPage==1))
  {
    if (++ChValue >= SIMULATOR_ITEMS) {ChValue = SIMULATOR_ITEMS;}
  }
  
    if ((OledPage==Menu_Pedals_Setup)&&(OledSubPage==1))
  {
    if (++ChValue >= PED_ITEMS) {ChValue = PED_ITEMS;}
  }
  
  if ((OledPage==Menu_Pedals_Setup)&&(OledSubPage==3))
  {
    if (++ChValue >= 255) {ChValue = 255;}
  }
  
   if ((OledPage==Menu_LCell_Setup)&&(OledSubPage==1))
  {
    ChValue = 1;
  }
  
   if ((OledPage==Menu_LED_Setup)&&(OledSubPage==1))
  {
    ChValue = 1;
  }
     if ((OledPage== Menu_Board_Setup)&&(OledSubPage!=0))
  {
    ChValue = 1;
  }
  
 
}

void PrintMsg(char* msg)
{
  for (int i=0; i<7;i++)
   for (int j=0; j<32; j++)
   {
    MsgBuffer[i][j] =  MsgBuffer[i+1][j]; 
   }
  
 sprintf(MsgBuffer[7],"%.*s                                ",20,msg); 
}


static char* ControlName(uint8_t index)
{
  switch (index) {
   case 0: return "-NONE-";
           break;
   case CT_DAEWOO_LEV_L: return "Daewoo Lever Lt";
           break;        
   case CT_DAEWOO_LEV_R: return "Daewoo Lever Rt";
           break;            
   case CT_FIAT_CONTROLS: return "Fiat Controls";
           break;    
   case CT_FIAT_BUTTONS: return "Fiat Buttons";
           break;    
   case CT_FIAT_WHEEL: return "Fiat Wheel";
           break;    
   case CT_VW_LEV_L: return "VW Lever Left";
           break;    
   case CT_VW_LEV_R: return "VW Lever Right";
           break; 
   case CT_VW_LIGHT: return "VW Light";
           break;                
   default: return "-UNKNOWN-";
    
  }


}


static char* ClusterName(uint8_t index)
{
  switch (index) {
   case 0: return "-NONE-";
           break;
   case CL_SANDERO: return "Dacia Sandero";
           break;        
   case CL_PANDA: return "Fiat Panda";
           break;   
   case CL_EUROCARGO: return "Iveco Eurocargo";
           break;    
   default: return "-UNKNOWN-";
    
  }


}

static char* SimulatorName(uint8_t index)
{
  switch (index) {
   case 0: return "-NONE-";
           break;
   case SIM_CCDE: return "CCD Enterprise     ";
           break;  
   case SIM_CCDH: return "CCD Home           ";
           break;        
   default: return "-UNKNOWN-";
    
  }


}

static char* PedName(uint8_t index)
{
  switch (index) {
   case 0: return "-NONE-";
           break;
   case PED_ANA: return "Analog ";
           break;       
   case PED_DIGI: return "Digital";
           break;               
   default: return "-UNKNOWN-";
    
  }


}

static char* MsgOnOff(int msg)
{
  if (msg) {return "ON";}
  else  {return "OFF";}
  
}
              
 static char* MsgNoNc(int msg)
{
  if (msg) {return "Norm Close";}
  else  {return "Norm Open";}
  
}

