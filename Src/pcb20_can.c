#include "stm32f4xx_hal.h"

static CAN_HandleTypeDef hcan;
extern RTC_HandleTypeDef RtcHandle;

extern int ClusterType;   
extern int SimulatorType;    

extern int d_rpm;        //RPM
extern int d_speed;      //Speed in km/h
extern int d_fuel;      //Fuel level (%)
extern int d_temp;       //Temperature
extern int d_park_dst;  //Park distance 0-15
extern int d_odo;      

extern int d_cluster_pwr;

extern int d_high_beam;  //indicator LEDs
extern int d_low_beam;
extern int d_low_beam_lev;
extern int d_front_fog;
extern int d_rear_fog;
extern int d_left;
extern int d_right;
extern int d_brake;
extern int d_steering_e;
extern int d_backlight_c;
extern int d_fault;
extern int d_liq_fl_e;
extern int d_br_pads_e;
extern uint8_t d_doors;
extern int d_eco;
extern int d_lfd;
extern int d_rfd;
extern int d_lrd;
extern int d_rrd;
extern int d_boot;
extern int d_bonnet;
extern int d_fuel_need;
extern int d_lock;
extern int d_lock_err;
extern int d_fuel_cut;
extern int d_city;
extern int d_ab2_fl;
extern int d_ab2;
extern int d_ab1_fl;
extern int d_ab_fail;
extern int d_b_fl_1;
extern int d_b_fl_2;
extern int d_batt;
extern int d_batt_fl;
extern int d_ss_c;
extern int d_ss_u;
extern int d_ss_ind;
extern int d_check;
extern int d_check_fl;
extern int d_overheat;
extern int d_oil;
extern int d_oil_fl;
extern int d_ch_oil;
extern uint8_t d_asr;
extern int d_esc;
extern int d_ebd_f;
extern int d_abs_u;
extern int d_esc_u;
extern int d_shift_up;
extern int d_shift_dn;
extern int d_park;
extern int d_tco_perf;
extern int d_emerg;
extern int d_hb;
extern int d_gear;
extern int d_adblue;
extern int d_deccel;
extern int d_eb;
extern int d_btn_up;
extern int d_btn_esc;
extern int d_btn_page;
extern int d_btn_menu;
extern int d_btn_minus;
extern int d_btn_plus;
extern int d_btn_down;
extern uint32_t d_trid;
extern uint32_t d_totd;
extern uint32_t d_hours;
extern uint8_t d_press_1;
extern uint8_t d_press_2;

extern uint8_t d_cruise_lcd;
extern uint8_t d_cruise_led;
extern uint8_t d_cruise_pto;
extern uint8_t d_cruise_speed;

extern uint8_t d_starter;
extern uint8_t d_mirror;
extern uint8_t d_beam_pos;

extern uint8_t d_preheat;

extern uint8_t d_speed_limit_led;
extern uint8_t d_speed_limit_value;
extern uint8_t d_speed_limit_set;
extern uint8_t d_cab_tilted;
extern uint8_t d_oil_press_low;
extern uint8_t d_oil_temp_high;
extern uint8_t d_tlr_lcd;
extern uint8_t d_tlr_led;
extern uint8_t d_br_err_1;
extern uint8_t d_br_err_2;
extern uint8_t d_br_err_3;
extern uint8_t d_1pto;
extern uint8_t d_2pto;
extern uint8_t d_3pto;
extern uint8_t d_tr_el_con;
extern uint8_t d_etc7_mode;
extern uint8_t d_airsusp;
extern uint8_t d_susp_lcm;
extern uint8_t d_susp_lift;
extern uint8_t d_esp;
extern uint8_t d_no_abs;
extern uint8_t d_no_asr;
extern uint8_t d_warning_1;
extern uint8_t d_warning_2;
extern uint8_t d_lock_front;
extern uint8_t d_lock_center;
extern uint8_t d_lock_rear;
extern uint8_t d_oil_low;
extern uint8_t d_oil_high;
extern uint8_t d_coolant_low;
extern uint8_t d_br_fluid_low;
extern uint8_t d_st_fluid_low;
extern uint8_t d_ws_fluid_low;
extern uint8_t d_steer_error;
extern uint8_t d_air_filter;
extern uint8_t d_fuel_filter;
extern uint8_t d_oil_filter;
extern uint8_t d_water_tank;
extern uint8_t d_oil_too_low;
extern uint8_t d_coolant_too_low;
extern uint8_t d_oil_level;
extern uint8_t d_oil_pressure;
extern uint16_t d_ext_temp;
extern uint8_t d_edc_warning;
extern uint8_t d_edc_error;

extern uint8_t d_tt[9];


                
#define CLUSTER_SANDERO         1   //1 - Dacia Sandero                           
#define CLUSTER_PANDA           2   //2 - Fiat Panda
#define CLUSTER_EUROCARGO       3   //3 - Iveco Eurocargo

static void CAN_Init(void);
static void CAN_Send_Dummy(void);

static void CAN_ClusterPowerOn(void);
static void CAN_ClusterPowerOff(void);
static void CAN_Cluster_10ms(void);
static void CAN_Cluster_50ms(void);
static void CAN_Cluster_100ms(void);
static void CAN_Cluster_250ms(void);
static void CAN_Cluster_500ms(void);
static void CAN_Cluster_1000ms(void);

static void CanTr_STD(uint32_t addr, uint32_t num, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7);
static void CanTr_EXT(uint32_t addr, uint32_t num, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7);

static CanTxMsgTypeDef TxMessage;
static CanRxMsgTypeDef RxMessage;

void CAN_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;   //PD0,PD1
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  __CAN1_CLK_ENABLE();
  
  hcan.Instance = CAN1;
  hcan.pTxMsg = &TxMessage;
  hcan.pRxMsg = &RxMessage;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_14TQ;
  hcan.Init.BS2 = CAN_BS2_6TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  
  switch (ClusterType) {
    case CLUSTER_SANDERO:
       hcan.Init.Prescaler = 4;
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
       hcan.Init.Prescaler = 8;    
    break;
  }
  
  HAL_CAN_Init(&hcan);

}


static void CAN_Send_Dummy(void)
{
  hcan.pTxMsg->StdId = 0x11;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  hcan.pTxMsg->IDE = CAN_ID_STD;
  hcan.pTxMsg->DLC = 2;
  hcan.pTxMsg->Data[0] = 0x12;
  hcan.pTxMsg->Data[1] = 0x34;
  HAL_CAN_Transmit(&hcan,10);
 }




static void CAN_ClusterPowerOn(void)
{
  switch (ClusterType) {
    case CLUSTER_SANDERO:
      hcan.pTxMsg->StdId = 0x1F6;
      hcan.pTxMsg->RTR = CAN_RTR_DATA;
      hcan.pTxMsg->IDE = CAN_ID_STD;
      hcan.pTxMsg->DLC = 2;
      hcan.pTxMsg->Data[0] = 0x00;
      hcan.pTxMsg->Data[1] = 0x20;
      HAL_CAN_Transmit(&hcan,1);
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
    
    break;

  }
  
  
}


static void CAN_ClusterPowerOff(void)
{
    switch (ClusterType) {
    case CLUSTER_SANDERO:
      hcan.pTxMsg->StdId = 0x350;
      hcan.pTxMsg->RTR = CAN_RTR_DATA;
      hcan.pTxMsg->IDE = CAN_ID_STD;
      hcan.pTxMsg->DLC = 2;
      hcan.pTxMsg->Data[0] = 0x84;
      HAL_CAN_Transmit(&hcan,1);
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
    
    break;
    }
}

static void CAN_Cluster_10ms(void)
{
  switch (ClusterType) {
    case CLUSTER_SANDERO:   
      
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->IDE = CAN_ID_EXT;      
    CanTr_EXT(0x0CF00400,8,0xF1,0x7D,0x92,0xC8,d_rpm/32,0xFF,0x01*d_starter,0xFF); //EEC1 - Electronic Engine Controller 1
    
    
    break;
  }
  
}

static void CAN_Cluster_50ms(void)
{
      switch (ClusterType) {
      case CLUSTER_SANDERO:
        if (d_brake == 1) {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);}
                 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);}   
        
      CanTr_STD(0x186,7,d_rpm/32, 0xA2,0x37,0xC3,0x7A,0x54,0x20,0); //Tachometer data

      CanTr_STD(0x217,8,0,0,0,(d_speed*10)/16,((d_speed*10)%16)*16,0,0,0); //Speedometer data

      CanTr_STD(0x5DE,8,(d_high_beam*1) + (d_low_beam*2) + /*(d_backlight_c*4)*/ 4 + (d_front_fog*8) + (d_rear_fog*16) + (d_left*32) + (d_right*64),
                 (d_fault*128),(d_doors*16),0,0,0,0,d_eco);   //Lamps 1 

      
      
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
    CanTr_EXT(0x18FF6221,8,(d_backlight_c*1)+(d_high_beam*0x10)+(d_low_beam*0x40),  // BCAO2 - BC Auxiliary Outputs 2 - Body, Lights, Doors
                                    (d_rear_fog*4) + (d_front_fog*1),
                                    (d_left*1)+(d_right*4)+(d_emerg*0x10),0,0,d_mirror,d_doors,d_beam_pos);
 
    CanTr_EXT(0x18FF2100,8, // Edc2Bc - EDC to Body Controller
              0x01*d_preheat,
              0x01*d_speed_limit_set+(0x40*d_overheat),
              d_speed_limit_value,
              d_check*0x01+d_check_fl*0x02+d_speed_limit_led*0x10,
              d_cab_tilted*0x01,
              d_oil_press_low*0x01+d_oil_temp_high*0x40,
              0,0);                                       
  
    CanTr_EXT(0x0CFE6CEE,8,0x00,0x00,0x00,(0x10*d_tco_perf),0,0,0,d_speed);                          // TCO1 - Tachograph
  
    CanTr_EXT(0x18FFC272,8,0,0,0,0,0,0,0,0);                                     // EmCmd - Expansion Module Command
    
    break;
  }
  
}

static void CAN_Cluster_100ms(void)
{
  switch (ClusterType) {
    case CLUSTER_SANDERO:   
      
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
        
      CanTr_EXT(0x18FF6121,8,(d_cluster_pwr*0x01),(d_brake*0xFF),d_tlr_lcd*0x01+d_tlr_led*0x40,(d_hb*0x40),0,0,(d_b_fl_1*0x40)+(d_b_fl_2*0x80),0); //BCAO1 - BC Auxiliary Outputs 1 - Powertrain
     
      CanTr_EXT(0x10FF2931,8,0,0,0,0,0,0,0,0);                           // CAC_BM - Cab Alarm Clock (BM Sts 2, BC2IC, IC2BC)

      
      CanTr_EXT(0x18FF6321,8,d_br_err_1,d_br_err_2,0,d_tr_el_con,0,d_br_err_3,0,0);  // BCAO3 - BC Auxiliary Outputs 3 - Brake, PTO

      
      CanTr_EXT(0x0CFE5A2F,8,0xFF,d_airsusp,0,d_susp_lcm*0x10,d_susp_lift*0x40,0,0,0);                        // ASC1_A - Air Suspension Control 1


      CanTr_EXT(0x18FE4A03,8,0x0F,0x1F,d_etc7_mode,0xFF,0xFF,0xFF,0xFF,0xFF);     // ETC7 - Electronic Transmission Controller 7

    ///  d_tt[1],d_tt[2],d_tt[3],d_tt[4],d_tt[5],d_tt[6],d_tt[7],d_tt[8]
      CanTr_EXT(0x18FE4F0B,8,(d_esp==0)*0x04,0,0,0,0,0,0,0);                             // VDC1 - Vehicle Dynamic Stability Control 1


      CanTr_EXT(0x18FEFC21,8,0xFF,d_fuel*2.5,0,0,0,0,0,0);                // DD_BC - Dash Display


      CanTr_EXT(0x18F0010B,8,d_asr,0xFF,d_no_abs*0x01+d_no_asr*0x04,0xFF,0xFF,0xCD,0xFF,0x00);     // EBC1 - Electronic Brake Controller 1


      CanTr_EXT(0x18F00503,8,d_gear,0x00,0x00,d_gear,0x4E,0x20,0x4E,0x43); // ETC2 - Electronic Transmission Controller 2


      CanTr_EXT(0x18FEF100,8,(d_brake*0x04),0,0,0x01*d_cruise_lcd,0,d_cruise_speed,0x01*d_cruise_pto+0x20*d_cruise_led,0);               // CCVS1 - Cruise Control/Vehicle Speed 1


      CanTr_EXT(0x18FFD003,8,d_warning_1,d_warning_2,0,0,0,0,0,0);                        // ICRW_ETC - Instrument Cluster Remote Warning


      CanTr_EXT(0x18F00010,8,0,(!d_deccel*0xFF),0,0,0,0,0,0);           // ERC1_DR - Electronic Retarder Controller 1


      CanTr_EXT(0x18F00029,8,0,(!d_eb*0xFF),0,0,0,0,0,0);               // ERC1_EX - Electronic Retarder Controller 1


      CanTr_EXT(0x18F00621,8,0,d_lock_front*0x01+d_lock_rear*0x10,d_lock_center*0x01,0,0,0,0,0);                          // EAC1_21 - Electronic Axle Controller 1

      
      CanTr_EXT(0x18FF0203,8,0xCC,0xFF ,0xFF ,0x00 ,0xFF ,0x38 ,0x12 ,0xFF);                          // Tc2Bc - Transmission Controller to Body Computer
   //
      
      CanTr_EXT(0x18011771,4,(d_btn_up)+ (d_btn_esc*0x04)+(d_btn_page*0x10)+(d_btn_menu*0x40),
                                        (d_btn_minus*0x04)+(d_btn_plus*0x10)+(d_btn_down*0x40),0,0,0,0,0,0);   // Swi2Ic - Steering Wheel Interface to Instrument Cluster

      
      CanTr_EXT(0x18FEF027,8,0,0,0,0,0,0,0,0);                          // PTO_MC - Power Takeoff Information

      
      CanTr_EXT(0x18FEF200,8,0,0,0,0,0,0,0,0);                          // LFE1 - Fuel Economy (Liquid)

      
      CanTr_EXT(0x18FEC872,8,0,0,0,0,0,0,0,0);                          // GPM22_EM - General Purpose Message #2/2

      
      CanTr_EXT(0x1807023D,8,d_adblue,0,0,0,0,0,0,0);                                 // AdBlue

      
    
    break;
  }
  
}

static void CAN_Cluster_250ms(void)
{
  switch (ClusterType) {
    case CLUSTER_SANDERO:   
      
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
      CanTr_EXT(0x18FF6421,8,d_oil_low*0x01+d_oil_high*0x40+d_fuel_need*0x04+d_coolant_low*0x10,
                             d_br_fluid_low*0x01+d_st_fluid_low*0x04+d_ws_fluid_low*0x10+d_steer_error*0x40,
                             d_air_filter*0x01+d_fuel_filter*0x04+d_oil_filter*0x10+d_water_tank*0x40,
                             0,0,0,
                             d_oil_too_low*0x01+d_coolant_too_low*0x04,0);                 // LowLev - Fluid Low Levels, Body Sensors
  
    
    break;
  }
  
}

static void CAN_Cluster_500ms(void)
{
  switch (ClusterType) {
    case CLUSTER_SANDERO:   
      
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
    CanTr_EXT(0x18FEEF21,8,0,0,d_oil_level,0,0,0,0,0);                              // EFLP1_BC - Engine Fluid Level/Pressure 1
  
    CanTr_EXT(0x18FEEF00,8,100,0,0,d_oil_pressure,0,0,0,0);                            // EFLP1Crf - Engine Fluid Level/Pressure 1
      
    
    break;
  }
  
}

static void CAN_Cluster_1000ms(void)
{
  switch (ClusterType) {
    case CLUSTER_SANDERO:   
      
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
      
    uint8_t h1 = d_hours & 0xFF;
    uint8_t h2 = (d_hours >> 8) & 0xFF;
    uint8_t h3 = (d_hours >> 16) & 0xFF;
    uint8_t h4 = (d_hours >> 24) & 0xFF;    
      
    CanTr_EXT(0x18FEE500,8,h1,h2,h3,h4,0,0,0,0);    // HOURS - Engine Hours, Revolutions
    
    uint8_t t1 = d_ext_temp & 0xFF;
    uint8_t t2 = (d_ext_temp >> 8) & 0xFF;
    
    CanTr_EXT(0x0CFEF521,8,0xFF,0xFF,0xFF,t1,t2,0xFF,0xFF,0xFF);       // AMB_BC - Ambient Conditions
    
    CanTr_EXT(0x18FEAE21,8,0,0,d_press_1,d_press_2,0,0,0,0);             // AIR1_BC - Air Supply Pressure
   
    CanTr_EXT(0x18FEEE00,8,d_temp,0xFF,0x29,0x29,0xFF,0xFF,0xFF,0xFF);        // ET1 - Engine Temperature 1
    
    uint32_t dx_totd = d_totd * 200;
    
    uint8_t to1 = dx_totd & 0xFF;
    uint8_t to2 = (dx_totd >> 8) & 0xFF;
    uint8_t to3 = (dx_totd >> 16) & 0xFF;
    uint8_t to4 = (dx_totd >> 24) & 0xFF;  
    
    uint32_t dx_trid = d_trid * 20;
    
    uint8_t tr1 = dx_trid & 0xFF;
    uint8_t tr2 = (dx_trid >> 8) & 0xFF;
    uint8_t tr3 = (dx_trid >> 16) & 0xFF;
    uint8_t tr4 = (dx_trid >> 24) & 0xFF;    
    
    CanTr_EXT(0x18FEC1EE,8,to1,to2,to3,to4,tr1,tr2,tr3,tr4); // VDHR - High Resolution Vehicle Distance
    

    
    CanTr_EXT(0x18FF4072,8,d_3pto*0x04+d_2pto*0x10+d_1pto*0x40,0,0,0,0,0,0,0);     // PtoSts - PTO Status
    
    CanTr_EXT(0x18FD7C00,8,0,0,0,0,0,0,0,0);                               // DPFC1 - Diesel Particulate Filter Control 1
  
    CanTr_EXT(0x18FEF803,8,0,0,0,0,0,0,128,0x10);                           // TRF1 - Transmission Fluids 1
    
    RTC_TimeTypeDef stimestructureget;
    HAL_RTC_GetTime(&RtcHandle, &stimestructureget, FORMAT_BIN);
    RTC_DateTypeDef sdatestructureget;
    HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  
    CanTr_EXT(0x18FEE6EE,8,0x07,stimestructureget.Minutes,stimestructureget.Hours,sdatestructureget.Month,sdatestructureget.Date,2000 + sdatestructureget.Year,0x7D,0x7D);          // TD - Time/Date
  
    CanTr_EXT(0x18FEE900,8,0,0,0,0,0,0,0,0);                                 // LFC - Fuel Consumption (Liquid)
  
    CanTr_EXT(0x18FEF721,8,0,0,0,0,0,0,0xE0*d_batt,1*d_batt);                                 // VEP1_21 - Vehicle Electrical Power 1
    
    CanTr_EXT(0x18FECA00,8,d_edc_warning*0x04 + d_edc_error*0x10,0,0,0,0,0,0,0);                  // DM01_EEC - Active Diagnostic Trouble Codes  
    //0x2F-suspension
    //0x71-swi
    //0xEE-tco
      
    break;
  }
  
}


void CanTr_STD(uint32_t addr, uint32_t num, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
      hcan.pTxMsg->StdId = addr;
      hcan.pTxMsg->RTR = CAN_RTR_DATA;
      hcan.pTxMsg->IDE = CAN_ID_STD;
      hcan.pTxMsg->DLC = num;
      hcan.pTxMsg->Data[0] = b0;
      hcan.pTxMsg->Data[1] = b1;
      hcan.pTxMsg->Data[2] = b2;
      hcan.pTxMsg->Data[3] = b3;
      hcan.pTxMsg->Data[4] = b4;
      hcan.pTxMsg->Data[5] = b5;
      hcan.pTxMsg->Data[6] = b6;
      hcan.pTxMsg->Data[7] = b7;
      
      HAL_CAN_Transmit(&hcan,1);       
}

void CanTr_EXT(uint32_t addr, uint32_t num, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
      hcan.pTxMsg->ExtId = addr;
      hcan.pTxMsg->RTR = CAN_RTR_DATA;
      hcan.pTxMsg->IDE = CAN_ID_EXT;
      hcan.pTxMsg->DLC = num;
      hcan.pTxMsg->Data[0] = b0;
      hcan.pTxMsg->Data[1] = b1;
      hcan.pTxMsg->Data[2] = b2;
      hcan.pTxMsg->Data[3] = b3;
      hcan.pTxMsg->Data[4] = b4;
      hcan.pTxMsg->Data[5] = b5;
      hcan.pTxMsg->Data[6] = b6;
      hcan.pTxMsg->Data[7] = b7;
      
      HAL_CAN_Transmit(&hcan,1);       
}