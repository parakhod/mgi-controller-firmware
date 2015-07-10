#include "stm32f4xx_hal.h"

static CAN_HandleTypeDef hcan;

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
extern int d_doors;
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
extern int d_asr;
extern int d_esc;
extern int d_ebd_f;
extern int d_abs_u;
extern int d_esc_u;
extern int d_shift_up;
extern int d_shift_dn;
extern int d_park;
                
#define CLUSTER_SANDERO         1   //1 - Dacia Sandero                           
#define CLUSTER_PANDA           2   //2 - Fiat Panda
#define CLUSTER_EUROCARGO       3   //3 - Iveco Eurocargo

static void CAN_Init(void);
static void CAN_Send_Dummy(void);

static void CAN_ClusterPowerOn(void);
static void CAN_ClusterPowerOff(void);
static void CAN_Cluster_50ms(void);

static void CanTr_STD(uint32_t addr, uint32_t num, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7);

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

static void CAN_Cluster_50ms(void)
{
      switch (ClusterType) {
      case CLUSTER_SANDERO:
        if (d_brake == 1) {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);}
                 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);}
        
        
      hcan.pTxMsg->RTR = CAN_RTR_DATA;
      hcan.pTxMsg->IDE = CAN_ID_STD;        
        
      CanTr_STD(0x186,7,d_rpm/32, 0xA2,0x37,0xC3,0x7A,0x54,0x20,0); //Tachometer data

      CanTr_STD(0x217,8,0,0,0,(d_speed*10)/16,((d_speed*10)%16)*16,0,0,0); //Speedometer data

      CanTr_STD(0x5DE,8,(d_high_beam*1) + (d_low_beam*2) + /*(d_backlight_c*4)*/ 4 + (d_front_fog*8) + (d_rear_fog*16) + (d_left*32) + (d_right*64),
                 (d_fault*128),(d_doors*16),0,0,0,0,d_eco);   //Lamps 1 

      
      
    break;
    
    case CLUSTER_PANDA:
    
    break;    
    
    case CLUSTER_EUROCARGO:
    
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