#ifndef __PCB20_DEFINES
#define __PCB20_DEFINES

#define FW_VERSION "0.1.9"

#define OLED_ON_TIME 180
#define OLED_DIM_TIME 30

#define Menu_Pages_Count        16

#define Menu_General_Info       0
#define Menu_Controls_Status    1
#define Menu_Controls_Setup     2
#define Menu_Cluster_Status     3
#define Menu_Cluster_Setup      4
#define Menu_GPIO_In            5
#define Menu_GPIO_Setup_1       6
#define Menu_GPIO_Setup_2       7
#define Menu_Pedals_Status      8
#define Menu_Pedals_Setup       9
#define Menu_Simulator          10
#define Menu_Turn_Setup         11
#define Menu_Board_Setup        12
#define Menu_LCell_Status       13
#define Menu_LCell_Setup        14
#define Menu_LED_Status         15
#define Menu_LED_Setup          16

#define Cell_Reset              0
#define Cell_Init               1

#define Cell_ContRead           5

#define Cell_MsgReady           99
#define Cell_Ready              100
#define Cell_Fail               200
#define Cell_Prst0_0            10
#define Cell_Prst0_1            11
#define Cell_Prst0_2            12
#define Cell_Prst1_0            20
#define Cell_Prst1_1            21
#define Cell_Prst1_2            22
#define Cell_Prst2_0            30
#define Cell_Prst2_1            31
#define Cell_Prst2_2            32
#define Cell_Prst3_0            40
#define Cell_Prst3_1            41
#define Cell_Prst3_2            42
#define Cell_SelCh1             50
#define Cell_SelCh2             51
#define Cell_SelCh3             52
#define Cell_SelCh4             53

#define Cell_CalZero            54
#define Cell_CalWeight          55

#define CellCalCycles           10



#define CT_DAEWOO_LEV_L         1
#define CT_DAEWOO_LEV_R         2
#define CT_FIAT_CONTROLS        10
#define CT_FIAT_BUTTONS         11
#define CT_FIAT_WHEEL           12
#define CT_VW_LEV_L             21
#define CT_VW_LEV_R             22
#define CT_VW_LIGHT             23

static const int ControlsList[]={
        0,
        CT_DAEWOO_LEV_L,
        CT_DAEWOO_LEV_R,
        CT_FIAT_CONTROLS,
        CT_FIAT_BUTTONS,
        CT_FIAT_WHEEL,
        CT_VW_LEV_L,
        CT_VW_LEV_R,
        CT_VW_LIGHT};
  
#define CONTROL_ITEMS  8

#define CL_SANDERO              1
#define CL_PANDA                2
#define CL_EUROCARGO            3

static const int ClusterList[]={
        0,
        CL_SANDERO,
        CL_PANDA,
        CL_EUROCARGO};

#define CLUSTER_ITEMS  3

#define SIM_CCDE                1
#define SIM_CCDH                2

static const int SimulatorList[]={
        0,
        SIM_CCDE,
        SIM_CCDH};

#define SIMULATOR_ITEMS  2

#define PED_ANA                 1
#define PED_DIGI                2

static const int PedList[]={
        0,
        PED_ANA,
        PED_DIGI};

#define PED_ITEMS  2

#define EE_SN                   0x0000

#define EE_ClusterType          0x0011
#define EE_ClusterAutonomy      0x0012

#define EE_CtrlTypes0           0x0020
#define EE_CtrlTypes1           0x0021
#define EE_CtrlTypes2           0x0022
#define EE_CtrlTypes3           0x0023

#define EE_TurnDelay            0x0030
#define EE_TurnPeriod           0x0031
#define EE_TurnON               0x0032

#define EE_CTRL_SB_INV          0x0040
#define EE_CTRL_BR_INV          0x0041
#define EE_CTRL_IG_INV          0x0042
#define EE_CTRL_ST_INV          0x0043

#define EE_CTRL_SB_PIN          0x0050
#define EE_CTRL_BR_PIN          0x0051
#define EE_CTRL_IG_PIN          0x0052
#define EE_CTRL_ST_PIN          0x0053

#define EE_SimulatorType        0x0060

#define EE_PedType              0x0070
#define EE_PedFiltering         0x0072

#define EE_Pedal_1_Lo           0x0080
#define EE_Pedal_1_Hi           0x0082
#define EE_Pedal_2_Lo           0x0084
#define EE_Pedal_2_Hi           0x0086
#define EE_Pedal_3_Lo           0x0088
#define EE_Pedal_3_Hi           0x008a

#define EE_LCellEN              0x00a0
#define EE_LEDDriverEN          0x00a1

#define EE_RtcEN                0x0100
#define EE_TempEN               0x0101
#define EE_OledInitEN           0x0102
#define EE_OledDimmerEN         0x0103
#define EE_ButtonsEN            0x0104

#define EE_CellZero0            0x0110
#define EE_CellZero1            0x0114
#define EE_CellZero2            0x0118
#define EE_CellZero3            0x011c

#define EE_CellScale0           0x0120
#define EE_CellScale1           0x0124
#define EE_CellScale2           0x0128
#define EE_CellScale3           0x012c



#endif