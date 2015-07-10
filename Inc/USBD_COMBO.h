#ifndef __USB_COMBO_CORE_H_
#define __USB_COMBO_CORE_H_

#include  "usbd_ioreq.h"


#define HID_EPIN_SIZE                 16

#define HID_DESCRIPTOR_TYPE           0x21
#define HID_REPORT_DESC               0x22
#define HID_POLLING_INTERVAL            20

#define HID_REQ_SET_PROTOCOL          0x0B
#define HID_REQ_GET_PROTOCOL          0x03

#define HID_REQ_SET_IDLE              0x0A
#define HID_REQ_GET_IDLE              0x02

#define HID_REQ_SET_REPORT            0x09
#define HID_REQ_GET_REPORT            0x01


#define HID_EPIN_ADDR                   0x82
#define CDC_IN_EP                       0x81  /* EP1 for data IN */
#define CDC_OUT_EP                      0x01  /* EP1 for data OUT */
#define CDC_CMD_EP                      0x83  /* EP2 for CDC commands */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE        512  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE         64  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                  8  /* Control Endpoint Packet size */ 

#define USB_COMBO_CONFIG_DESC_SIZ           100
#define USB_HID_DESC_SIZ                    9
#define HID_GAMEPAD_REPORT_DESC_SIZE        45

#define CDC_DATA_HS_IN_PACKET_SIZE                CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE               CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE               CDC_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/

#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23


typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}USBD_CDC_LineCodingTypeDef;

typedef struct _USBD_CDC_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  

}USBD_CDC_ItfTypeDef;

typedef enum
{
  HID_IDLE = 0,
  HID_BUSY,
}
HID_StateTypeDef; 


typedef struct
{
  uint32_t             Protocol;   
  uint32_t             IdleState;  
  uint32_t             AltSetting;
  HID_StateTypeDef     state;  
  
  uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;    
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState;    
}
USBD_COMBO_HandleTypeDef; 


extern USBD_ClassTypeDef  USBD_CDC;
#define USBD_CDC_CLASS    &USBD_CDC

uint8_t  USBD_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_CDC_ItfTypeDef *fops);

uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length);

uint8_t  USBD_CDC_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_CDC_ReceivePacket  (USBD_HandleTypeDef *pdev);

uint8_t  USBD_CDC_TransmitPacket  (USBD_HandleTypeDef *pdev);


#endif  // __USB_COMBO_CORE_H_

