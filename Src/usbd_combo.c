#include "USBD_COMBO.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_COMBO_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length);


uint8_t  *USBD_COMBO_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMBO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC = 
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_COMBO_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,     
  USBD_CDC_GetFSCfgDesc,  
  USBD_CDC_GetFSCfgDesc,    
  USBD_CDC_GetFSCfgDesc, 
  USBD_COMBO_GetDeviceQualifierDescriptor,
};


/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_CDC_CfgFSDesc[USB_COMBO_CONFIG_DESC_SIZ] __ALIGN_END =
{
  //  Configuration Descriptor:
//------------------------------
0x09,// 	bLength
0x02,// 	bDescriptorType
USB_COMBO_CONFIG_DESC_SIZ,// 
0x00,// 	wTotalLength
0x03,// 	bNumInterfaces
0x01,// 	bConfigurationValue
0x00,// 	iConfiguration
0x80,// 	bmAttributes   (Bus-powered Device)
0x96,// 	bMaxPower   (300 mA)

//Interface Descriptor:
//------------------------------
0x09,// 	bLength
0x04,// 	bDescriptorType
0x00,// 	bInterfaceNumber //=========================HID IF number
0x00,// 	bAlternateSetting
0x01,// 	bNumEndPoints
0x03,// 	bInterfaceClass   (Vendor specific)
0x01,// 	bInterfaceSubClass   
0x00,// 	bInterfaceProtocol   
0x04,// 	iInterface   "ST-Link Debug"
//18
 /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_GAMEPAD_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
  
  HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
  0x00,
  HID_POLLING_INTERVAL,          /*bInterval: Polling Interval (10 ms)*/
  /* 34 */


//Interface Association Descriptor:
//------------------------------
0x08,// 	bLength
0x0B,// 	bDescriptorType
0x01,// 	bFirstInterface
0x02,// 	bInterfaceCount
0x02,// 	bFunctionClass   (Communication Device Class)
0x02,// 	bFunctionSubClass   (Abstract Control Model)
0x01,// 	bFunctionProtocol   (ITU-T V.250)
0x05,// 	iFunction   "ST-Link VCP Ctrl"

//Interface Descriptor:
//------------------------------
0x09,// 	bLength
0x04,// 	bDescriptorType
0x01,// 	bInterfaceNumber   //--------VCPC IF N-----------------------02 (01)
0x00,// 	bAlternateSetting
0x01,// 	bNumEndPoints
0x02,// 	bInterfaceClass   (Communication Device Class)
0x02,// 	bInterfaceSubClass   (Abstract Control Model)
0x01,// 	bInterfaceProtocol   (ITU-T V.250)
0x00,// 	iInterface   "ST-Link VCP Ctrl"

//CDC Header Functional Descriptor:
//------------------------------
0x05,// 	bFunctionalLength
0x24,// 	bDescriptorType
0x00,// 	bDescriptorSubtype
0x10,// 
0x01,// 	bcdCDC

//CDC Call Management Functional Descriptor:
//------------------------------
0x05,// 	bFunctionalLength
0x24,// 	bDescriptorType
0x01,// 	bDescriptorSubtype
0x00,// 	bmCapabilities
0x02,// 	bDataInterface       =================03========================

//CDC Abstract Control Management Functional Descriptor:
//------------------------------
0x04,// 	bFunctionalLength  /* bFunctionLength */
0x24,// 	bDescriptorType   /* bDescriptorType: CS_INTERFACE */
0x02,// 	bDescriptorSubtype /* bDescriptorSubtype: Abstract Control Management desc */
0x02,// 	bmCapabilities    /* bmCapabilities */

//CDC Union Functional Descriptor:
//------------------------------
0x05,// 	bFunctionalLength
0x24,// 	bDescriptorType
0x06,// 	bDescriptorSubtype
0x01,// 	bControlInterface    //----02 (0)  /* bMasterInterface: Communication class interface */
0x02,// 	bSubordinateInterface(0)  //===== 03 (1)   /* bSlaveInterface0: Data Class Interface */

//Endpoint Descriptor:
//------------------------------
0x07,// 	bLength
0x05,// 	bDescriptorType
CDC_CMD_EP,// 	bEndpointAddress   (IN Endpoint)
0x03,// 	bmAttributes	(Transfer: Interrupt / Synch: None / Usage: Data)
0x08,// 
0x00,// 	wMaxPacketSize   (2 Bytes) 
0x10,// 	bInterval         //=================0xFF (10)

//Interface Descriptor:
//------------------------------
0x09,// 	bLength
0x04,// 	bDescriptorType
0x02,// 	bInterfaceNumber /// ==========================3(01)
0x00,// 	bAlternateSetting
0x02,// 	bNumEndPoints
0x0A,// 	bInterfaceClass   (CDC Data)
0x00,// 	bInterfaceSubClass   
0x00,// 	bInterfaceProtocol   
0x00,// 	iInterface   "ST-Link VCP Data"  ///05

//Endpoint Descriptor:
//------------------------------
0x07,// 	bLength
0x05,// 	bDescriptorType
CDC_OUT_EP,// 	bEndpointAddress   (OUT Endpoint)
0x02,// 	bmAttributes	(Transfer: Bulk / Synch: None / Usage: Data)
CDC_DATA_FS_OUT_PACKET_SIZE,// 
0x00,// 	wMaxPacketSize   (8 Bytes) 
0x00,// 	bInterval

//Endpoint Descriptor:
//------------------------------
0x07,// 	bLength
0x05,// 	bDescriptorType
CDC_IN_EP,// 	bEndpointAddress   (IN Endpoint)
0x02,// 	bmAttributes	(Transfer: Bulk / Synch: None / Usage: Data)
CDC_DATA_FS_IN_PACKET_SIZE,//
0x00,// 	wMaxPacketSize   (16 Bytes) 
0x00// 	        bInterval
} ;


/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ]  __ALIGN_END  =
{
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_GAMEPAD_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};


__ALIGN_BEGIN static uint8_t HID_GAMEPAD_ReportDesc[HID_GAMEPAD_REPORT_DESC_SIZE]  __ALIGN_END =
{
   
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x20,                    //     USAGE_MAXIMUM (Button 32)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x20,                    //     REPORT_COUNT (32)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop) 02
    0x09, 0x30,                    //     USAGE (Y) 0x09, 0xC4,  
    0x09, 0x31,                    //     USAGE (Z) 0x09, 0xC5, 
    0x09, 0x32,                    //     USAGE (Rx) 0x09, 0xC6, 
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x03,              //     LOGICAL_MAXIMUM (1023)
    0x75, 0x10,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (4)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
      
}; 
/*
__ALIGN_BEGIN static uint8_t HID_GAMEPAD_ReportDesc[HID_GAMEPAD_REPORT_DESC_SIZE]  __ALIGN_END =
{
   
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x20,                    //     USAGE_MAXIMUM (Button 16)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x20,                    //     REPORT_COUNT (32)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop) 02
    0x09, 0x30,                    //     USAGE (Y) 0x09, 0xC4,  
    0x09, 0x31,                    //     USAGE (Z) 0x09, 0xC5, 
    0x09, 0x32,                    //     USAGE (Rx) 0x09, 0xC6, 
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x03,              //     LOGICAL_MAXIMUM (1023)
    0x75, 0x10,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (4)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
      
}; 
*/




/**
  * @brief  USBD_CDC_Init
  *         Initilaize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_COMBO_HandleTypeDef   *hcdc;
  
    USBD_LL_OpenEP(pdev,
                 HID_EPIN_ADDR,
                 USBD_EP_TYPE_INTR,
                 HID_EPIN_SIZE);  
  
    /* Open EP IN */
   USBD_LL_OpenEP(pdev,
                   CDC_IN_EP,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_IN_PACKET_SIZE);
    
    /* Open EP OUT */
   USBD_LL_OpenEP(pdev,
                   CDC_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_OUT_PACKET_SIZE);
   /* Open Command IN EP */
   USBD_LL_OpenEP(pdev,
                 CDC_CMD_EP,
                 USBD_EP_TYPE_INTR,
                 CDC_CMD_PACKET_SIZE);
  
    
  pdev->pClassData = (void *) USBD_malloc(sizeof (USBD_COMBO_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    hcdc = pdev->pClassData;
    
    /* Init  physical Interface components */
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();
    
    /* Init Xfer states */
    hcdc->TxState =0;
    hcdc->RxState =0;
       
   hcdc->state = HID_IDLE; 
     
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP,
                             hcdc->RxBuffer,
                             CDC_DATA_FS_OUT_PACKET_SIZE);

    
    
  }
  return ret;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  uint8_t ret = 0;
  
  USBD_LL_CloseEP(pdev,
                  HID_EPIN_ADDR);
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev,
              CDC_IN_EP);
  
  /* Open EP OUT */
  USBD_LL_CloseEP(pdev,
              CDC_OUT_EP);
  
  /* Open Command IN EP */
  USBD_LL_CloseEP(pdev,
              CDC_CMD_EP);
  
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return ret;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_COMBO_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
 USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  uint16_t len = 0;
  uint8_t  *pbuf = NULL;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                          (uint8_t *)hcdc->data,
                                                          req->wLength);
          USBD_CtlSendData (pdev, 
                            (uint8_t *)hcdc->data,
                            req->wLength);
      }
      else
      {
        hcdc->CmdOpCode = req->bRequest;
        hcdc->CmdLength = req->wLength;
        
        USBD_CtlPrepareRx (pdev, 
                           (uint8_t *)hcdc->data,
                           req->wLength);
      }
      
    }
    else
    {
      /*  ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                          NULL,
                                                          0);*/
     
      switch (req->bRequest)
    {
      
      
    case HID_REQ_SET_PROTOCOL:
      hcdc->Protocol = (uint8_t)(req->wValue);
      break;
      
    case HID_REQ_GET_PROTOCOL:
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hcdc->Protocol,
                        1);    
      break;
      
    case HID_REQ_SET_IDLE:
      hcdc->IdleState = (uint8_t)(req->wValue >> 8);
      break;
      
    case HID_REQ_GET_IDLE:
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hcdc->IdleState,
                        1);        
      break;      
      
    default:
     // USBD_CtlError (pdev, req);
      return USBD_FAIL; 
    }
      
    }  
      
      
    
    break;
    
    case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( req->wValue >> 8 == HID_REPORT_DESC)
      {
        len = MIN(HID_GAMEPAD_REPORT_DESC_SIZE , req->wLength);
        pbuf = HID_GAMEPAD_ReportDesc;
      }
      else if( req->wValue >> 8 == HID_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_HID_Desc;   
        len = MIN(USB_HID_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&hcdc->AltSetting,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      hcdc->AltSetting = (uint8_t)(req->wValue);
      break;
    }   
       
 
  default: 
    break;
  }
  return USBD_OK;
}

/**
  * @brief  usbd_audio_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  
  if (epnum == 1) {
   if(pdev->pClassData != NULL)
   {
    
     hcdc->TxState = 0;

     return USBD_OK;
   }
   else
   {
     return USBD_FAIL;
   }
  } 
  
    
  if (epnum == 2) {
   hcdc->state = HID_IDLE;
   return USBD_OK;    
  }
  
   if (epnum == 3) {
   return USBD_OK;    
  }
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  
  /* Get the received data length */
  hcdc->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Receive(hcdc->RxBuffer, &hcdc->RxLength);

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}



/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
  USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  
  if((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFF))
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(hcdc->CmdOpCode,
                                                      (uint8_t *)hcdc->data,
                                                      hcdc->CmdLength);
      hcdc->CmdOpCode = 0xFF; 
      
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_GetFSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_CDC_CfgFSDesc);
  return USBD_CDC_CfgFSDesc;
}


/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_COMBO_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_COMBO_DeviceQualifierDesc);
  return USBD_COMBO_DeviceQualifierDesc;
}

/**
* @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_CDC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData= fops;
    ret = USBD_OK;    
  }
  
  return ret;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  
  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;  
  
  return USBD_OK;  
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
  USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  
  hcdc->RxBuffer = pbuff;
  
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{      
  USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    if(hcdc->TxState == 0)
    {
      
      /* Transmit next packet */
      
      
      HAL_PCD_EP_Transmit(pdev->pData, CDC_IN_EP, hcdc->TxBuffer, hcdc->TxLength);
      
      /* Tx Transfer in progress */
      hcdc->TxState = 1;
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{      
  USBD_COMBO_HandleTypeDef   *hcdc = pdev->pClassData;
  
  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {

      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP,
                             hcdc->RxBuffer,
                             CDC_DATA_FS_OUT_PACKET_SIZE);

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_HID_SendReport 
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_SendReport     (USBD_HandleTypeDef  *pdev, 
                                 uint8_t *report,
                                 uint16_t len)
{
   
  USBD_COMBO_HandleTypeDef     *hhid = pdev->pClassData;
  
  uint8_t OpResult;
  OpResult = 9;
  
  if (pdev->dev_state == USBD_STATE_CONFIGURED )
  {
    if(hhid->state == HID_IDLE)
    {
      hhid->state = HID_BUSY;
      
     OpResult = HAL_PCD_EP_Transmit(pdev->pData, HID_EPIN_ADDR, report, len);
    }
  }
  return OpResult;
}

