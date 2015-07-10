/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usbd_desc.h"
#include "usbd_combo.h" 
//#include "usbd_hid.h" 

#include "usbd_cdc_interface.h"

    
#include "pcb20_system.c"     
#include "pcb20_can.c" 
#include "pcb20_i2c.c" 
#include "pcb20_rtc.c"     
#include "pcb20_uart.c"  
//#include "pcb20_usb.c"  
#include "pcb20_adc.c"  
#include "pcb20_oled.c"  
#include "pcb20_menu.c"  
#include "pcb20_spi_e.c"      


#endif /* __MAIN_H */

