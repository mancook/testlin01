/**************************************************************************************************
  Filename:       SampleApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Sample Application definitions.


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef SAMPLEAPP_H
#define SAMPLEAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define SAMPLEAPP_ENDPOINT           20

#define SAMPLEAPP_PROFID             0x0F08
#define SAMPLEAPP_DEVICEID           0x0001
#define SAMPLEAPP_DEVICE_VERSION     0
#define SAMPLEAPP_FLAGS              0

#define SAMPLEAPP_MAX_CLUSTERS       5//20140602
#define SAMPLEAPP_PERIODIC_CLUSTERID 1
#define SAMPLEAPP_FLASH_CLUSTERID    2
#define SAMPLEAPP_COM_CLUSTERID      3
  
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID  4 //´«Êä±àºÅ
#define SAMPLEAPP_ZIGBEE_SECURITY_KEY_CLUSTERID 5

// Send Message Timeout
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT  1000     // Every 5 seconds
#define SAMPLEAPP_IEEE_PERIODIC_CHECK_TIMEOUT 10000
  
// Application Events (OSAL) - These are bit weighted definitions.
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT       0x0001
#define SYS_EVENT_IEEE                        0x0010  //print short address event
  
// Group ID for Flash Command
#define SAMPLEAPP_FLASH_GROUP                  0x0001
  
// Flash Command Duration - in milliseconds
#define SAMPLEAPP_FLASH_DURATION               1000
  
// ST message pattern

#define ST_APP_DO_OPERATION        0xE0
#define ST_ZB_OPEATION_RESULT      0xE1
#define ST_ZB_OPEATION_FAILURE     0xE2
#define ST_ZB_DEVCIE_EVENT         0xE3
// ST communication specification version  
#define ST_VERSION_1_0 1  

//ST communication Device Operation Code
#define ST_OPT_GET_PROP     0x1101
#define ST_PROP_T1          0x0001
#define ST_OPT_SET_PROP     0x1102
#define ST_OPT_GET_POWER_MODE 0x1103

typedef unsigned char       st_byte;
typedef unsigned char       st_char;
typedef unsigned char       st_uint8;
typedef unsigned short      st_uint16;
typedef unsigned long       st_uint32;
// signed integers
typedef signed char         st_int8;
typedef signed short        st_int16;
typedef signed long         st_int32;

typedef float 	            st_float;
typedef double		    st_double;

  
  
/*********************************************************************
 * MACROS
 */
typedef struct
{
  uint8  user;
  uint16 nwkAddr;
  uint8  extAddr[Z_EXTADDR_LEN];
  uint8  flag;
} g_st_node_t;

typedef struct
{
  osal_event_hdr_t  hdr;
  uint8             state; 
  uint16            shortaddr;  
} st_short_t;

typedef struct{
  uint8 version;
  uint8	type;
  uint8	len;  
  uint16  addr;
  uint8   checksum;
  uint16  reserved;
} st_serial_pkg_hdr_t;
typedef union
{
  uint16 Addr16;
  uint8 Addr8[2];
}st_nwkaddress_t;

typedef union
{
  uint16 u16;
  uint8 u8[2];
}st_uint16_uint8_t;

typedef union
{
  uint32 u32;
  uint8 u8[4];
}st_uint32_uint8_t;
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void SampleApp_Init( uint8 task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 SampleApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SAMPLEAPP_H */
