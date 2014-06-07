/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "ZComDef.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "DHT11.H"
#include "OSAL_Nv.h"
#define PEOPLE P2_0      
#define NODE_ID '3'
#define ST_ZIGBEE_KEY 0x50


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;
g_st_node_t g_st_node_info_t[Z_MAX_NODE_NUM];
uint8 st_nodenum;
uint16 g_st_add16;
st_nwkaddress_t NwkAddr;
st_nwkaddress_t NwkAddrRecive;
st_uint16_uint8_t cmd_t;
NLME_LeaveReq_t *LeaveReq;
uint16 NV_ID = 0x0F00;
st_uint8 g_st_message_from_app[200];
st_uint8 g_st_get_header = 0;
st_uint8 g_st_start_check_header = 0;
st_uint32 g_st_send_periodic_msg_timeout = 5;
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern st_uint8 g_st_wendu_xiaoshu;
extern st_uint8 g_st_shidu_xiaoshu;
extern st_uint8 g_st_wendu_shi;
extern st_uint8 g_st_wendu_ge;
extern st_uint8 g_st_shidu_shi;
extern st_uint8 g_st_shidu_ge;
extern void st_temperature(void);
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

afAddrType_t Point_To_Point_DstAddr;//point to point communication
afAddrType_t SampleApp_Cmd_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint8 count_a = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendPointToPointMessage(void); 
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
st_uint8 st_peoplescan(void); 
st_uint8 st_message_transfer_whole(st_uint8 *a , st_uint8 len);
st_uint8 st_whole_transfer_message(st_uint8 *a , st_uint8 len);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/****************************
     The people scan
*****************************/
st_uint8 st_peoplescan(void)
{
  if(PEOPLE==0)
  {
      Delay_ms(10);
      if(PEOPLE==0)
      {      
        return 1;   // no people
      }
  }
  return 0;           //get people
}
// use it transfer the 0xF0,0xFE,0xFF to 0xF0F0,0xF0FE,0xF0FF,added by Kevin at 20140601
st_uint8 st_message_transfer_whole(st_uint8 *a , st_uint8 len)
{
  st_uint8 i,k = 0;
  st_uint8 m = 0;
  st_uint8 *p;
  for(i = 1;i < (len-3);i++){
    if((a[i] == 0xF0)||(a[i] == 0xFE)||(a[i] == 0xFF)){
      k++;
    }
  }
  if(k == 0){
      return len;
  }else{
    p = osal_mem_alloc((sizeof(st_uint8))*(k+len));
    i = 1;
    while( i<(len-3)){   //the last of the message is the 0xFEFF,it is not necessary to transfer it.
      if((a[i] == 0xF0)||(a[i] == 0xFE)||(a[i] == 0xFF)){
        p[m++] = 0xF0;
        p[m] = a[i];
        m++;
        i++;
      }else{
        p[m++] = a[i++];
      }
    }
    p[m++] = 0xFE;
    p[m] = 0xFF;
    for(m = 0;m<(k+len);m++){
      a[m] = p[m];
    }
    osal_mem_free(p);
    return k+len;
  }
}
// use it transfer 0xF0F0,0xF0FE,0xF0FF to 0xF0,0xFE,0xFF,added by Kevin at 20140601
st_uint8 st_whole_transfer_message(st_uint8 *a , st_uint8 len)
{
  st_uint8 i,k = 0;
  st_uint8 m = 0;
  st_uint8 *p;
  st_uint8 get_0xf0 = 0;
  st_uint8 j = 0;

  for(i = 0;i < (len-2);i++){
    if((a[i] == 0xF0)&&((a[i+1] == 0xF0)||(a[i+1] == 0xFE)||(a[i+1] == 0xFF))){
      if((a[i] == 0xF0)&&(a[i+1] == 0xF0)){
        if(get_0xf0 == 0){
          k++;
          get_0xf0 = 1;
        }//use bool character n ,in case of the condition that we are faced with 0xF0 one by one
        else{
          get_0xf0 = 0;
        }
      }else{
        k++;
      }
    }
  }
  if(k == 0){
      return len;
  }else{
    p = osal_mem_alloc((sizeof(st_uint8))*(len-k));
    i = 0;
    while( i<(len-2)){   //the last of the message is the 0xFEFF,it is not necessary to transfer it.
      if((a[i] == 0xF0)&&((a[i+1] == 0xF0)||(a[i+1] == 0xFE)||(a[i+1] == 0xFF))){
        if((a[i] == 0xF0)&&(a[i+1] == 0xF0)){
          if(get_0xf0 == 0){
            i++;
            get_0xf0 = 1;
          }   
          else{ //use bool character get_0xf0 ,in case of the condition that we are faced with 0xF0 one by one
            get_0xf0 = 0;
            p[m] = a[i];
            i++;
            m++;
          }
        }else{
          i++;
        }
      }else{
        p[m] = a[i];
        m++;
        i++;
      }  
    }
    p[m] = 0xFE;
    m++;
    p[m] = 0xFF;
    a[0] = len-k+1;// The length  include the first byte
    for(j=0;j<=len-k;j++)// The length include the first byte which store the length
      HalUARTWrite(0,p+j,1 ); 
    HalUARTWrite(0,"\n\n\n",3 );
    for(m = 0;m<len;m++){
      if(m<len-k){
        a[m+1] = p[m];
      }else{
        a[m+1] = 0;
      }   
    }
    osal_mem_free(p);
    return len-k+1;// return the lenth of the array
  }
}
/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  st_uint8 i;
  st_uint8 j;
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
#if defined(Kevin)
  for(i = 0;i<Z_MAX_NODE_NUM;i++){
    g_st_node_info_t[i].user=0xff;
    g_st_node_info_t[i].nwkAddr=0xffff;
    for(j = 0;j < Z_EXTADDR_LEN;j++){
      g_st_node_info_t[i].extAddr[j]=0xff;
    }
    g_st_node_info_t[i].flag=0xff;
  }
#endif // Initial the struct which store the nodes' nwkaddr,
     // extAddr and the flag(use for the security validation)
 /***********init serial************/
  MT_UartInit();
  MT_UartRegisterTaskID(task_id);//register serial taskID
  HalUARTWrite(0,"Hello World\n",12);
  
  /******pyroelectric transducer initial******/
  P2SEL &= ~0X01;     //set P20 common port  
  P2DIR &= ~0X01;    // set P20 input port 
  P2INP &= ~0x01;   // set pull-up resister
  
  P0SEL &= 0xbf;      //temperature sensor initial
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
    // Point to point communication unicast 
  Point_To_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //unicast 
  Point_To_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  Point_To_Point_DstAddr.addr.shortAddr = 0x0000;//unicast to coordinator
  

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter
 
  st_uint8 i;
  st_uint8 j;
  ZStatus_t k;
  
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        
        case CMD_SERIAL_MSG:  //串口收到数据后由MT_UART层传递过来的数据，编译时不定义MT_TASK，则由MT_UART层直接传递到此应用层
       // 如果是由MT_UART层传过来的数据，则上述例子中29 00 14 31都是普通数据，串口控制时候用的。 
          SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);                    
          break;
        
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
        
        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          HalUARTWrite(0,"The Node join\n" ,14); 
          if ( //(SampleApp_NwkState == DEV_ZB_COORD)Coordinator can't unicast to itself
                (SampleApp_NwkState == DEV_ROUTER)
             || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            #if defined (Kevin)
            #else
              st_uint8 value = ST_ZIGBEE_KEY;
              st_uint8 value_read = 0;
              st_uint16 nwkaddress=0xffff; 
              st_uint8 buf[5] = {0x0};
              
              osal_nv_item_init(NV_ID,1,NULL);
              osal_nv_write(NV_ID,0,1,&value);
              osal_nv_read(NV_ID,0,1,&value_read);//read the ST_ZIGBEE_KEY from NV
              
              HalUARTWrite(0,"Send the Key to Coor\n" ,21);
              nwkaddress = NLME_GetShortAddr();
              NwkAddr.Addr16 = nwkaddress;
              
              buf[0] = NwkAddr.Addr8[0];
              buf[1] = NwkAddr.Addr8[1];
              buf[2] = value;
              buf[3] = NODE_ID;
              buf[4] = 0;
              if ( AF_DataRequest( &Point_To_Point_DstAddr,
                 &SampleApp_epDesc,
                 SAMPLEAPP_ZIGBEE_SECURITY_KEY_CLUSTERID,
                 5,
                 buf,
                 &SampleApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
              {
              }
              else
              {
              // Error occurred in request to send.
              }/* Added by Kevin at 20140427, the end device will send its Nwkaddr
                and the zigbee join_key to the coordinator, as soon as join the 
                zigbee net*/
           #endif
          }
          if ( //(SampleApp_NwkState == DEV_ZB_COORD)Coordinator can't unicast to itself
                (SampleApp_NwkState == DEV_ROUTER)
             || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              g_st_send_periodic_msg_timeout*200 );
          }
#if defined (Kevin)          
          else if(SampleApp_NwkState == DEV_ZB_COORD){
            osal_start_timerEx( SampleApp_TaskID,
                              SYS_EVENT_IEEE,
                              SAMPLEAPP_IEEE_PERIODIC_CHECK_TIMEOUT );
          }
#endif
          else
          {
            // Device is no longer in the network
          }
          break;
          
#if defined (Kevin)
      case ST_IEEE_ADDRESS:
        HalUARTWrite(0,"Ask the IEEE address of the node!\n",34);
        ZDP_IEEEAddrReq(g_st_node_info_t[st_nodenum].nwkAddr, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
        break;// Coordinator ask the node to send the IEEE address .
#endif
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    // SampleApp_SendPeriodicMessage();
    st_uint8 T[8];     //Temperature + People + Node information
    st_temperature();   //Temperature
    P0DIR |= 0x40; //IO port need to reconfigure
   
    
    SampleApp_SendPointToPointMessage();  
    // Setup to send message again in normal period (+ a little jitter)

    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (g_st_send_periodic_msg_timeout*200 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
#if defined (Kevin)
  if ( events & SYS_EVENT_IEEE )
  {
 //   zAddrType_t *DeleteAddr; 
    for(i = 0;i<Z_MAX_NODE_NUM;i++){     
        if(g_st_node_info_t[i].flag != ST_ZIGBEE_KEY){
          //according to the record ,the node join the network，but it does not send the security validation
          if(g_st_node_info_t[i].nwkAddr != 0xffff){
 //           DeleteAddr->addr.shortAddr = g_st_node_info_t[i].nwkAddr;
 //           DeleteAddr->addrMode = Addr16Bit;
  //          ZDP_MgmtLeaveReq( DeleteAddr, g_st_node_info_t[i].extAddr, 0,
  //            0, 0 );
            
            LeaveReq->extAddr = g_st_node_info_t[i].extAddr;
            LeaveReq->removeChildren = TRUE; 
            LeaveReq->rejoin = FALSE;
            LeaveReq->silent = FALSE;
            k = NLME_LeaveReq(LeaveReq);
 //         NLME_RemoveChild(g_st_node_info_t[i].extAddr,0);
            AssocRemove( g_st_node_info_t[i].extAddr );
            HalUARTWrite(0,"Delete the Strange Node\n",25);
            g_st_node_info_t[i].nwkAddr = 0xffff;
            for(j = 0;j<Z_EXTADDR_LEN;j++){  
              g_st_node_info_t[i].extAddr[j] = 0xff;
            }
            break;
          }//Delete the node from the net which don't send the correct ST_ZIGBEE_KEY,
          //and delete the node information in the struct.
        }
    }
    osal_start_timerEx( SampleApp_TaskID, SYS_EVENT_IEEE,
        (SAMPLEAPP_IEEE_PERIODIC_CHECK_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SYS_EVENT_IEEE);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  /*hex  ASCII*/
  //uint8 asc_16[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'},i;
  st_uint16 flashTime;
  st_uint8 j;
  st_uint8 value = ST_ZIGBEE_KEY;
  st_uint8 value_read = 0;
  st_uint8 len = 0;
  st_uint8 len_new_buff = 0;
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
      HalUARTWrite(0,"The Node:",9); 
      HalUARTWrite(0,&pkt->cmd.Data[7],1); 
      if(pkt->cmd.Data[0])
        HalUARTWrite(0,"    Got People",14);     //got people
      else 
        HalUARTWrite(0,"    No People",13);     //no people   
      /***********print temperature***************/
      HalUARTWrite(0,"    Temp is:",12);  
      HalUARTWrite(0,&pkt->cmd.Data[1],2); 
      HalUARTWrite(0,".",1);  
      HalUARTWrite(0,&pkt->cmd.Data[3],1); 
      HalUARTWrite(0,"\n",1);  
      /***************humidity****************/
      HalUARTWrite(0,"    Humidity is:",16);   
      HalUARTWrite(0,&pkt->cmd.Data[4],2);  
      HalUARTWrite(0,".",1);  
      HalUARTWrite(0,&pkt->cmd.Data[6],1);
      HalUARTWrite(0,"\n",1); 
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
#if defined (Kevin)
    case SAMPLEAPP_ZIGBEE_SECURITY_KEY_CLUSTERID:

      NwkAddrRecive.Addr8[0] = pkt->cmd.Data[0];
      NwkAddrRecive.Addr8[1] = pkt->cmd.Data[1];      
    //  buf0[0] = (ShortAddr0 >> 4 )+ '0';
    //  buf0[1] = (ShortAddr0&&0xf )+ '0';
      
     // buf1[0] = (ShortAddr1 >> 4)&&(0x0f) + 0x32;
     // buf1[1] = ShortAddr1&&0xf + 0x32;
     // HalUARTWrite(0,"The ShortAddr is \n",17);
    //  HalUARTWrite(0,&buf0[0],1);
    //  HalUARTWrite(0,&buf0[1],1);
      HalUARTWrite(0,"\n",1);
      HalUARTWrite(0,"Get the Node ",13);
      HalUARTWrite(0,&pkt->cmd.Data[3],1);
      HalUARTWrite(0,"  Key\n",6);
     
      osal_nv_item_init(NV_ID,1,NULL);
      osal_nv_write(NV_ID,0,1,&value);
      osal_nv_read(NV_ID,0,1,&value_read);
      
      for(j = 0;j<Z_MAX_NODE_NUM;j++){
        if (NwkAddrRecive.Addr16 == g_st_node_info_t[j].nwkAddr){
          if(pkt->cmd.Data[2] == value_read){
            g_st_node_info_t[j].flag = value_read;
            break;
          }
        }
      }
      j = 0;
#endif//the Coordinator receive the ST_ZIGBEE_KEY from the node , and compares it with 
      // the key that it keeps.      
      case SAMPLEAPP_COM_CLUSTERID:
        if ((SampleApp_NwkState == DEV_ROUTER)|| (SampleApp_NwkState == DEV_END_DEVICE) ){
          st_uint16_uint8_t serial_cmd;
          st_uint16_uint8_t addr_transfer;
          st_uint32_uint8_t msg_tmp_t;
          len = pkt->cmd.Data[0];
          addr_transfer.u8[0] = pkt->cmd.Data[4];
          addr_transfer.u8[1] = pkt->cmd.Data[5];
          
          //check whether the dest is mine
          
          serial_cmd.u8[0] = pkt->cmd.Data[9];
          serial_cmd.u8[1] = pkt->cmd.Data[8];//make the tw0 char into a uint16,using union
          switch(serial_cmd.u16){
            case ST_OPT_GET_PROP:
              cmd_t.u8[0] = pkt->cmd.Data[11];
              cmd_t.u8[1] = pkt->cmd.Data[10];
              switch(cmd_t.u16){}
              break;
              
            case ST_OPT_SET_PROP:
              cmd_t.u8[0] = pkt->cmd.Data[11];
              cmd_t.u8[1] = pkt->cmd.Data[10];
              switch(cmd_t.u16){ 
                case ST_PROP_T1: 
                  msg_tmp_t.u8[0] = pkt->cmd.Data[15];
                  msg_tmp_t.u8[1] = pkt->cmd.Data[14];
                  msg_tmp_t.u8[2] = pkt->cmd.Data[13];
                  msg_tmp_t.u8[3] = pkt->cmd.Data[12];
                  HalUARTWrite(0,&pkt->cmd.Data[13],4); 
                  HalUARTWrite(0,"\n\n\n",3);
                  g_st_send_periodic_msg_timeout = msg_tmp_t.u32;
                  break;
    //                    case ST_PROP_T2:
    //                      break;
    //                    case ST_PROP_C1:
    //                      break;
    //                    case ST_PROP_C2:
    //                      break;
    //                    case ST_PROP_H1:
    //                      break;
    //                    case ST_PROP_H2:
    //                      break;
                default:
                  break;
              }
              break;
          case ST_OPT_GET_POWER_MODE:
            break;
          default:
            break;
        }
        break;
      }
      if (SampleApp_NwkState == DEV_ZB_COORD ){
        
      }
      break;
    default:
      break;
   } 
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  uint8 data[10]={0,1,2,3,4,5,6,7,8,9};
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       10,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SampleApp_SendPointToPointMessage( void )//Send node sensor information
{
  st_uint8 transbuf[8];
  st_uint8 people_status = 1;
  people_status = st_peoplescan();
  transbuf[1]=g_st_wendu_shi+48;
  transbuf[2]=g_st_wendu_ge+48;
  transbuf[3]=g_st_wendu_xiaoshu+48; 
  transbuf[4]=g_st_shidu_shi+48;
  transbuf[5]=g_st_shidu_ge+48;
  transbuf[6]=g_st_shidu_xiaoshu+48;    
  transbuf[7]=NODE_ID;
  if(people_status==0)
  { 
    transbuf[0]=1;
    HalUARTWrite(0,"Got People\n",11);   
    if ( AF_DataRequest( &Point_To_Point_DstAddr,
                       &SampleApp_epDesc,
                       SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                       8,
                       transbuf,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
    // Error occurred in request to send.
    }
   // HalLcdWriteString( "Got People", HAL_LCD_LINE_3 ); //LCD
  }  
  else 
  {
    transbuf[0]=0;//No people
    HalUARTWrite(0,"No People\n",10);  
    count_a++;
    if(count_a == 3){
      count_a = 0;
      if ( AF_DataRequest( &Point_To_Point_DstAddr,
                       &SampleApp_epDesc,
                       SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                       8,
                       transbuf,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
      }
      else
      {
      // Error occurred in request to send.
      }
    }
  }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)//send FE 02 01 F1  , feedback 01 F1
{
  st_uint8 len_str,*str=NULL;
 // str = cmdMsg->msg;

  if(SampleApp_NwkState == DEV_ZB_COORD){   
    len_str = *str;
    st_uint8 len = 0;
    st_uint8 num = 0;
    st_uint8 index = 0;
    st_uint8 i,j = 0;
    st_uint8 k = 0;
    st_uint8 m = 0;
    st_uint8 len_new_buff = 0;
    st_uint8 len_msg = 0;
    j = cmdMsg->msg [0]; 
//    for(m = 0;m<=j;m++)
//      HalUARTWrite(0,cmdMsg->msg+m,1 ); 
    
    while(index<=j){// here should be <,but with the < the program can't go into the 0xFE
      index++;
      if(g_st_get_header){
        if(cmdMsg->msg[num] == 0xFF){
          if(cmdMsg->msg[num-1] == 0xFE){ // if the first element is 0xFF ,how to do
            g_st_message_from_app[i] = cmdMsg->msg[num];
            len = i+1;
            i = 0;
            num = 0;
            g_st_get_header = 0;
            g_st_start_check_header = 0;
            index = 0;
            st_uint16 shortaddr;
            st_nwkaddress_t nwkaddr_t;
            
            //can't print some kinds of command when receive the uart message in one time     
//            for(k=0;k<len;k++)
//              HalUARTWrite(0,g_st_message_from_app+k,1 ); 
//            HalUARTWrite(0,"\n",1 );
            
            len_new_buff = st_whole_transfer_message(g_st_message_from_app,len);
            //The len_new_buff include real data +1. The first of the g_st_message_from_app store the length
            
            for(k=0;k<len_new_buff;k++)
              HalUARTWrite(0,g_st_message_from_app+k,1 ); 
            HalUARTWrite(0,"\n",1 );
            
            nwkaddr_t.Addr8[0] = g_st_message_from_app[3];
            nwkaddr_t.Addr8[1] = g_st_message_from_app[4];// set the dst addr from two uint8 to uint16
            
            SampleApp_Cmd_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //unicast 
            SampleApp_Cmd_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
            SampleApp_Cmd_DstAddr.addr.shortAddr = nwkaddr_t.Addr16; // unicast to the node which our cubieboard set
            if(nwkaddr_t.Addr16 == 0x0000){
              len_msg = st_message_transfer_whole(g_st_message_from_app,len_new_buff);             
              for(k=0;k<len_msg;k++)
                HalUARTWrite(0,g_st_message_from_app+k,1 ); 
              HalUARTWrite(0,"\n",1 );
            }else{
              if ( AF_DataRequest( &SampleApp_Cmd_DstAddr, &SampleApp_epDesc,// the addr we are sending need to be added
                           SAMPLEAPP_COM_CLUSTERID,
                           len_new_buff,// data length         
                           g_st_message_from_app,//data 
                           &SampleApp_TransID,//  cluster ID
                           AF_DISCV_ROUTE,
                           AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
              {
              }
              else
              {
                // Error occurred in request to send.
              }
            }
            for(i = 0; i< len;i++)
              g_st_message_from_app[i] = 0;
            break;// This break should be removed to receive more command in one message one time
          }
        }
        g_st_message_from_app[i] = cmdMsg->msg[num];
        i++;
        num++;
      }else{
        if(g_st_start_check_header == 0){  
          if(cmdMsg->msg[num]==ST_VERSION_1_0 ){
            g_st_start_check_header = 1;
            num++;
          }else{
            num++;
          }
        }else{
          if(cmdMsg->msg[num] == ST_APP_DO_OPERATION){
            g_st_get_header = 1;
            g_st_message_from_app[0] = ST_VERSION_1_0;
            g_st_message_from_app[1] = cmdMsg->msg[num];
            i=2;
            num++;
          }
        }
      }// added by Kevin at 20130531
    }
  }  
}
/*********************************************************************
*********************************************************************/
