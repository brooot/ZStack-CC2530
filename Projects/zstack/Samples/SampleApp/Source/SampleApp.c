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
#include "MT.h"  //透传中CMD_SERIAL_MSG的定义

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"
#include "string.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"



//用户引入的头文件
#include "MT_UART.h"
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

/*********************************************************************
 * EXTERNAL VARIABLES
 */

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
afAddrType_t SampleApp_Flash_DstAddr; //组播

afAddrType_t Point_To_Point_DstAddr; //WeBee点对点通信定义

aps_Group_t SampleApp_Group; //分组内容

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS     函数声明
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg); // 串口通信
void SampleApp_SendPointToPointMessage( void ); //点对点
void SampleApp_SendGroupMessage( void );// 网蜂组播通讯发送函数定义
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

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
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  MT_UartInit(); //初始化UART
  MT_UartRegisterTaskID(task_id);// 登记任务号
  HalUARTWrite(0, "Hello World\n", 12); // （串口 0 ，'字符’,字符个数）







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



  //WeBee点对点通讯定义
  Point_To_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播模式
  Point_To_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  Point_To_Point_DstAddr.addr.shortAddr = 0x0000; //发给协调器






  // Setup for the periodic message's destination address
  // Broadcast to everyone     广播方式定义
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;//发送给所有设备,睡眠中的节点由父节点暂存直到超时
  //0xFFFD 发送给非睡眠状态的所有设备
  //0xFFFC 发送给所有路由器以及协调器







  //组播Group1设置
  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;  //endpoint 端点
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;





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



  /***********设置 Group_ID*************/
  // By default, all devices start out in Group 1
  SampleApp_Group.ID = SAMPLEAPP_FLASH_GROUP; //0x0001
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
 *          ******************应用处理程序****************************
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG ) //系统信息发送事件
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case CMD_SERIAL_MSG:    // 串口收到数据后由MT_UART层传递过的数据，用网蜂方法接收，编译时不定义MT相关内容
            SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
            break;

        // Received when a key is pressed
        case KEY_CHANGE: //按键事件及处理函数
          HalUARTWrite(0, "KEY ", 4);
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        // 接收处理函数
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:  //网络状态改变，如未连接上网络
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD) //协调器
              || (SampleApp_NwkState == DEV_ROUTER) //路由器
              || (SampleApp_NwkState == DEV_END_DEVICE) ) //终端 都执行
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
//              HalUARTWrite(0,"网络断开",8);
          }
          break;

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
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )  //周期性发送函数
  {
    /*Send the periodic message*/
    SampleApp_SendPeriodicMessage(); //用户周期性广播发送函数

    /*Point_to_Point*/
    //SampleApp_SendPointToPointMessage(); //点对点发送函数

    /*Group*/
    SampleApp_SendGroupMessage(); //组播通讯程序


    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}




/*自行定义的 处理串口发来数据包 的函数*/
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)
{
    uint8 i,len,*str=NULL; //len 有用数据长度
    str=cmdMsg->msg; //指向数据开头
    len=*str; //msg 里的第 1 个字节代表后面的数据长度
    /********打印出串口接收到的数据，用于提示*********/
    HalUARTWrite(0,"我说: ",strlen("我说: "));
    for(i=1;i<=len;i++)
    {
      HalUARTWrite(0,str+i,1);
    }

    HalUARTWrite(0,"\n",1 );//换行

    //模拟异或操作
//    int8 A[] = {'1','0','0','0','0','0','0','1'};
//    unsigned char One = '1',Zero = '0';
//    for (i=0;i<8;++i)
//    {
//      if(A[i]-*(str+i+1))
//      {
//        HalUARTWrite(0,&One,1);
//        *(str+1+i) = One;
//      }
//      else
//      {
//        HalUARTWrite(0,&Zero,1);
//        *(str+1+i) = Zero;
//      }
//
//    }
//



//
//    /*下面瞎写的*/
//    if( *(str) == 4)
//    {
//      uint8 data[10]={'0','1','2','3','4','5','6','7','8','9'};
//
//      //HalUARTWrite(0,"get an 'a' !",strlen("get an 'a' !") );
//      if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
//                          SAMPLEAPP_COM_CLUSTERID,//自己定义一个
//                          10, // 数据长度
//                          data, //数据内容
//                          &SampleApp_TransID,
//                          AF_DISCV_ROUTE,
//                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
//      {
//      }
//
//
//      else
//      {
//        // Error occurred in request to send.
//      }
//    }
//    /*上面瞎写的*/


    /*******发送出去***参考网蜂 1 小时无线数据传输教程*********/
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                        SAMPLEAPP_COM_CLUSTERID,//自己定义一个
                        len+1, // 数据长度
                        str, //数据内容
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





/*********************************************************
                   自定义点对点发送函数
**********************************************************/
void SampleApp_SendPointToPointMessage( void )
{
  uint8 data[11]={'f','r','o','m',' ','r','o','u','t','e','1'};
  if ( AF_DataRequest( &Point_To_Point_DstAddr, //点对点通信方式目标地址
                      &SampleApp_epDesc,
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //点对点簇ID
                      12,
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







/********************************************************
                    组播 发送函数
*********************************************************/
void SampleApp_SendGroupMessage( void )
{
  uint8 data[10]={'G','R','O','U','P',' ','2',' ','r','1'};// 自定// 义数据
  if ( AF_DataRequest( & SampleApp_Flash_DstAddr,
                      &SampleApp_epDesc,
                      SAMPLEAPP_FLASH_CLUSTERID,
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


/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 *
 *********************************************************
                    周期性广播 发送函数
 **********************************************************/
void SampleApp_SendPeriodicMessage( void )
{
  uint8 data[10]={'f','r','o','m',' ','b','r','o',' ','4'};
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       10,    //数据长度
                       data,  //指针方式
//                       1,  //数据长度
//                       (uint8*)&SampleAppPeriodicCounter, //要发送的内容
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





/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.

            ****************接收函数*********************
 *          处理接收到的数据,可能是从其他设备收到的数据.
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime=1000;//一秒,1000ms
  uint8 i,len; //透传
  switch ( pkt->clusterId )
  {

    /*收到 点对点 消息*/
  case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
    HalUARTWrite(0, "收到点对点消息: ", strlen("收到点对点消息: "));  //给串口发送提示
    HalUARTWrite(0, &pkt->cmd.Data[0], 11); // 打印收到数据
    HalUARTWrite(0, "\n", 1); // 回车换行
    break;

    /*收到 周期性 消息*/
  case SAMPLEAPP_PERIODIC_CLUSTERID:
                HalUARTWrite(0, "周期性消息: ", strlen("周期性消息: "));  //给串口发送提示
                HalUARTWrite(0, &pkt->cmd.Data[0], 10); // 打印收到数据
                HalUARTWrite(0, "\n", 1); // 回车换行
                HalLedBlink( HAL_LED_3, 3, 50, (flashTime / 4) ); //三号灯闪烁,三次,亮度50%,时间间隔0.25s
    break;

//    /*组播消息*/
//  case SAMPLEAPP_FLASH_CLUSTERID:
//    flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
//    HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
//    break;

    /*收到 对方串口透传 消息*/
  case SAMPLEAPP_COM_CLUSTERID:
    HalUARTWrite(0,"对方说: ",strlen("对方说: "));
    len=pkt->cmd.Data[0];
    for(i=0;i<len;i++)
      HalUARTWrite(0,&pkt->cmd.Data[i+1],1);//发给 PC 机
    HalUARTWrite(0,"\n",1); // 回车换行
    break;

    /*收到 组播 消息*/
  case SAMPLEAPP_FLASH_CLUSTERID:
    HalUARTWrite(0,"组播消息:",strlen("组播消息:"));// 用于提示有数据
    HalUARTWrite(0, &pkt->cmd.Data[0],10); // 打印收到数据
    HalUARTWrite(0,"\n",1); //
    HalLedBlink( HAL_LED_2, 2, 50, (flashTime / 5) ); //二号灯闪烁,两次,亮度50%,时间间隔0.2s
    break;
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
    //SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
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
  //判断按键S2按下，发送数据到串口
  if ( keys & HAL_KEY_SW_6 )
  {
    HalUARTWrite(0,"S1\n",3);
    HalLedBlink( HAL_LED_3, 2,50, 500 ); //LED1 闪烁提示
  }
}






/*********************************************************************
*********************************************************************/
