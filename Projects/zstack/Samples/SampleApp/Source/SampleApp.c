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

extern uint16 NLME_GetShortAddr( void ); //获取自身短地址
extern void NLME_UpdateLinkStatus( void ); //更新连接状态



/*********** 注意: 每个设备不一样,记得修改!  **********/
uint8 SOURCE_DATA = '3';  //采集的数据
uint8 Device_ID = 3;



/*********************************************************************
 * MACROS
 */
#define random(x) (rand() % x)
#define uchar unsigned char
#define Node_Num uint8 3  //网络中节点的数量
/*********************************************************************
 * CONSTANTS
 */
//用户引入的头文件
#include "MT_UART.h"
#include <stdio.h>
#include "nwk_util.h"
#include <stdlib.h>
#include "OSAL_Clock.h"
#include <math.h>


/*********************************************************************
 * TYPEDEFS
 */
/*<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
 * GLOBAL VARIABLES 全局变量
 */
int8 LIFE_TIME=5;               //邻居活跃度初始值
int8 NEI_NUM = 0;               //邻居数量
int16 Nei_Table[3] = {0};       //邻居表
unsigned short ACTIVE[3] = {0}; //邻居活跃度列表
int8 _TURN = 0;                 //发送轮次控制


/*#######################以下是Sink节点的配置############################*/

/*一个码字的组成*/
typedef struct Encoded_PKT  //编码过的码字
{
    /*数据域*/
    short data; //码字数据
    /*编码信息,由哪几个节点的数据包组成*/
    uint16 Degree_ID;
}PKT;


const int Slot_Size = 6;        //码字槽大小

PKT DATA_SLOT[5+1];             //码字结构体数组


//uint8 DATA_SLOT[5+1] = {0};     //码字槽
//uint8 DATA_Degree[5+1] = {0};   //码字度
unsigned short MAX_Degree = 1;  //初始化允许的最大度
uint8 Decoded_X[100] = {0};     //初始化已解码的码字数组
uint8 Undecoded_Y[100] = {0};   //初始化未解码的复杂码字数组
int16 Exchanging_Neighbor;      //待进行码字交换的邻居
uint8 Exchanging_Data;          //待交换的码字
uint8 Exchange_index;           //待交换的码字的索引

uint8 Waiting_Request = FALSE;   //是否在等待对方返回交换的数据包回应
uint8 Exchange_Lock = FALSE;
/*<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
* end
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
void Reset_down_neighbor(int16 nei_addr); //重置邻居设备函数
void Show_Bit(uint8 X, uint8 N); //串口打印一个8位比特二进制数字
void Show_Num8(uint8 X);  //串口打印8bit数字
uint8 Power_of_2(uint8 X);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void Show_Bit( uint8 X, uint8 N )
{
  uchar Bit[2] = {'0','1'};
  uint8 index;
  for(short j=N-1;j>=0;--j)
  {
    index = (X>>j) & 1;
    HalUARTWrite(0,&Bit[index],1);
  }
  HalUARTWrite(0,"\n",1);
}

void Show_Num8( uint8 X )
{
  uchar temp = '0' + X;
  HalUARTWrite(0,&temp,1);
}

uint8 Power_of_2( uint8 X )
{
  if(!X)
    return 1;
  else
  {
    uint8 temp = 2;
    for(uchar i=2;i<=X;++i)
    {
      temp *=2;
    }
    return temp;
  }
    
}
/*********************************************************************
 * @fn      SampleApp_Init   应用初始化
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
  HalUARTWrite(0, "System is started.\n\n", 20); // （串口 0 ，'字符’,字符个数）


  //路由表初始化
  for(short i=0;i<3;++i)
  {
    Nei_Table[i] += 65534;
    ACTIVE[i] += LIFE_TIME;  //邻居活跃度初始化
  }


  //给码字槽赋初值(除了sink节点)
  for(short i=0;i<6;++i)
  {
    DATA_SLOT[i].data = SOURCE_DATA;  //给码字槽赋初值
    DATA_SLOT[i].Degree_ID = Power_of_2(i);       //给码字赋节点信息
  }

  
 
  
  
  
  
  
  
  
  
  
  

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
  Point_To_Point_DstAddr.addr.shortAddr = 0xB6AE; //发给协调器






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
 *        ******************应用处理程序***************************
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
          //HalUARTWrite(0, "KEY ", 4);
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
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )  //*****周期性发送函数*****
  {


/***********************************周期性发送函数****************************************/

    /*Send the periodic message*/
    //SampleApp_SendPeriodicMessage(); //用户周期性广播发送函数

    if(_TURN == 0){
      /*Group*/
      SampleApp_SendGroupMessage(); //组播通讯程序
      _TURN = 1;
    }

    else{
      /*Point_to_Point*/
      SampleApp_SendPointToPointMessage(); //点对点发送函数
      _TURN = 0;
    }

/**********************************周期性发送函数*****************************************/




    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                       (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);

  }

  // Discard unknown events
  return 0;
}







/********************************************************
                    组播 发送函数
*********************************************************/
void SampleApp_SendGroupMessage( void )
{
  //uint8 data[10]={'G','R','O','U','P',' ','1',' ','r','4'};// 自定义数据
  if ( AF_DataRequest( & SampleApp_Flash_DstAddr,
                      &SampleApp_epDesc,
                      SAMPLEAPP_FLASH_CLUSTERID,
                      1,
                      (uint8*)&SampleAppPeriodicCounter, //要发送的内容
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
  unsigned int flashTime=1000;
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,  //数据长度
                       (uint8*)&SampleAppPeriodicCounter, //要发送的内容
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    HalLedBlink( HAL_LED_1, 1, 60, (flashTime / 3) );
  }
  else
  {
    // Error occurred in request to send.
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
 * @fn      SampleApp_HandleKeys 按键处理函数
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
    int self_addr_10 = NLME_GetShortAddr();
    int neig_addr_10;
    uchar Self_ADDR[4]; //16进制地址格式
    uchar Neig_ADDR[4];
    uchar Num[2];

    //串口输出自身地址
//    HalUARTWrite(0,"自身地址为: 0x",strlen("自身地址为: 0x"));
//    if(self_addr_10 == 0)
//    {
//      HalUARTWrite(0,"0000",4);
//      HalUARTWrite(0,"\n",1);
//    }
//    else
//    {
//      sprintf((char*)Self_ADDR, "%X", self_addr_10); //10进制整型转换成16进制字符串
//
//      for (int i=0;i<4;++i)
//      {
//        HalUARTWrite(0,&Self_ADDR[i],1);
//      }
//      HalUARTWrite(0,"\n",1);
//    }

    //串口输出源数据
    HalUARTWrite(0,"源数据是: ",strlen("源数据是: "));
    Show_Bit(SOURCE_DATA,8);
    for(short p=1;p<6;++p)
    {
      Show_Bit(DATA_SLOT[p].data, 8);
      HalUARTWrite(0,"码字信息: ",strlen("码字信息: "));
      Show_Bit(DATA_SLOT[p].Degree_ID, 16);
    }
    
    HalUARTWrite(0,"邻居个数:",5);
    uint8 nei_N = 0;
    for(int num_nei=0;num_nei<3;++num_nei)
    {
      if(Nei_Table[num_nei] != (uint16)65534)
        nei_N++;
    }
    Show_Bit(nei_N, 8);
    //串口输出所有邻居地址
//    for(int i=0;i<3;++i)
//    {
//      neig_addr_10 = Nei_Table[i];
//
//      sprintf((char*)Num, "%d", i+1);
//      sprintf((char*)Neig_ADDR, "%X", neig_addr_10);
//      HalUARTWrite(0,"邻居",strlen("邻居"));
//      for (int j=0;j<2;++j)
//      {
//        HalUARTWrite(0,&Num[j],1);
//      }
//
//      HalUARTWrite(0,": 0x",strlen(": 0x"));
//
//      if(neig_addr_10 == 0)
//      {
//        HalUARTWrite(0,"0000",4);
//      }
//      else
//      {
//        for (int j=0;j<4;++j)
//        {
//          HalUARTWrite(0,&Neig_ADDR[j],1);
//        }
//      }
//      HalUARTWrite(0,"\n",1);
//    }

//    HalLedBlink( HAL_LED_3, 2,50, 500 ); //LED3 闪烁提示
  }
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


//    /*******发送出去***参考网蜂 1 小时无线数据传输教程*********/
//    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
//                        SAMPLEAPP_COM_CLUSTERID,//自己定义一个
//                        len+1, // 数据长度
//                        str, //数据内容
//                        &SampleApp_TransID,
//                        AF_DISCV_ROUTE,
//                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
//    {
//    }
//
//
//    else
//    {
//        // Error occurred in request to send.
//    }
}







/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB  接收函数
 *
 *          处理接收到的数据,可能是从其他设备收到的数据.
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime=1000;//一秒,1000ms
  uint8 i,len; //透传
  unsigned short neighbor_addr_10 = pkt->srcAddr.addr.shortAddr;  //发送方的数字地址
  unsigned char ADDR[4]; //数字地址转换后的字符地址
  int8 LOST_nei = 0;
  uchar active;
  
  uchar BUSY[4] = "####";
  uint8 Peer_data; //对方数据
  int16 Peer_info; //对方码字信息
  uint8 Self_data; //己方数据
  int16 Self_info; //己方码字信息
  int8 degree_low; //码字信息低位
  int8 degree_high;//码字信息高位
  switch ( pkt->clusterId )
  {

    /*************************收到 点对点 消息********************************/
  case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
    //收到重置信号
    if(pkt->cmd.Data[0] == '!')
    {
      HalUARTWrite(0,"重置中...",strlen("重置中..."));
      SystemResetSoft();
    }

    HalUARTWrite(0, "收到点对点消息: ", strlen("收到点对点消息: "));  //给串口发送提示
    HalUARTWrite(0, &pkt->cmd.Data[0], 1); // 打印收到数据
    Show_Bit(pkt->cmd.Data[1],8);

    sprintf((char*)ADDR, "%X", neighbor_addr_10);
    HalUARTWrite(0," from ",6);
    HalUARTWrite(0,ADDR,4);
    HalUARTWrite(0, "\n\n", 2); // 回车换行

    /*如果在自身已经发起交换请求的情况下,收到对方发来的交换数据包响应*/
    if(Waiting_Request && (neighbor_addr_10 == Exchanging_Neighbor)) 
    {
      /*对方正忙*/
      if(pkt->cmd.Data[0] == '#' && pkt->cmd.Data[1] == '#' && pkt->cmd.Data[2] == '#' && pkt->cmd.Data[3] == '#')
      {
        Waiting_Request = FALSE;
        HalUARTWrite(0,"peer is busy",12);
        HalUARTWrite(0,"\n",1);
//        SampleApp_SendPointToPointMessage(); //再次发起交换请求
      }
      /*对方正常响应*/
      if(!Exchange_Lock && (pkt->cmd.Data[0] == '$'))
      {
        Exchange_Lock = TRUE; //开启码字槽的锁
        
        Peer_data = pkt->cmd.Data[1]; //对方发来的数据
        Peer_info = pkt->cmd.Data[3]; 
        Peer_info |= pkt->cmd.Data[2]<<8; //对方码字信息
        //查看发来的数据包的度
        uint8 Peer_degree = 0;
        uint8 Self_included = FALSE;
        for(short k=0;k<16;++k)
        {
          if(1 & Peer_info>>k)
          {
            ++Peer_degree;
            if(k == Device_ID)
            {
              Self_included = TRUE; //码字包含自身的节点数据包
              HalUARTWrite(0,"包含自身信息",12);
            }
            
          }
        }
        //判断是否符合编码条件,不包含自身的数据包,且度不大于最大允许的度
        if(!Self_included && (Peer_degree < MAX_Degree))
        {
          Peer_data ^= DATA_SLOT[0].data; //异或编码
          Peer_info &= Device_ID; //码字信息中添加自身的设备ID
          HalUARTWrite(0,"已编码\n编码后的值是:",strlen("已编码\n编码后的值是:"));
          Show_Bit(Peer_data, 8);
        }
        else
        {
          HalUARTWrite(0,"未编码\n对方的值是:",strlen("未编码\n对方的值是:"));
          Show_Bit(Peer_data, 8);
        }
        Show_Num8(Peer_degree);
        DATA_SLOT[Exchange_index].data = Peer_data; //对应位置替换为对方的码字
        DATA_SLOT[Exchange_index].Degree_ID = Peer_info; //记录码字信息
        
        
        Exchange_Lock = FALSE; //关闭码字槽的锁
      }
      
      
      
      
      
    }

    /*在等待对方返回交换数据包的过程中收到其他节点的交换请求*/
    else if(Waiting_Request && (neighbor_addr_10 != Exchanging_Neighbor) && (pkt->cmd.Data[0] == '*')) 
    {
      /*返回正忙信号 "####" */ 
      Point_To_Point_DstAddr.addr.shortAddr = neighbor_addr_10;
      if(AF_DataRequest( &Point_To_Point_DstAddr, //点对点通信方式目标地址
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //点对点簇ID
                        4,
                        BUSY,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS )== afStatus_SUCCESS )
      {
      }
      else
      {
      }
      HalUARTWrite(0,"i am busy!\n",strlen("i am busy!\n"));
      
    }

    /*如果收到对方发来的交换数据包请求*/
    else if(!Exchange_Lock)
    {
      Exchange_Lock = TRUE; //开启码字槽的锁
      /*挑选自身随机的数据包发送给对方*/
      srand((int)osal_getClock());
      Exchange_index = random(5)+1;//挑选随机交换码字
      Self_data = DATA_SLOT[Exchange_index].data; //即将交换的码字
      Self_info = DATA_SLOT[Exchange_index].Degree_ID; //码字信息
      degree_high = (uint8)Self_info>>8;
      degree_low = (uint8)Self_info;
      //发送
      uint8 _DATA[4] = {'$', Self_data, degree_high, degree_low};
      Point_To_Point_DstAddr.addr.shortAddr = neighbor_addr_10;
      if(AF_DataRequest( &Point_To_Point_DstAddr, //点对点通信方式目标地址
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //点对点簇ID
                        4,
                        _DATA,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS )== afStatus_SUCCESS )
      {
      }
      else
      {
      }
      
      /*将对方发来的数据包和自身随机挑选的数据包编码 <先判断是否需要编码> */
      Peer_data = pkt->cmd.Data[1]; //对方发来的数据
      Peer_info = pkt->cmd.Data[3]; 
      Peer_info |= pkt->cmd.Data[2]<<8; //对方码字信息
      //查看发来的数据包的度
      uint8 Peer_degree = 0;
      uint8 Self_included = FALSE;
      for(short k=0;k<16;++k)
      {
        if(1<<k & Peer_info)
        {
          ++Peer_degree;
          if(k == Device_ID)
            Self_included = TRUE; //码字包含自身的节点数据包
          
        }
      }
      //判断是否符合编码条件,不包含自身的数据包,且度不大于最大允许的度
      if(!Self_included && (Peer_degree < MAX_Degree))
      {
        
        Peer_data ^= DATA_SLOT[0].data; //异或编码
        Peer_info &= Device_ID; //码字信息中添加自身的设备ID
        HalUARTWrite(0,"已编码\n编码后的值是:",strlen("已编码\n编码后的值是:"));
        Show_Bit(Peer_data, 8);
      }
      else
      {
        HalUARTWrite(0,"未编码\n对方的值是:",strlen("未编码\n对方的值是:"));
        Show_Bit(Peer_data, 8);
      }
      DATA_SLOT[Exchange_index].data = Peer_data; //对应位置替换为对方的码字
      DATA_SLOT[Exchange_index].Degree_ID = Peer_info; //记录码字信息
      
      Exchange_Lock = FALSE; //关闭码字槽的锁
    }



    HalLedBlink( HAL_LED_2, 2, 60, (flashTime / 6) ); //三号灯闪烁,三次,亮度50%,时间间隔0.25s
    break;



/*************************************************************************************************************************
                                      收到 组播 消息
**************************************************************************************************************************/
  case SAMPLEAPP_FLASH_CLUSTERID:


    for(int i=0;i<NEI_NUM;++i)
    {
      if(ACTIVE[i]-- <= 0)
      {
        Reset_down_neighbor(Nei_Table[i]); //给掉线邻居发送设备重置信号
        Nei_Table[i] = Nei_Table[NEI_NUM-1];
        Nei_Table[NEI_NUM-1] = (int16)65534;
        ACTIVE[i] = ACTIVE[NEI_NUM-1];
        ACTIVE[NEI_NUM-1] = LIFE_TIME;
        LOST_nei++;
      }
    }

    NEI_NUM -= LOST_nei;  //邻居个数减少



    for(int i=0;i<3;++i)
    {
      //邻居表中有该路由
      if(Nei_Table[i]==neighbor_addr_10)
      {
        ACTIVE[i] = LIFE_TIME; //收到该邻居信息,更新存活时间
        break;
      }

      //其他合法路由
      else if((Nei_Table[i]!=(int16)65534) && (Nei_Table[i]!=neighbor_addr_10))
        continue;

      //添加路由
      else
      {
        Nei_Table[i]=neighbor_addr_10;
        NEI_NUM++;
        break;
      }
    }

    if(LOST_nei != 0)
    {
      //输出邻居的活跃点数
      HalUARTWrite(0,"\n------------\n",14);
      HalUARTWrite(0,"active: ",strlen("active: "));
      for (int j=0;j<NEI_NUM;++j)
      {
        active = '0' + ACTIVE[j];
        HalUARTWrite(0,&active,1);
        HalUARTWrite(0," ",1);
      }
      //输出邻居个数
      HalUARTWrite(0,"\n有",3);
      active = '0' + NEI_NUM;
      HalUARTWrite(0,&active,1);
      HalUARTWrite(0,"个邻居",6);
      
      HalUARTWrite(0,"\n------------\n",14);
    }


    //HalLedBlink( HAL_LED_2, 2, 50, (flashTime / 5) ); //二号灯闪烁,两次,亮度50%,时间间隔0.2s
    break;



/*******************************************
             收到 周期性 消息
********************************************/
  case SAMPLEAPP_PERIODIC_CLUSTERID:

    //NLME_UpdateLinkStatus(); //更新链路信息
    //HalLedBlink( HAL_LED_2, 2, 60, (flashTime / 6) ); //三号灯闪烁,三次,亮度50%,时间间隔0.25s
    break;
    /*收到 对方串口透传 消息*/
  case SAMPLEAPP_COM_CLUSTERID:
    HalUARTWrite(0,"对方说: ",strlen("对方说: "));
    len=pkt->cmd.Data[0];
    for(i=0;i<len;i++)
      HalUARTWrite(0,&pkt->cmd.Data[i+1],1);//发给 PC 机
    HalUARTWrite(0,"\n",1); // 回车换行
    break;

  }
}


//给掉线的邻居发送重置信号
void Reset_down_neighbor(int16 nei_addr)
{
  short flashTime = 1000;
  Point_To_Point_DstAddr.addr.shortAddr = nei_addr;
  unsigned char DATA[] = "!";
  if ( AF_DataRequest( &Point_To_Point_DstAddr, //点对点通信方式目标地址
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //点对点簇ID
                        1,
                        DATA,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      HalLedBlink( HAL_LED_3, 1, 60, (flashTime / 3) ); //led3闪
    }
    else
    {

    }
}


/***********************************************************<><><><><><><>*****
                   自定义 点对点 发送函数
************************************************************<><><><><><><>****/
void SampleApp_SendPointToPointMessage( void )
{
  uint16 flashTime = 1000;
  srand((int)osal_getClock()); //生成随机数种子
  int8 nei_index = random(NEI_NUM);  //挑选随机邻居节点
  Exchange_index = random(5)+1;//挑选随机交换码字

  Exchanging_Neighbor = Nei_Table[nei_index];  //即将交换码字的邻居
  Exchanging_Data = DATA_SLOT[Exchange_index].data; //即将交换的码字

  uint8 data = Exchanging_Data;  //赋值交换的随机数据
  uint8 degree_high = DATA_SLOT[Exchange_index].Degree_ID >>8; //高位码字度信息
  uint8 degree_low = (uint8)DATA_SLOT[Exchange_index].Degree_ID;  //低位码字度信息
  uint8 _DATA[] = {'*', data, degree_high, degree_low};  //发送数据交换请求 用'*' 表示请求类型
  /*给选出的邻居发送数据*/
  if((Nei_Table[nei_index] != (int16)65534) && (0<=nei_index) && (nei_index<NEI_NUM))
  {
    Point_To_Point_DstAddr.addr.shortAddr = Exchanging_Neighbor;
    if ( AF_DataRequest( &Point_To_Point_DstAddr, //点对点通信方式目标地址
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //点对点簇ID
                        4,
                        _DATA,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      HalLedBlink( HAL_LED_1, 1, 60, (flashTime / 3) ); //led1闪
      Waiting_Request = TRUE; //请求交换数据包标志位
    }
    else
    {
      HalUARTWrite(0,"交换数据包请求发送失败.",strlen("交换数据包请求发送失败."));
    }


    uchar INDEX = nei_index + '0';
    HalUARTWrite(0,"send to addr",strlen("send to addr"));
    HalUARTWrite(0,&INDEX,1);
    HalUARTWrite(0,"\n",1);
  }

}


/*********************************************************************
*********************************************************************/

