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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "MT.h"  //͸����CMD_SERIAL_MSG�Ķ���

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"
#include "string.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

extern uint16 NLME_GetShortAddr( void ); //��ȡ����̵�ַ
extern void NLME_UpdateLinkStatus( void ); //��������״̬



/*********** ע��: ÿ���豸��һ��,�ǵ��޸�!  **********/
uint8 SOURCE_DATA = '3';  //�ɼ�������
uint8 Device_ID = 3;



/*********************************************************************
 * MACROS
 */
#define random(x) (rand() % x)
#define uchar unsigned char
#define Node_Num uint8 3  //�����нڵ������
/*********************************************************************
 * CONSTANTS
 */
//�û������ͷ�ļ�
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
 * GLOBAL VARIABLES ȫ�ֱ���
 */
int8 LIFE_TIME=5;               //�ھӻ�Ծ�ȳ�ʼֵ
int8 NEI_NUM = 0;               //�ھ�����
int16 Nei_Table[3] = {0};       //�ھӱ�
unsigned short ACTIVE[3] = {0}; //�ھӻ�Ծ���б�
int8 _TURN = 0;                 //�����ִο���


/*#######################������Sink�ڵ������############################*/

/*һ�����ֵ����*/
typedef struct Encoded_PKT  //�����������
{
    /*������*/
    short data; //��������
    /*������Ϣ,���ļ����ڵ�����ݰ����*/
    uint16 Degree_ID;
}PKT;


const int Slot_Size = 6;        //���ֲ۴�С

PKT DATA_SLOT[5+1];             //���ֽṹ������


//uint8 DATA_SLOT[5+1] = {0};     //���ֲ�
//uint8 DATA_Degree[5+1] = {0};   //���ֶ�
unsigned short MAX_Degree = 1;  //��ʼ�����������
uint8 Decoded_X[100] = {0};     //��ʼ���ѽ������������
uint8 Undecoded_Y[100] = {0};   //��ʼ��δ����ĸ�����������
int16 Exchanging_Neighbor;      //���������ֽ������ھ�
uint8 Exchanging_Data;          //������������
uint8 Exchange_index;           //�����������ֵ�����

uint8 Waiting_Request = FALSE;   //�Ƿ��ڵȴ��Է����ؽ��������ݰ���Ӧ
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
afAddrType_t SampleApp_Flash_DstAddr; //�鲥

afAddrType_t Point_To_Point_DstAddr; //WeBee��Ե�ͨ�Ŷ���

aps_Group_t SampleApp_Group; //��������

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS     ��������
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg); // ����ͨ��
void SampleApp_SendPointToPointMessage( void ); //��Ե�
void SampleApp_SendGroupMessage( void );// �����鲥ͨѶ���ͺ�������
void Reset_down_neighbor(int16 nei_addr); //�����ھ��豸����
void Show_Bit(uint8 X, uint8 N); //���ڴ�ӡһ��8λ���ض���������
void Show_Num8(uint8 X);  //���ڴ�ӡ8bit����
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
 * @fn      SampleApp_Init   Ӧ�ó�ʼ��
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
  MT_UartInit(); //��ʼ��UART
  MT_UartRegisterTaskID(task_id);// �Ǽ������
  HalUARTWrite(0, "System is started.\n\n", 20); // ������ 0 ��'�ַ���,�ַ�������


  //·�ɱ��ʼ��
  for(short i=0;i<3;++i)
  {
    Nei_Table[i] += 65534;
    ACTIVE[i] += LIFE_TIME;  //�ھӻ�Ծ�ȳ�ʼ��
  }


  //�����ֲ۸���ֵ(����sink�ڵ�)
  for(short i=0;i<6;++i)
  {
    DATA_SLOT[i].data = SOURCE_DATA;  //�����ֲ۸���ֵ
    DATA_SLOT[i].Degree_ID = Power_of_2(i);       //�����ָ��ڵ���Ϣ
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



  //WeBee��Ե�ͨѶ����
  Point_To_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //�㲥ģʽ
  Point_To_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  Point_To_Point_DstAddr.addr.shortAddr = 0xB6AE; //����Э����






  // Setup for the periodic message's destination address
  // Broadcast to everyone     �㲥��ʽ����
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;//���͸������豸,˯���еĽڵ��ɸ��ڵ��ݴ�ֱ����ʱ
  //0xFFFD ���͸���˯��״̬�������豸
  //0xFFFC ���͸�����·�����Լ�Э����




  //�鲥Group1����
  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;  //endpoint �˵�
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



  /***********���� Group_ID*************/
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
 *        ******************Ӧ�ô������***************************
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

  if ( events & SYS_EVENT_MSG ) //ϵͳ��Ϣ�����¼�
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case CMD_SERIAL_MSG:    // �����յ����ݺ���MT_UART�㴫�ݹ������ݣ������䷽�����գ�����ʱ������MT�������
            SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
            break;

        // Received when a key is pressed
        case KEY_CHANGE: //�����¼���������
          //HalUARTWrite(0, "KEY ", 4);
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        // ���մ�����
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:  //����״̬�ı䣬��δ����������
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD) //Э����
              || (SampleApp_NwkState == DEV_ROUTER) //·����
              || (SampleApp_NwkState == DEV_END_DEVICE) ) //�ն� ��ִ��
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
//              HalUARTWrite(0,"����Ͽ�",8);
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
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )  //*****�����Է��ͺ���*****
  {


/***********************************�����Է��ͺ���****************************************/

    /*Send the periodic message*/
    //SampleApp_SendPeriodicMessage(); //�û������Թ㲥���ͺ���

    if(_TURN == 0){
      /*Group*/
      SampleApp_SendGroupMessage(); //�鲥ͨѶ����
      _TURN = 1;
    }

    else{
      /*Point_to_Point*/
      SampleApp_SendPointToPointMessage(); //��Ե㷢�ͺ���
      _TURN = 0;
    }

/**********************************�����Է��ͺ���*****************************************/




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
                    �鲥 ���ͺ���
*********************************************************/
void SampleApp_SendGroupMessage( void )
{
  //uint8 data[10]={'G','R','O','U','P',' ','1',' ','r','4'};// �Զ�������
  if ( AF_DataRequest( & SampleApp_Flash_DstAddr,
                      &SampleApp_epDesc,
                      SAMPLEAPP_FLASH_CLUSTERID,
                      1,
                      (uint8*)&SampleAppPeriodicCounter, //Ҫ���͵�����
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
                    �����Թ㲥 ���ͺ���
 **********************************************************/
void SampleApp_SendPeriodicMessage( void )
{
  unsigned int flashTime=1000;
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,  //���ݳ���
                       (uint8*)&SampleAppPeriodicCounter, //Ҫ���͵�����
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
 * @fn      SampleApp_HandleKeys ����������
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
  //�жϰ���S2���£��������ݵ�����
  if ( keys & HAL_KEY_SW_6 )
  {
    int self_addr_10 = NLME_GetShortAddr();
    int neig_addr_10;
    uchar Self_ADDR[4]; //16���Ƶ�ַ��ʽ
    uchar Neig_ADDR[4];
    uchar Num[2];

    //������������ַ
//    HalUARTWrite(0,"�����ַΪ: 0x",strlen("�����ַΪ: 0x"));
//    if(self_addr_10 == 0)
//    {
//      HalUARTWrite(0,"0000",4);
//      HalUARTWrite(0,"\n",1);
//    }
//    else
//    {
//      sprintf((char*)Self_ADDR, "%X", self_addr_10); //10��������ת����16�����ַ���
//
//      for (int i=0;i<4;++i)
//      {
//        HalUARTWrite(0,&Self_ADDR[i],1);
//      }
//      HalUARTWrite(0,"\n",1);
//    }

    //�������Դ����
    HalUARTWrite(0,"Դ������: ",strlen("Դ������: "));
    Show_Bit(SOURCE_DATA,8);
    for(short p=1;p<6;++p)
    {
      Show_Bit(DATA_SLOT[p].data, 8);
      HalUARTWrite(0,"������Ϣ: ",strlen("������Ϣ: "));
      Show_Bit(DATA_SLOT[p].Degree_ID, 16);
    }
    
    HalUARTWrite(0,"�ھӸ���:",5);
    uint8 nei_N = 0;
    for(int num_nei=0;num_nei<3;++num_nei)
    {
      if(Nei_Table[num_nei] != (uint16)65534)
        nei_N++;
    }
    Show_Bit(nei_N, 8);
    //������������ھӵ�ַ
//    for(int i=0;i<3;++i)
//    {
//      neig_addr_10 = Nei_Table[i];
//
//      sprintf((char*)Num, "%d", i+1);
//      sprintf((char*)Neig_ADDR, "%X", neig_addr_10);
//      HalUARTWrite(0,"�ھ�",strlen("�ھ�"));
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

//    HalLedBlink( HAL_LED_3, 2,50, 500 ); //LED3 ��˸��ʾ
  }
}





/*���ж���� �����ڷ������ݰ� �ĺ���*/
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)
{
    uint8 i,len,*str=NULL; //len �������ݳ���
    str=cmdMsg->msg; //ָ�����ݿ�ͷ
    len=*str; //msg ��ĵ� 1 ���ֽڴ����������ݳ���
    /********��ӡ�����ڽ��յ������ݣ�������ʾ*********/
    HalUARTWrite(0,"��˵: ",strlen("��˵: "));
    for(i=1;i<=len;i++)
    {
      HalUARTWrite(0,str+i,1);
    }

    HalUARTWrite(0,"\n",1 );//����


    //ģ��������
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


//    /*******���ͳ�ȥ***�ο����� 1 Сʱ�������ݴ���̳�*********/
//    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
//                        SAMPLEAPP_COM_CLUSTERID,//�Լ�����һ��
//                        len+1, // ���ݳ���
//                        str, //��������
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
 * @fn      SampleApp_MessageMSGCB  ���պ���
 *
 *          ������յ�������,�����Ǵ������豸�յ�������.
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
  uint16 flashTime=1000;//һ��,1000ms
  uint8 i,len; //͸��
  unsigned short neighbor_addr_10 = pkt->srcAddr.addr.shortAddr;  //���ͷ������ֵ�ַ
  unsigned char ADDR[4]; //���ֵ�ַת������ַ���ַ
  int8 LOST_nei = 0;
  uchar active;
  
  uchar BUSY[4] = "####";
  uint8 Peer_data; //�Է�����
  int16 Peer_info; //�Է�������Ϣ
  uint8 Self_data; //��������
  int16 Self_info; //����������Ϣ
  int8 degree_low; //������Ϣ��λ
  int8 degree_high;//������Ϣ��λ
  switch ( pkt->clusterId )
  {

    /*************************�յ� ��Ե� ��Ϣ********************************/
  case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
    //�յ������ź�
    if(pkt->cmd.Data[0] == '!')
    {
      HalUARTWrite(0,"������...",strlen("������..."));
      SystemResetSoft();
    }

    HalUARTWrite(0, "�յ���Ե���Ϣ: ", strlen("�յ���Ե���Ϣ: "));  //�����ڷ�����ʾ
    HalUARTWrite(0, &pkt->cmd.Data[0], 1); // ��ӡ�յ�����
    Show_Bit(pkt->cmd.Data[1],8);

    sprintf((char*)ADDR, "%X", neighbor_addr_10);
    HalUARTWrite(0," from ",6);
    HalUARTWrite(0,ADDR,4);
    HalUARTWrite(0, "\n\n", 2); // �س�����

    /*����������Ѿ����𽻻�����������,�յ��Է������Ľ������ݰ���Ӧ*/
    if(Waiting_Request && (neighbor_addr_10 == Exchanging_Neighbor)) 
    {
      /*�Է���æ*/
      if(pkt->cmd.Data[0] == '#' && pkt->cmd.Data[1] == '#' && pkt->cmd.Data[2] == '#' && pkt->cmd.Data[3] == '#')
      {
        Waiting_Request = FALSE;
        HalUARTWrite(0,"peer is busy",12);
        HalUARTWrite(0,"\n",1);
//        SampleApp_SendPointToPointMessage(); //�ٴη��𽻻�����
      }
      /*�Է�������Ӧ*/
      if(!Exchange_Lock && (pkt->cmd.Data[0] == '$'))
      {
        Exchange_Lock = TRUE; //�������ֲ۵���
        
        Peer_data = pkt->cmd.Data[1]; //�Է�����������
        Peer_info = pkt->cmd.Data[3]; 
        Peer_info |= pkt->cmd.Data[2]<<8; //�Է�������Ϣ
        //�鿴���������ݰ��Ķ�
        uint8 Peer_degree = 0;
        uint8 Self_included = FALSE;
        for(short k=0;k<16;++k)
        {
          if(1 & Peer_info>>k)
          {
            ++Peer_degree;
            if(k == Device_ID)
            {
              Self_included = TRUE; //���ְ�������Ľڵ����ݰ�
              HalUARTWrite(0,"����������Ϣ",12);
            }
            
          }
        }
        //�ж��Ƿ���ϱ�������,��������������ݰ�,�ҶȲ������������Ķ�
        if(!Self_included && (Peer_degree < MAX_Degree))
        {
          Peer_data ^= DATA_SLOT[0].data; //������
          Peer_info &= Device_ID; //������Ϣ�����������豸ID
          HalUARTWrite(0,"�ѱ���\n������ֵ��:",strlen("�ѱ���\n������ֵ��:"));
          Show_Bit(Peer_data, 8);
        }
        else
        {
          HalUARTWrite(0,"δ����\n�Է���ֵ��:",strlen("δ����\n�Է���ֵ��:"));
          Show_Bit(Peer_data, 8);
        }
        Show_Num8(Peer_degree);
        DATA_SLOT[Exchange_index].data = Peer_data; //��Ӧλ���滻Ϊ�Է�������
        DATA_SLOT[Exchange_index].Degree_ID = Peer_info; //��¼������Ϣ
        
        
        Exchange_Lock = FALSE; //�ر����ֲ۵���
      }
      
      
      
      
      
    }

    /*�ڵȴ��Է����ؽ������ݰ��Ĺ������յ������ڵ�Ľ�������*/
    else if(Waiting_Request && (neighbor_addr_10 != Exchanging_Neighbor) && (pkt->cmd.Data[0] == '*')) 
    {
      /*������æ�ź� "####" */ 
      Point_To_Point_DstAddr.addr.shortAddr = neighbor_addr_10;
      if(AF_DataRequest( &Point_To_Point_DstAddr, //��Ե�ͨ�ŷ�ʽĿ���ַ
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //��Ե��ID
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

    /*����յ��Է������Ľ������ݰ�����*/
    else if(!Exchange_Lock)
    {
      Exchange_Lock = TRUE; //�������ֲ۵���
      /*��ѡ������������ݰ����͸��Է�*/
      srand((int)osal_getClock());
      Exchange_index = random(5)+1;//��ѡ�����������
      Self_data = DATA_SLOT[Exchange_index].data; //��������������
      Self_info = DATA_SLOT[Exchange_index].Degree_ID; //������Ϣ
      degree_high = (uint8)Self_info>>8;
      degree_low = (uint8)Self_info;
      //����
      uint8 _DATA[4] = {'$', Self_data, degree_high, degree_low};
      Point_To_Point_DstAddr.addr.shortAddr = neighbor_addr_10;
      if(AF_DataRequest( &Point_To_Point_DstAddr, //��Ե�ͨ�ŷ�ʽĿ���ַ
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //��Ե��ID
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
      
      /*���Է����������ݰ������������ѡ�����ݰ����� <���ж��Ƿ���Ҫ����> */
      Peer_data = pkt->cmd.Data[1]; //�Է�����������
      Peer_info = pkt->cmd.Data[3]; 
      Peer_info |= pkt->cmd.Data[2]<<8; //�Է�������Ϣ
      //�鿴���������ݰ��Ķ�
      uint8 Peer_degree = 0;
      uint8 Self_included = FALSE;
      for(short k=0;k<16;++k)
      {
        if(1<<k & Peer_info)
        {
          ++Peer_degree;
          if(k == Device_ID)
            Self_included = TRUE; //���ְ�������Ľڵ����ݰ�
          
        }
      }
      //�ж��Ƿ���ϱ�������,��������������ݰ�,�ҶȲ������������Ķ�
      if(!Self_included && (Peer_degree < MAX_Degree))
      {
        
        Peer_data ^= DATA_SLOT[0].data; //������
        Peer_info &= Device_ID; //������Ϣ�����������豸ID
        HalUARTWrite(0,"�ѱ���\n������ֵ��:",strlen("�ѱ���\n������ֵ��:"));
        Show_Bit(Peer_data, 8);
      }
      else
      {
        HalUARTWrite(0,"δ����\n�Է���ֵ��:",strlen("δ����\n�Է���ֵ��:"));
        Show_Bit(Peer_data, 8);
      }
      DATA_SLOT[Exchange_index].data = Peer_data; //��Ӧλ���滻Ϊ�Է�������
      DATA_SLOT[Exchange_index].Degree_ID = Peer_info; //��¼������Ϣ
      
      Exchange_Lock = FALSE; //�ر����ֲ۵���
    }



    HalLedBlink( HAL_LED_2, 2, 60, (flashTime / 6) ); //���ŵ���˸,����,����50%,ʱ����0.25s
    break;



/*************************************************************************************************************************
                                      �յ� �鲥 ��Ϣ
**************************************************************************************************************************/
  case SAMPLEAPP_FLASH_CLUSTERID:


    for(int i=0;i<NEI_NUM;++i)
    {
      if(ACTIVE[i]-- <= 0)
      {
        Reset_down_neighbor(Nei_Table[i]); //�������ھӷ����豸�����ź�
        Nei_Table[i] = Nei_Table[NEI_NUM-1];
        Nei_Table[NEI_NUM-1] = (int16)65534;
        ACTIVE[i] = ACTIVE[NEI_NUM-1];
        ACTIVE[NEI_NUM-1] = LIFE_TIME;
        LOST_nei++;
      }
    }

    NEI_NUM -= LOST_nei;  //�ھӸ�������



    for(int i=0;i<3;++i)
    {
      //�ھӱ����и�·��
      if(Nei_Table[i]==neighbor_addr_10)
      {
        ACTIVE[i] = LIFE_TIME; //�յ����ھ���Ϣ,���´��ʱ��
        break;
      }

      //�����Ϸ�·��
      else if((Nei_Table[i]!=(int16)65534) && (Nei_Table[i]!=neighbor_addr_10))
        continue;

      //���·��
      else
      {
        Nei_Table[i]=neighbor_addr_10;
        NEI_NUM++;
        break;
      }
    }

    if(LOST_nei != 0)
    {
      //����ھӵĻ�Ծ����
      HalUARTWrite(0,"\n------------\n",14);
      HalUARTWrite(0,"active: ",strlen("active: "));
      for (int j=0;j<NEI_NUM;++j)
      {
        active = '0' + ACTIVE[j];
        HalUARTWrite(0,&active,1);
        HalUARTWrite(0," ",1);
      }
      //����ھӸ���
      HalUARTWrite(0,"\n��",3);
      active = '0' + NEI_NUM;
      HalUARTWrite(0,&active,1);
      HalUARTWrite(0,"���ھ�",6);
      
      HalUARTWrite(0,"\n------------\n",14);
    }


    //HalLedBlink( HAL_LED_2, 2, 50, (flashTime / 5) ); //���ŵ���˸,����,����50%,ʱ����0.2s
    break;



/*******************************************
             �յ� ������ ��Ϣ
********************************************/
  case SAMPLEAPP_PERIODIC_CLUSTERID:

    //NLME_UpdateLinkStatus(); //������·��Ϣ
    //HalLedBlink( HAL_LED_2, 2, 60, (flashTime / 6) ); //���ŵ���˸,����,����50%,ʱ����0.25s
    break;
    /*�յ� �Է�����͸�� ��Ϣ*/
  case SAMPLEAPP_COM_CLUSTERID:
    HalUARTWrite(0,"�Է�˵: ",strlen("�Է�˵: "));
    len=pkt->cmd.Data[0];
    for(i=0;i<len;i++)
      HalUARTWrite(0,&pkt->cmd.Data[i+1],1);//���� PC ��
    HalUARTWrite(0,"\n",1); // �س�����
    break;

  }
}


//�����ߵ��ھӷ��������ź�
void Reset_down_neighbor(int16 nei_addr)
{
  short flashTime = 1000;
  Point_To_Point_DstAddr.addr.shortAddr = nei_addr;
  unsigned char DATA[] = "!";
  if ( AF_DataRequest( &Point_To_Point_DstAddr, //��Ե�ͨ�ŷ�ʽĿ���ַ
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //��Ե��ID
                        1,
                        DATA,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      HalLedBlink( HAL_LED_3, 1, 60, (flashTime / 3) ); //led3��
    }
    else
    {

    }
}


/***********************************************************<><><><><><><>*****
                   �Զ��� ��Ե� ���ͺ���
************************************************************<><><><><><><>****/
void SampleApp_SendPointToPointMessage( void )
{
  uint16 flashTime = 1000;
  srand((int)osal_getClock()); //�������������
  int8 nei_index = random(NEI_NUM);  //��ѡ����ھӽڵ�
  Exchange_index = random(5)+1;//��ѡ�����������

  Exchanging_Neighbor = Nei_Table[nei_index];  //�����������ֵ��ھ�
  Exchanging_Data = DATA_SLOT[Exchange_index].data; //��������������

  uint8 data = Exchanging_Data;  //��ֵ�������������
  uint8 degree_high = DATA_SLOT[Exchange_index].Degree_ID >>8; //��λ���ֶ���Ϣ
  uint8 degree_low = (uint8)DATA_SLOT[Exchange_index].Degree_ID;  //��λ���ֶ���Ϣ
  uint8 _DATA[] = {'*', data, degree_high, degree_low};  //�������ݽ������� ��'*' ��ʾ��������
  /*��ѡ�����ھӷ�������*/
  if((Nei_Table[nei_index] != (int16)65534) && (0<=nei_index) && (nei_index<NEI_NUM))
  {
    Point_To_Point_DstAddr.addr.shortAddr = Exchanging_Neighbor;
    if ( AF_DataRequest( &Point_To_Point_DstAddr, //��Ե�ͨ�ŷ�ʽĿ���ַ
                        &SampleApp_epDesc,
                        SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //��Ե��ID
                        4,
                        _DATA,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      HalLedBlink( HAL_LED_1, 1, 60, (flashTime / 3) ); //led1��
      Waiting_Request = TRUE; //���󽻻����ݰ���־λ
    }
    else
    {
      HalUARTWrite(0,"�������ݰ�������ʧ��.",strlen("�������ݰ�������ʧ��."));
    }


    uchar INDEX = nei_index + '0';
    HalUARTWrite(0,"send to addr",strlen("send to addr"));
    HalUARTWrite(0,&INDEX,1);
    HalUARTWrite(0,"\n",1);
  }

}


/*********************************************************************
*********************************************************************/

