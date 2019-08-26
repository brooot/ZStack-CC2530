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



//�û������ͷ�ļ�
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
  MT_UartInit(); //��ʼ��UART
  MT_UartRegisterTaskID(task_id);// �Ǽ������
  HalUARTWrite(0, "Hello World\n", 12); // ������ 0 ��'�ַ���,�ַ�������







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
  Point_To_Point_DstAddr.addr.shortAddr = 0x0000; //����Э����






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
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *          ******************Ӧ�ô������****************************
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
          HalUARTWrite(0, "KEY ", 4);
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
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )  //�����Է��ͺ���
  {
    /*Send the periodic message*/
    SampleApp_SendPeriodicMessage(); //�û������Թ㲥���ͺ���

    /*Point_to_Point*/
    //SampleApp_SendPointToPointMessage(); //��Ե㷢�ͺ���

    /*Group*/
    SampleApp_SendGroupMessage(); //�鲥ͨѶ����


    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
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
//    /*����Ϲд��*/
//    if( *(str) == 4)
//    {
//      uint8 data[10]={'0','1','2','3','4','5','6','7','8','9'};
//
//      //HalUARTWrite(0,"get an 'a' !",strlen("get an 'a' !") );
//      if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
//                          SAMPLEAPP_COM_CLUSTERID,//�Լ�����һ��
//                          10, // ���ݳ���
//                          data, //��������
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
//    /*����Ϲд��*/


    /*******���ͳ�ȥ***�ο����� 1 Сʱ�������ݴ���̳�*********/
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                        SAMPLEAPP_COM_CLUSTERID,//�Լ�����һ��
                        len+1, // ���ݳ���
                        str, //��������
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
                   �Զ����Ե㷢�ͺ���
**********************************************************/
void SampleApp_SendPointToPointMessage( void )
{
  uint8 data[11]={'f','r','o','m',' ','r','o','u','t','e','1'};
  if ( AF_DataRequest( &Point_To_Point_DstAddr, //��Ե�ͨ�ŷ�ʽĿ���ַ
                      &SampleApp_epDesc,
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID, //��Ե��ID
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
                    �鲥 ���ͺ���
*********************************************************/
void SampleApp_SendGroupMessage( void )
{
  uint8 data[10]={'G','R','O','U','P',' ','2',' ','r','1'};// �Զ�// ������
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
                    �����Թ㲥 ���ͺ���
 **********************************************************/
void SampleApp_SendPeriodicMessage( void )
{
  uint8 data[10]={'f','r','o','m',' ','b','r','o',' ','4'};
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       10,    //���ݳ���
                       data,  //ָ�뷽ʽ
//                       1,  //���ݳ���
//                       (uint8*)&SampleAppPeriodicCounter, //Ҫ���͵�����
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

            ****************���պ���*********************
 *          ������յ�������,�����Ǵ������豸�յ�������.
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime=1000;//һ��,1000ms
  uint8 i,len; //͸��
  switch ( pkt->clusterId )
  {

    /*�յ� ��Ե� ��Ϣ*/
  case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
    HalUARTWrite(0, "�յ���Ե���Ϣ: ", strlen("�յ���Ե���Ϣ: "));  //�����ڷ�����ʾ
    HalUARTWrite(0, &pkt->cmd.Data[0], 11); // ��ӡ�յ�����
    HalUARTWrite(0, "\n", 1); // �س�����
    break;

    /*�յ� ������ ��Ϣ*/
  case SAMPLEAPP_PERIODIC_CLUSTERID:
                HalUARTWrite(0, "��������Ϣ: ", strlen("��������Ϣ: "));  //�����ڷ�����ʾ
                HalUARTWrite(0, &pkt->cmd.Data[0], 10); // ��ӡ�յ�����
                HalUARTWrite(0, "\n", 1); // �س�����
                HalLedBlink( HAL_LED_3, 3, 50, (flashTime / 4) ); //���ŵ���˸,����,����50%,ʱ����0.25s
    break;

//    /*�鲥��Ϣ*/
//  case SAMPLEAPP_FLASH_CLUSTERID:
//    flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
//    HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
//    break;

    /*�յ� �Է�����͸�� ��Ϣ*/
  case SAMPLEAPP_COM_CLUSTERID:
    HalUARTWrite(0,"�Է�˵: ",strlen("�Է�˵: "));
    len=pkt->cmd.Data[0];
    for(i=0;i<len;i++)
      HalUARTWrite(0,&pkt->cmd.Data[i+1],1);//���� PC ��
    HalUARTWrite(0,"\n",1); // �س�����
    break;

    /*�յ� �鲥 ��Ϣ*/
  case SAMPLEAPP_FLASH_CLUSTERID:
    HalUARTWrite(0,"�鲥��Ϣ:",strlen("�鲥��Ϣ:"));// ������ʾ������
    HalUARTWrite(0, &pkt->cmd.Data[0],10); // ��ӡ�յ�����
    HalUARTWrite(0,"\n",1); //
    HalLedBlink( HAL_LED_2, 2, 50, (flashTime / 5) ); //���ŵ���˸,����,����50%,ʱ����0.2s
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
  //�жϰ���S2���£��������ݵ�����
  if ( keys & HAL_KEY_SW_6 )
  {
    HalUARTWrite(0,"S1\n",3);
    HalLedBlink( HAL_LED_3, 2,50, 500 ); //LED1 ��˸��ʾ
  }
}






/*********************************************************************
*********************************************************************/
