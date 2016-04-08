/**************************************************************************************************
   ZigBee协议栈由德州仪器提供给开发者使用
  本项目组保证遵守相关协议并仅用于学习
  项目组成员：李嘉文 姜越 陈含璐

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
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"
#include <string.h>
/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"



#define SYS_CLOCK_32MHZ  0
#define SYS_CLOCK_16MHZ  1

//波特率
#define UART_BAUD_2400   596
#define UART_BAUD_4800   597
#define UART_BAUD_9600   598
#define Port1 1
#define Bit0  0
#define Pn_m_OUT      2  //输出;
#define HAL_ADC_EOC         0x80    /* End of Conversion bit */
#define HAL_ADC_START       0x40    /* Starts Conversion */

#define HAL_ADC_STSEL_EXT   0x00    /* External Trigger */
#define HAL_ADC_STSEL_FULL  0x10    /* Full Speed, No Trigger */
#define HAL_ADC_STSEL_T1C0  0x20    /* Timer1, Channel 0 Compare Event Trigger */
#define HAL_ADC_STSEL_ST    0x30    /* ADCCON1.ST =1 Trigger */

#define HAL_ADC_RAND_NORM   0x00    /* Normal Operation */
#define HAL_ADC_RAND_LFSR   0x04    /* Clock LFSR */
#define HAL_ADC_RAND_SEED   0x08    /* Seed Modulator */
#define HAL_ADC_RAND_STOP   0x0c    /* Stop Random Generator */
#define HAL_ADC_RAND_BITS   0x0c    /* Bits [3:2] */

#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_256     0x20    /* Decimate by 256 : 12-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_DEC_BITS    0x30    /* Bits [5:4] */

#define HAL_ADC_STSEL       HAL_ADC_STSEL_ST
#define HAL_ADC_RAND_GEN    HAL_ADC_RAND_STOP
#define HAL_ADC_REF_VOLT    HAL_ADC_REF_AVDD
#define HAL_ADC_DEC_RATE    HAL_ADC_DEC_064
#define HAL_ADC_SCHN        HAL_ADC_CHN_VDD3
#define HAL_ADC_ECHN        HAL_ADC_CHN_GND
#define HAL_ADC_CHANNEL_6          0x06
/* Resolution */
#define HAL_ADC_RESOLUTION_8       0x01
#define HAL_ADC_RESOLUTION_10      0x02
#define HAL_ADC_RESOLUTION_12      0x03
#define HAL_ADC_RESOLUTION_14      0x04

/* Reference Voltages */
#define HAL_ADC_REF_125V          0x00    /* Internal Reference (1.25V-CC2430)(1.15V-CC2530) */
#define HAL_ADC_REF_AIN7          0x40    /* AIN7 Reference */
#define HAL_ADC_REF_AVDD          0x80    /* AVDD_SOC Pin Reference */
#define HAL_ADC_REF_DIFF          0xc0    /* AIN7,AIN6 Differential Reference */
#define HAL_ADC_REF_BITS          0xc0    /* Bits [7:6] */
#define HAL_ADC_CHN_BITS    0x0f    /* Bits [3:0] */
#define HAL_ADC_REF_AVDD          0x80    /* AVDD_SOC Pin Reference */
#define HAL_ADC_CHN_VDD3    0x0f    /* VDD/3 */
#define HAL_ADC_CHN_GND     0x0c    /* GND */

#define Pn_m_IN_UP    0  //上拉输入;
#define Pn_m_IN_DOWN  1  //下拉输入
#define LEFT_SHIFT(x) (1<<x)

//寄存器PxSEL配置；
#define PORTSEL_CONF(port , pin , val) { if(val == 0) P##port##SEL &= ~LEFT_SHIFT(pin);\
                                         else   P##port##SEL |= LEFT_SHIFT(pin);     }


//寄存器PxDIR配置；
#define PORTDIR_CONF(port , pin , val) { if(val == 0) P##port##DIR &= ~LEFT_SHIFT(pin);\
                                         else   P##port##DIR |= LEFT_SHIFT(pin);     }


//寄存器PxINP配置；
#define PORTINP_CONF(port , pin , val) { if(val == 0) P##port##INP &= ~LEFT_SHIFT(pin);\
                                         else   P##port##INP |= LEFT_SHIFT(pin);    }

/*********************************************************************
 * MACROS
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr)[0])
#define DATA_PIN P0_6            //定义P0.6口为传感器的输入端

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
  SAMPLEAPP_FLASH_CLUSTERID,
  SAMPLEAPP_P2P_CLUSTERID
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

afAddrType_t SampleApp_Periodic_DstAddr; //广播
afAddrType_t SampleApp_Flash_DstAddr;    //组播
afAddrType_t SampleApp_P2P_DstAddr;      //点播

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_Send_P2P_Message(void);
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
void uart_sendstring(unsigned char *str , unsigned char len);
void uart_sendstring(unsigned char *str , unsigned char len)
{
     unsigned char i;
     
     U0CSR &= ~0x40;  //禁止接收；
     
     for(i = 0; i < len; i++)
     {
         U0DBUF = *str++;  //将字符串一个一个字节写入缓冲区；
         while(UTX0IF == 0); //等待中断标志位置1表示发送完成；
         UTX0IF = 0; //重新置0；
     }
     
     U0CSR |= 0x40;  //允许接收；
}
uint16 alAdcRead (uint8 channel, uint8 resolution);
uint16 alAdcRead (uint8 channel, uint8 resolution)
{
  int16  reading = 0;

  uint8   i, resbits;
  uint8   adctemp;
  volatile  uint8 tmp;
  uint8  adcChannel = 1;
  uint8  reference;

  /* store the previously set reference voltage selection */
  reference = ADCCON3 & HAL_ADC_REF_BITS;

  /*
  * If Analog input channel is AIN0..AIN7, make sure corresponing P0 I/O pin is enabled.  The code
  * does NOT disable the pin at the end of this function.  I think it is better to leave the pin
  * enabled because the results will be more accurate.  Because of the inherent capacitance on the
  * pin, it takes time for the voltage on the pin to charge up to its steady-state level.  If
  * HalAdcRead() has to turn on the pin for every conversion, the results may show a lower voltage
  * than actuality because the pin did not have time to fully charge.
  */
  if (channel < 8)
  {
    for (i=0; i < channel; i++)
    {
      adcChannel <<= 1;
    }
  }

  /* Enable channel */
  ADCCFG |= adcChannel;

  /* Convert resolution to decimation rate */
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      resbits = HAL_ADC_DEC_064;
      break;
    case HAL_ADC_RESOLUTION_10:
      resbits = HAL_ADC_DEC_128;
      break;
    case HAL_ADC_RESOLUTION_12:
      resbits = HAL_ADC_DEC_256;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      resbits = HAL_ADC_DEC_512;
      break;
  }

  /* read ADCL,ADCH to clear EOC */
  tmp = ADCL;
  tmp = ADCH;

  /* Setup Sample */
  adctemp = ADCCON3;
  adctemp &= ~(HAL_ADC_CHN_BITS | HAL_ADC_DEC_BITS | HAL_ADC_REF_BITS);
  adctemp |= channel | resbits | (reference);

  /* writing to this register starts the extra conversion */
  ADCCON3 = adctemp;

  /* Wait for the conversion to be done */
  while (!(ADCCON1 & HAL_ADC_EOC));

  /* Disable channel after done conversion */
  ADCCFG &= (adcChannel ^ 0xFF);

  /* Read the result */
  reading = (int16) (ADCL);
  reading |= (int16) (ADCH << 8);

  /* Treat small negative as 0 */
  if (reading < 0)
    reading = 0;

  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      reading >>= 8;
      break;
    case HAL_ADC_RESOLUTION_10:
      reading >>= 6;
      break;
    case HAL_ADC_RESOLUTION_12:
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      reading >>= 2;
    break;
  }
  
  return ((uint16)reading);
}
unsigned char uint16_to_str(unsigned char *str , unsigned int n);
unsigned char uint16_to_str(unsigned char *str , unsigned int n)
{
      unsigned char len = 0;
                 
      if(n/10000 != 0)
      {
          str[0] = n/10000+0x30;
          str[1] = n%10000/1000+0x30;
          str[2] = n%1000/100 +0x30;
          str[3] = n%100/10 + 0x30;
          str[4] = n%10+0x30;
                 
          len = 5;
      }
      else if(n/1000 != 0)
      {
          str[0] = n/1000+0x30;
          str[1] = n%1000/100+0x30;
          str[2] = n%100/10 +0x30;
          str[3] = n%10 + 0x30;
              
          len = 4;
      }
      else if(n/100 != 0)
      {
          str[0] = n/100+0x30;
          str[1] = n%100/10+0x30;
          str[2] = n%10 +0x30;

          len = 3;
      }
      else if(n/10 != 0)
      {
          str[0] = n/10+0x30;
          str[1] = n%10+0x30;
          
          len = 2;
      }
      else
      {
          str[0] = n+0x30;
          
          len = 1;
      }
              
      return len;
}

void uart_init(unsigned char sys_clock , unsigned int baudrate);
void uart_init(unsigned char sys_clock , unsigned int baudrate)
{  
     P0SEL  |= 0x0c;  //配置P0.2和P0.3为外设，非GPIO
     U0CSR  |= 0x80;  //配置当前为UART，非SPI
    
     //#define UART_BAUD_9600   598
     //配置波特率；
     U0GCR  &= 0xf0;
     if(baudrate < 3000)
     {
        // U0BAUD = baudrate/10;  //59
        // baudrate %= 10; //8
       
        U0BAUD = 59;  //59
        baudrate =6; //8
       
         baudrate += sys_clock;  //32M:0   16:1
         U0GCR |= baudrate;  
     }
     else
     {
         U0BAUD = baudrate/100;
         baudrate %= 100;
         baudrate += sys_clock;
         U0GCR |= baudrate;
     }
     
     UTX0IF  = 0;     //清除中断标志
     U0CSR |=  0x40;  //允许接收数据
     IEN0  |=  0x04;  //打开接收中断   
     EA  = 1;  //打开总中断
}
void alAdcInit (void);
void alAdcInit (void)
{
  volatile uint8  tmp;

  ADCCON1 = HAL_ADC_STSEL | HAL_ADC_RAND_GEN | 0x03;
  ADCCON2 = HAL_ADC_REF_VOLT | HAL_ADC_DEC_RATE | HAL_ADC_SCHN;
  /*
  *  After reset, the first ADC reading of the extra conversion always reads GND level.
  *  We will do a few dummy conversions to bypass this bug.
  */
  tmp = ADCL;     /* read ADCL,ADCH to clear EOC */
  tmp = ADCH;
  ADCCON3 = HAL_ADC_REF_VOLT | HAL_ADC_DEC_RATE | HAL_ADC_ECHN;
  while ((ADCCON1 & HAL_ADC_EOC) != HAL_ADC_EOC);   /* Wait for conversion */
  tmp = ADCL;     /* read ADCL,ADCH to clear EOC */
  tmp = ADCH;
  ADCCON3 = HAL_ADC_REF_VOLT | HAL_ADC_DEC_RATE | HAL_ADC_ECHN;
  while ((ADCCON1 & HAL_ADC_EOC) != HAL_ADC_EOC);   /* Wait for conversion */
  tmp = ADCL;     /* read ADCL,ADCH to clear EOC */
  tmp = ADCH;
}

//涉及到的寄存器进行配置；
#define REGISTER_CONF(port , pin , mode) {\
      if(mode == Pn_m_IN_UP || mode == Pn_m_IN_DOWN)\
      {\
          PORTDIR_CONF(port , pin , 0);\
          PORTSEL_CONF(port , pin , 0);\
          PORTINP_CONF(port , pin , 0);\
          PORTINP_CONF(2 , (5+port) ,mode);\
      }\
      else if(mode == Pn_m_OUT)\
      {\
          PORTDIR_CONF(port , pin , 1);\
          PORTSEL_CONF(port , pin , 0);\
      }\
}

void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
    CLKCONCMD &= ~0x40;  //系统时钟源为32MHZ晶振；
    while(CLKCONSTA & 0x40); //等待时钟稳定；
    CLKCONCMD &= ~0x47; //系统主时钟频率：32MHZ;
  uart_init(SYS_CLOCK_32MHZ , UART_BAUD_9600);
  REGISTER_CONF(Port1,Bit0,Pn_m_OUT);
  alAdcInit();
 // P0SEL &= ~0x40;                 //设置P0.6为普通IO口
 // P0DIR &= ~0x40;                 //P0.6定义为输入口
  
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
  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播 
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器

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
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
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

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
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
          if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                 (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
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
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    //SampleApp_SendPeriodicMessage();
    SampleApp_Send_P2P_Message();

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

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
  uint16 flashTime;

  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_P2P_CLUSTERID:
     // HalUARTWrite(0, "02:", 3);       //提示接收到数据
      HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
      HalUARTWrite(0, "\n", 1);         // 回车换行
      break;    
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
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
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
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
 * @fn      SampleApp_Send_P2P_Message
 *
 * @brief   point to point.
 *
 * @param   none
 *
 * @return  none
 */
void delay_us(unsigned int n);
void delay_us(unsigned int n)
{
    n>>=1;
    while(n--)
    {
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
          asm("NOP");
    }
}



/***********************************************
*  函数名称： delay_ms
*  功能    ： ms延时函数，系统默认时钟为16MHZ
*  参数列表： n_ms 为延时的ms数，最大不超过65535 
*  返回值  ： 无
.************************************************/
void delay_ms(unsigned int n_ms);
void delay_ms(unsigned int n_ms)
{
    unsigned int i,j;
    
    for (i = 0; i < n_ms; i++)
    {
        for (j = 0; j < 1070; j++);
    }
}

unsigned int sennor_getpmval(void);
unsigned int sennor_getpmval(void)
{
    unsigned int pmval = 0;
      
    //sensor_led = 1;  
    delay_us(280); //280us; 
    
    pmval = alAdcRead(HAL_ADC_CHANNEL_6,HAL_ADC_RESOLUTION_12);
    delay_us(40); //40us;
    
    //sensor_led = 0;
    delay_us(9680);   //9680us;
 
    return pmval;
}

void SampleApp_Send_P2P_Message( void )
{

    unsigned int pmval = 0;
    unsigned char pmval_s[5] = {'\0'}; //长度不小于5
    unsigned char pmval_s_len = 0; //转换成字符串后有效长度
    unsigned char pmval_send[8] = {'0','1',':'}; //长度不小于5
             
    pmval = sennor_getpmval();
         
         if(pmval > 500)
         {
         //    pmval <<= 2;l
         //    pmval -= 150;
              
             pmval/=38;
             pmval_s_len = uint16_to_str(pmval_s , pmval);
              
             //获取成功后打印到串口;
             uart_sendstring(pmval_s , pmval_s_len);
             uart_sendstring("\r\n" ,2);
         
         strcat(pmval_send,pmval_s); 
       //  delay_ms(1000);

    

   
  if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       pmval_s_len+3,
                       pmval_send,
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
/*********************************************************************
*********************************************************************/

