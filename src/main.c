/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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

#define ONEWIRE_SEARCH 1
#define ONEWIRE_CRC 0

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "integer.h"
#include "string.h"
#include <stdio.h>
#include "ST7565.h"
#include "OneWire.h"
#include "math.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

//typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/   
/* Private define ------------------------------------------------------------*/
#define a1 2.508355e-2
#define a2 7.860106e-8
#define a3 -2.503131e-10
#define a4 8.315270e-14
#define a5 -1.228034e-17
#define a6 9.804036e-22
#define a7 -4.413030e-26
#define a8 1.057734e-30
#define a9 -1.052755e-35

/* Private macro -------------------------------------------------------------*/   
/* Private variables ---------------------------------------------------------*/   
static __IO uint32_t TimingDelay;
static volatile uint32_t msCounter;

SPI_InitTypeDef  SPI_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;
OWire OneWire;
unsigned char addr[8] = {0x28, 0xEC, 0x21, 0xEA, 0x03, 0x00, 0x00, 0xC9};
int edgeCount;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
//const unsigned int PowerValues[17] = {0,12021, 15462, 18037, 20220, 22183, 24017, 25776, 27500, 29224, 30983, 32817, 34780, 36963, 39538, 42979, 55000};
const unsigned int PowerValues[100] = {0 ,6401 ,8107 ,9327 ,10305 ,11147 ,11891 ,12563 ,13179 ,13759 ,14293 ,14801 ,15287 ,15749 ,16191 ,16621 ,17035 ,17435 ,17825 ,18203 ,18571 ,18931 ,19285 ,19633 ,19969 ,20305 ,20633 ,20953 ,21275 ,21587 ,21897 ,22203 ,22503 ,22807 ,23105 ,23399 ,23691 ,23983 ,24271 ,24557 ,24843 ,25125 ,25407 ,25689 ,25971 ,26249 ,26527 ,26805 ,27085 ,27359 ,27641 ,27919 ,28195 ,28473 ,28753 ,29035 ,29313 ,29593 ,29877 ,30157 ,30445 ,30733 ,31017 ,31309 ,31601 ,31899 ,32195 ,32497 ,32797 ,33105 ,33413 ,33725 ,34047 ,34367 ,34695 ,35031 ,35373 ,35715 ,36069 ,36429 ,36797 ,37179 ,37565 ,37965 ,38379 ,38809 ,39251 ,39715 ,40199 ,40707 ,41241 ,41821 ,42437 ,43109 ,43853 ,44695 ,45673 ,46887 ,48593, 55000};
/* Private function prototypes -----------------------------------------------*/   
void GPIO_Setup(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void InitADC1(void);
unsigned int GetADC1Channel(unsigned char chanel);
void DelayuS(vu32 nCount);	 			// 1uS Delay
void DelaymS(vu32 nTime);	 			// 1uS Delay
float Dac2Dt(float DacValue);
unsigned long millis(void);
void zeroCrossDetect(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Cable_Config (FunctionalState NewState);
void Get_SerialNum(void);


/** @addtogroup Template_Project
  * @{
  */




/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  char test[25];
  unsigned int an1, an2, an3, an4, an5;
  float Ta=0;
  int i;
  long msum[4];
  float fm[4], T1, T2, T3, SP;
  unsigned int DSState = 0;
  unsigned long DSTimer=0, t0, t1, dt, HTimer, TTimer, t_sec, t_min;
  unsigned char data[2];
  unsigned char out=0;
  unsigned int decval;
  volatile float pp, pi, pd, f_error, error_old=0, pid_out, pid_out_i, pid_out_p, pid_out_d, error_i=0;
  u8 Send_Buffer[25];
  u8 tmp[4];
//char no_windup = 0;

  RCC_Configuration();
  NVIC_Configuration();

  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
  SysTick_SetReload(9000);

  /* Enable SysTick interrupt */
  SysTick_ITConfig(ENABLE);

  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);

  GPIO_Setup();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* SPI2 Config -------------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  /* Enable SPI1 */
  SPI_CalculateCRC(SPI2, DISABLE);
  SPI_Cmd(SPI2, ENABLE);

  InitADC1();      // ADC1 Init

  OWInit(&OneWire, GPIOB, GPIO_Pin_8);

  /* Connect Key Button EXTI Line to Key Button GPIO Pin */
  //GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_Pin_7);

  /* Configure Key Button EXTI Line to generate an interrupt on falling edge */
  //EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  //EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  //EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //EXTI_Init(&EXTI_InitStructure);

    /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 55000;
  TIM_TimeBaseStructure.TIM_Prescaler = 12;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* TIM2 PWM2 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_OCInitStructure.TIM_Pulse = 1;//20000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OCInit(TIM3, &TIM_OCInitStructure);

  /* TIM2 configuration in Input Capture Mode */

  TIM_ICStructInit(&TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;

  TIM_ICInit(TIM3, &TIM_ICInitStructure);

  /* One Pulse Mode selection */
  TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Single);

  /* Input Trigger selection */
  TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);

  /* Slave Mode selection: Trigger Mode */
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Trigger);

  ST7565_st7565_init();
  ST7565_st7565_command(CMD_SET_BIAS_9);
  ST7565_st7565_command(CMD_DISPLAY_ON);
  ST7565_st7565_command(CMD_SET_ALLPTS_NORMAL);
  ST7565_st7565_set_brightness(0x0C);

  OWSearch(&OneWire, addr);
  /*sprintf(test, "%02X %02X %02X %02X %02X %02X %02X %02X",
         addr[7],addr[6],
         addr[5],addr[4],
         addr[3],addr[2],
         addr[1],addr[0]);
  ST7565_drawstring(6, 6, test);
*/

  USB_Init();

  ST7565_display(); // show splashscreen
  t0 = millis();
  HTimer = millis();
  TTimer = millis();
  pp = 10;
  pi = 0;
  pd = 150;
  while(1)
  {
	msum[0] = 0;
	msum[1] = 0;
	msum[2] = 0;
	msum[3] = 0;
	for (i = 0; i < 1000; i++) {
      an1 = GetADC1Channel(ADC_Channel_1);
      an2 = GetADC1Channel(ADC_Channel_2);
      an3 = GetADC1Channel(ADC_Channel_3);
      an4 = GetADC1Channel(ADC_Channel_4);
      an5 = GetADC1Channel(ADC_Channel_5);
      msum[0] += an1;
      msum[1] += an3 - an2;
      msum[2] += an4 - an2;
      msum[3] += an5 - an2;
      //DelayuS(333);
	}
	SP = round((msum[0] / 1000.0) * (60.0 / 4096)) * 5;
	fm[1] = (msum[1] / 1000.0);
	fm[2] = (msum[2] / 1000.0) + 12;
	fm[3] = (msum[3] / 1000.0) - 7;

    T1 = (T1 + Ta + Dac2Dt(fm[1])) / 2;
    T2 = (T2 + Ta + Dac2Dt(fm[2])) / 2;
    T3 = (T3 + Ta + Dac2Dt(fm[3])) / 2;

    t1 = millis();
    dt = t1 - t0;
 	t0 = t1;
    if (millis() - HTimer > 1000) {
      f_error = SP - T2;
      //if (noerror_i += error;
      pid_out_p = pp * f_error;
      pid_out_i = pi * error_i;
      pid_out_d = pd * (f_error - error_old);
      pid_out = pid_out_p + pid_out_i + pid_out_d;
      error_old = f_error;
      //out = pid_out;
      if (pid_out > 99) {
    	out = 99;
      } else if (pid_out < 0) {
        out = 0;
      } else {
        out = round(pid_out);
      }
      TIM_SetCompare1(TIM3, 55000-PowerValues[out]);
	  HTimer += 1000;

    //error_old = 10;
    sprintf(test, "T1 : %5.1f E  %5.1f ", T1, f_error);
    ST7565_drawstring(6, 0, test);
	sprintf(test, "T2 : %5.1f ", T2);
    ST7565_drawstring(6, 1, test);
	sprintf(test, "T3 : %5.1f ", T3);
    ST7565_drawstring(6, 2, test);
	sprintf(test, "SP : %5.1f P %6.1f ", SP, pid_out_p);
    ST7565_drawstring(6, 3, test);
	sprintf(test, "Ta : %5.1f I %6.1f ", Ta, pid_out_i);
    ST7565_drawstring(6, 4, test);
    sprintf(test, "dt : %5lu D %6.1f ", dt, pid_out_d);
    ST7565_drawstring(6, 5, test);
 	sprintf(test, "out: %3u %%   %6.1f ", out, pid_out);
    ST7565_drawstring(6, 6, test);
    t_sec = (millis() - TTimer) / 1000;
    t_min = floor(t_sec / 60);
    t_sec %= 60;
 	Send_Buffer[0] = 0x07;
 	decval = round(T1 * 100);
 	memcpy(&Send_Buffer[1], &decval, 2);
 	decval = round(T2 * 100);
 	memcpy(&Send_Buffer[3], &decval, 2);
 	decval = round(T3 * 100);
 	memcpy(&Send_Buffer[5], &decval, 2);
 	decval = round(SP * 100);
 	memcpy(&Send_Buffer[7], &decval, 2);
 	memcpy(&Send_Buffer[9], &out, 1);

 	//sprintf(test, "%02X %02X %02X %02X ", Send_Buffer[1], Send_Buffer[2], Send_Buffer[3], Send_Buffer[4]);
    //ST7565_drawstring(6, 7, test);

    UserToPMABufferCopy(Send_Buffer, ENDP1_TXADDR, 9);
    SetEPTxCount(ENDP1, 9);
    SetEPTxValid(ENDP1);
    ST7565_display();
 	}
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0) {
      TTimer = millis();
    }

    //onewire
    switch (DSState){
    case 0:
      OWReset(&OneWire);
      OWWrite(&OneWire, 0xCC);         // skip ROM
      OWWrite(&OneWire, 0x44);         // start conversion
      DSTimer = millis();
      DSState++;
      break;
    case 1:
      if((millis() - DSTimer) >= 1000){
    	OWReset(&OneWire);
        OWSelect(&OneWire, addr);
        OWWrite(&OneWire, 0xBE);             // Read Scratchpad
        data[0] = OWRead(&OneWire);
        data[1] = OWRead(&OneWire);
        Ta = ((data[1] << 8) | data[0]) / 16.0;
        DSState = 0;
      }
      break;
    }
  }
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

/*******************************************************************************
* Function Name  : USB_Cable_Config.
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : NewState: new state.
* Output         : None.
* Return         : None
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(GPIOD, GPIO_Pin_2);
  }
  else
  {
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
  }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  u32 Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(u32*)(0x1FFFF7E8);
  Device_Serial1 = *(u32*)(0x1FFFF7EC);
  Device_Serial2 = *(u32*)(0x1FFFF7F0);

  if(Device_Serial0 != 0)
  {
     CustomHID_StringSerial[2] = (u8)(Device_Serial0 & 0x000000FF);
     CustomHID_StringSerial[4] = (u8)((Device_Serial0 & 0x0000FF00) >> 8);
     CustomHID_StringSerial[6] = (u8)((Device_Serial0 & 0x00FF0000) >> 16);
     CustomHID_StringSerial[8] = (u8)((Device_Serial0 & 0xFF000000) >> 24);

     CustomHID_StringSerial[10] = (u8)(Device_Serial1 & 0x000000FF);
     CustomHID_StringSerial[12] = (u8)((Device_Serial1 & 0x0000FF00) >> 8);
     CustomHID_StringSerial[14] = (u8)((Device_Serial1 & 0x00FF0000) >> 16);
     CustomHID_StringSerial[16] = (u8)((Device_Serial1 & 0xFF000000) >> 24);

     CustomHID_StringSerial[18] = (u8)(Device_Serial2 & 0x000000FF);
     CustomHID_StringSerial[20] = (u8)((Device_Serial2 & 0x0000FF00) >> 8);
     CustomHID_StringSerial[22] = (u8)((Device_Serial2 & 0x00FF0000) >> 16);
     CustomHID_StringSerial[24] = (u8)((Device_Serial2 & 0xFF000000) >> 24);
  }
}

void zeroCrossDetect(void){
  edgeCount++;
}

float Dac2Dt(float DacValue) {
  float v1, v2, v3, v4, v5, v6, v7, v8, v9;
  v1 = DacValue * (33000 / 4096);
  v2 = v1 * v1;
  v3 = v2 * v1;
  v4 = v3 * v1;
  v5 = v4 * v1;
  v6 = v5 * v1;
  v7 = v6 * v1;
  v8 = v7 * v1;
  v9 = v8 * v1;
  v1 *= a1;
  v2 *= a2;
  v3 *= a3;
  v4 *= a4;
  v5 *= a5;
  v6 *= a6;
  v7 *= a7;
  v8 *= a8;
  v9 *= a9;
  return v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8 + v9;
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
  /* Configure one bit for preemption priority */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Enable the EXTI9_5 Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

unsigned long millis(void){
  return msCounter;
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void DelayuS(vu32 nCount)
{
  nCount *= 8;
  while (nCount--);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nTime: specifies the delay time length, in milliseconds.
* Output         : None
* Return         : None
*******************************************************************************/
void DelaymS(u32 nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

/*// Disable SysTick Counter
  SysTick_CounterCmd(SysTick_Counter_Disable);
  // Clear SysTick Counter
  SysTick_CounterCmd(SysTick_Counter_Clear);
  */
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  msCounter++;
  if (TimingDelay != 0x00)
    TimingDelay--;
}

/*************************************************************************
 * Function Name: InitADC1
 * Parameters: none
 * Return: none
 *
 * Description: ADC Init subroutine
 *
 *************************************************************************/
void InitADC1(void)
{
  ADC_InitTypeDef   ADC_InitStructure;

  // ADC init
  // ADC Deinit
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC_DeInit(ADC1);

  // ADC Structure Initialization
  ADC_StructInit(&ADC_InitStructure);

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  // Enable the ADC
  ADC_Cmd(ADC1, ENABLE);

  // ADC calibration
  // Enable ADC1 reset calibaration register
  ADC_ResetCalibration(ADC1);

  // Check the end of ADC1 reset calibration register
  while(ADC_GetResetCalibrationStatus(ADC1) == SET);

  // Start ADC1 calibaration
  ADC_StartCalibration(ADC1);

  // Check the end of ADC1 calibration
  while(ADC_GetCalibrationStatus(ADC1) == SET);

}

/*************************************************************************
 * Function Name: GetADC1Channel
 * Parameters: Int8U channel
 * Return: Int16U
 *
 * Description: ADC Convert
 *
 *************************************************************************/
unsigned int GetADC1Channel(unsigned char chanel)
{
  // Configure channel
  ADC_RegularChannelConfig(ADC1, chanel, 1, ADC_SampleTime_55Cycles5);

  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
}

void GPIO_Setup(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOG Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_GPIOD, ENABLE);

//Analog inputs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |
                                GPIO_Pin_2 |
                                GPIO_Pin_3 |
                                GPIO_Pin_4 |
                                GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//LEDS
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 |
                                GPIO_Pin_10 |
                                GPIO_Pin_11 |
                                GPIO_Pin_12 |
                                GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

//DS sensor
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

//SPI2 pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |
                                GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


//TAMP switch
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

//WKUP switch
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//zero crossing
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//optotriac
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
