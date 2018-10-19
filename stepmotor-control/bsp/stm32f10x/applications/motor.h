
#ifndef __STM32F10X_SYSTEM_H__
#define __STM32F10X_SYSTEM_H__



extern void System_Init(void);
extern void	GPIO_Configuration(void);				//初始化一些端口的功能,但也许会有一些模块会在其驱动程序中用户自行初始化	

extern void	NVIC_Configuration(void);				//初始化NVIC中断系统,以及使能一些NVIC中断
extern void PH_RCC_Configuration(void);
void TimerDly(unsigned int Timer);
void ResetSystem(void);

//USART1
extern void USART1_Initial(void);
extern void USART1_Printfstr(unsigned char *p);
extern void USART1_Printf(unsigned char *q,unsigned char len); 
//USART3
extern void USART3_Initial(void);
extern void USART3_Printfstr(unsigned char *p);
extern void USART3_Printf(unsigned char *q,unsigned char len);
//CAN
extern void CAN_Initial(void);
extern void CAN_SendData(unsigned char *q,unsigned char len,unsigned char Type); 
//模块地址
extern unsigned char ReadSubAddr(void);
//系统滴答
extern void InitSysTick(void); 
//外部中断
extern void EXTI_Configuration(void);
//电机IO初始化
extern void Initial_MotorIO(void);
//电机初始化
extern void Initial_Motor(unsigned char MotorID, unsigned char StepDive);
//各路PWM初始化
extern void Initial_PWM1(void);
extern void Initial_PWM2(void);
extern void Initial_PWM3(void);
extern void ReagentMotorConfiguration(void);
extern void Initial_PWM4(void);
//启动电机
extern void Start_Motor(unsigned char MotorID,unsigned char dir,unsigned int Degree);
//阀门IO初始化
extern void Initial_ValveIO(void);
//开阀门
extern void TurnOnValve(unsigned char valvenum);
//关阀门
extern void TurnOffValve(unsigned char valvenum);
//泵IO初始化 
extern void Initial_PumpIO(void);
//开泵
extern void TurnOnPump(unsigned char valvenum);
//关泵
extern void TurnOffPump(unsigned char valvenum);

//LED&Beeper
extern void Initial_LEDBeeper(void);
extern void TurnOnLED( unsigned char LEDNum);
extern void TurnOffLED( unsigned char LEDNum);
extern void TurnOnBeeper( void);
extern void TurnOffBeeper( void);
//EEROM
extern void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite);
extern void I2C2_EE_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
extern void I2C2_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite);
extern void I2C2_EE_ByteWrite(u8* pBuffer, u8 WriteAddr);
extern void I2C2_EE_Init(void);

extern void I2C_GPIOInitial(void);
extern bool I2C_FRAM_BufferWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
extern bool I2C_FRAM_VerifyWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
extern bool I2C_FRAM_BufferRead(u8* pBuffer, u16 WriteAddr, u16 NumByteToRead);


//协议解析函数
extern void ProtocolAnalyse_CAN(unsigned char *buf); 
extern void ProtocolAnalyse_USART1(unsigned char *buf); 
extern void ReagentInfInitial(void);
extern void BeadInfInitial(void);
extern void InitialCupTask(void);
extern void ReagentTaskScheduling(void);
extern void BeadTaskScheduling(void);
extern void PrintReagenttoAddInf(void);
extern void PrintReagentRemainInf(void);
extern void AddReagentOnce(void);
extern void AddReagentAction(void);
extern void AddBeadAction(void);
void MotorRunParaInitial(void);

extern void PCModifiableVarialInit(void);
extern bool RewriteEEROM_Motor1(void);
extern bool RewriteEEROM_Motor2(void);
extern bool RewriteEEROM_Motor3(void);
extern bool RewriteEEROM_Motor4(void);
extern void StopBroadCast(unsigned char ErrorCode);
extern void ErrorReport(unsigned char* ErrorCode);
__inline void AddReagenOnce(void);
__inline void AddReagenTwice(void);
void  AddBeadAction(void);

extern void HeartBeatBroadCast(void);
void ReporttoReactionCupModule(unsigned char state);
							



#endif

