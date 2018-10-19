//Í·ÎÄ¼þ
#include "include.h" 
#include "ringbuffer.h"

volatile uint8_t  USART1_TxBuffer[USART1TXSIZE];	// ´®¿Ú1·¢ËÍ»º³åÇø
volatile uint16_t PTxBufferUSART11=0;			    // ´®¿Ú1·¢ËÍÇ°ÏòÎ»ÖÃ
volatile uint16_t PTxBufferUSART12=0;			    // ´®¿Ú1·¢ËÍºóÏòÎ»ÖÃ£¬ºóÏò-Ç°Ïò=Î´·¢ËÍµÄÊý¾Ý
volatile uint16_t USART1_TxCounter =0;				// ´®¿Ú1·¢ËÍ¼ÆÊý
volatile uint16_t USART1_RxCounter =0; 				// ´®¿Ú1½ÓÊÕ¼ÆÊý
volatile uint16_t USART1_NbrOfDataToTransfer=0;		// ´®¿Ú1Òª·¢ËÍµÄÊý¾Ý¸öÊý
volatile uint8_t  USART1_RxBuffer[USART1RXSIZE];	// ´®¿Ú1½ÓÊÕ»º³åÇø
volatile uint16_t PRxBufferUSART11=0;
volatile uint16_t PRxBufferUSART12=0;
volatile uint8_t  USART1_NbrOfDataReceived=0;		// ´®¿Ú1Òª½ÓÊÕµÄÊý¾Ý¸öÊý
volatile uint8_t  USART1_Received_Flag=0;			// ´®¿Ú1ÊÕµ½Êý¾Ý±êÖ¾£¬1£º½ÓÊÕµ½Êý¾Ý£¬0£ºÃ»ÓÐ½ÓÊÕµ½Êý¾Ý
struct rt_ringbuffer rb_recv;


volatile uint8_t  USART2_TxBuffer[USART2TXSIZE];	// ´®¿Ú2·¢ËÍ»º³åÇø
volatile uint16_t PTxBufferUSART21=0;				// ´®¿Ú2·¢ËÍÇ°ÏòÎ»ÖÃ
volatile uint16_t PTxBufferUSART22=0;				// ´®¿Ú2·¢ËÍºóÏòÎ»ÖÃ£¬ºóÏò-Ç°Ïò=Î´·¢ËÍµÄÊý¾Ý
volatile uint16_t USART2_TxCounter=0 ;				// ´®¿Ú2·¢ËÍ¼ÆÊý
volatile uint16_t USART2_RxCounter=0 ; 				// ´®¿Ú2½ÓÊÕ¼ÆÊý
volatile uint16_t USART2_NbrOfDataToTransfer=0;		// ´®¿Ú2Òª·¢ËÍµÄÊý¾Ý¸öÊý
volatile uint8_t  USART2_RxBuffer[USART2RXSIZE];	// ´®¿Ú2½ÓÊÕ»º³åÇø
volatile uint16_t PRxBufferUSART21=0;
volatile uint16_t PRxBufferUSART22=0;
volatile uint8_t  USART2_NbrOfDataReceived=0;		// ´®¿Ú2Òª½ÓÊÕµÄÊý¾Ý¸öÊý
volatile uint8_t  USART2_Received_Flag=0;			// ´®¿Ú2ÊÕµ½Êý¾Ý±êÖ¾£¬1£º½ÓÊÕµ½Êý¾Ý£¬0£ºÃ»ÓÐ½ÓÊÕµ½Êý¾Ý


volatile uint8_t  USART3_TxBuffer[USART3TXSIZE];	// ´®¿Ú3·¢ËÍ»º³åÇø
volatile uint16_t PTxBufferUSART31=0;				// ´®¿Ú3·¢ËÍÇ°ÏòÎ»ÖÃ
volatile uint16_t PTxBufferUSART32=0;				// ´®¿Ú3·¢ËÍºóÏòÎ»ÖÃ£¬ºóÏò-Ç°Ïò=Î´·¢ËÍµÄÊý¾Ý
volatile uint16_t USART3_TxCounter=0 ;				// ´®¿Ú3·¢ËÍ¼ÆÊý
volatile uint16_t USART3_RxCounter=0 ; 				// ´®¿Ú3½ÓÊÕ¼ÆÊý
volatile uint16_t USART3_NbrOfDataToTransfer=0;		// ´®¿Ú3Òª·¢ËÍµÄÊý¾Ý¸öÊý
volatile uint8_t  USART3_RxBuffer[USART3RXSIZE];	// ´®¿Ú3½ÓÊÕ»º³åÇø
volatile uint16_t PRxBufferUSART31=0;
volatile uint16_t PRxBufferUSART32=0;
volatile uint8_t  USART3_NbrOfDataReceived=0;		// ´®¿Ú3Òª½ÓÊÕµÄÊý¾Ý¸öÊý
volatile uint8_t  USART3_Received_Flag=0;			// ´®¿Ú3ÊÕµ½Êý¾Ý±êÖ¾£¬1£º½ÓÊÕµ½Êý¾Ý£¬0£ºÃ»ÓÐ½ÓÊÕµ½Êý¾Ý


#define F2TIME_PARA				12000000   			// ½«ÆµÂÊÖµ×ª»»Îª¶¨Ê±Æ÷¼Ä´æÆ÷ÖµµÃ×ª»»²ÎÊý
#define STEP_PARA				10	   				// ÈÎÒâÊ±¿Ì×ª¶¯²½ÊýÐÞÕýÒò×Ó
#define STEP_AA					31       			// ¼Ó¼ÓËÙ½×¶Î£¬ÀëÉ¢»¯µãÊý
#define STEP_UA					31			  		// ÔÈ¼ÓËÙ½×¶Î£¬ÀëÉ¢»¯µãÊý
#define STEP_RA					31					// ¼õ¼ÓËÙ½×¶Î£¬ÀëÉ¢»¯µãÊý

#define STEP_SPTA				20					// SPTA×î´óËÙ¶ÈµÈ¼¶
#define MAXSPEED_SPTA			80000				// SPTA×î´óËÙ¶È
#define ACCSPEED_SPTA			150000				// SPTA¼ÓËÙ¶È


/*Õý³£SÐÍÇúÏß²ÎÊýÉú³ÉµÄ±í¸ñ*/
uint16_t Motor1TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor1StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor2TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor2StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor3TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor3StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};	

/*½«²ÎÊý½µÎª2/3 SÐÍÇúÏß²ÎÊýÉú³ÉµÄ±í¸ñ*/
uint16_t Motor1_23TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor1_23StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor2_23TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor2_23StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor3_23TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor3_23StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};	

/*½«²ÎÊý½µÎª1/3 SÐÍÇúÏß²ÎÊýÉú³ÉµÄ±í¸ñ*/
uint16_t Motor1_13TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor1_13StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor2_13TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor2_13StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor3_13TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor3_13StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};	
	
unsigned long long Get_Time_Cost(unsigned char MotorID);

//USART1Ê±ÖÓ³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1Ê±ÖÓ³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART1_RCC_Configuration(void)
{
	/*******************´®¿Ú1µÄÊ±ÖÓ³õÊ¼»¯********************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
}
/**********************End****************/

//USART1Òý½Å³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1Òý½Å³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART1_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the USART2 Pins Software Remapping */
	//GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	/* Configure USART1 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/**********************End****************/


//USART1ÖÐ¶Ïº¯Êý³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1ÖÐ¶Ïº¯Êý³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART1_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/**********************End****************/

//USART1¼Ä´æÆ÷³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1¼Ä´æÆ÷³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART1_Initial(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART1_RCC_Configuration();
	USART1_GPIO_Configuration();
	USART1_NVIC_Configuration();
	USART_InitStructure.USART_BaudRate =115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	//USART1_NbrOfDataToTransfer=0;

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
//USART1·¢ËÍÊý¾Ý
/*Èë¿Ú²ÎÊý£ºÒª·¢ËÍµÄbufferÖ¸Õë£¬Òª·¢ËÍµÄÊý¾Ý³¤¶È****/
/*º¯Êý¹¦ÄÜ£ºUSART1·¢ËÍÊý¾Ý**************************/
/*·µ»Ø²ÎÊý£ºÎÞ**************************************/
/*¶ÔÓÚ²»¶¨³¤µÄ·¢ËÍ£¬¿ÉÒÔ²ÉÓÃÏÈÅÐ¶ÏÒª·¢ËÍµÄÊý¾Ý³¤¶ÈºÍ
  Ê£Óà»º³åÇø´óÐ¡£¬ÔÙ¾ö¶¨ÈçºÎ¿½±´µÄ·½·¨£¬ÒÔ¼õÉÙ
  PTxBufferUSART12ÊÇ·ñÔ½½çµÄÅÐ¶Ï£¬µ«ÊÇÖ®Ç°´¦ÀíµÄ·½Ê½
  ÊÇÄ¬ÈÏ²»»áÒç³ö£¬ºóÆÚ¿ÉÄÜ»á½«µ÷ÊÔÐÅÏ¢É¾³ý£¬ËùÒÔ¾Í²»
  ¸ÄÁË£¬ÏÂÍ¬
*********************Start************************/
void USART1_Printf(unsigned char *q,unsigned char len)
{
	u8 i=0;

	for(i=0;i<len;i++)
	{
		USART1_TxBuffer[PTxBufferUSART12++]=q[i];
		if(PTxBufferUSART12>=USART1TXSIZE)
		{
			PTxBufferUSART12=0;	
		}
	}
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);	    
}

//USART1·¢ËÍÊý¾Ý
/*Èë¿Ú²ÎÊý£ºÒª·¢ËÍµÄbufferÖ¸Õë£¬Ò»°ãÊÇ×Ö·ûÊý×é***/
/*º¯Êý¹¦ÄÜ£ºUSART1·¢ËÍÊý¾Ý***********************/
/*·µ»Ø²ÎÊý£ºÎÞ***********************************/
/**********************Start*********************/
void USART1_Printfstr(unsigned char *p)
{
	while((*p)!=0)
	{
		USART1_TxBuffer[PTxBufferUSART12++]=*p;
		p++;
		if(PTxBufferUSART12>=USART1TXSIZE)
		{
			PTxBufferUSART12=0;	
		}
	}
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}
/**********************End****************/

//USART2Ê±ÖÓ³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1Ê±ÖÓ³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART2_RCC_Configuration(void)
{
	/*******************´®¿Ú1µÄÊ±ÖÓ³õÊ¼»¯********************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
}
/**********************End****************/

//USART2Òý½Å³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1Òý½Å³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART2_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the USART2 Pins Software Remapping */
	//GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	/* Configure USART2 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART2 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/**********************End****************/


//USART2ÖÐ¶Ïº¯Êý³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART2ÖÐ¶Ïº¯Êý³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART2_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/**********************End****************/

//USART2¼Ä´æÆ÷³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1¼Ä´æÆ÷³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART2_Initial(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART2_RCC_Configuration();
	USART2_GPIO_Configuration();
	USART2_NVIC_Configuration();
	USART_InitStructure.USART_BaudRate =9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	//USART1_NbrOfDataToTransfer=0;

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

void USART2_Printf(unsigned char *q,unsigned char len)
{
	u8 i = 0;

	for(i = 0; i < len; i++)
	{
		USART2_TxBuffer[PTxBufferUSART22++] = q[i];
		if(PTxBufferUSART22 >= USART2TXSIZE)
		{
			PTxBufferUSART22 = 0;	
		}
	}
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);	    
}

//USART2·¢ËÍÊý¾Ý
/*Èë¿Ú²ÎÊý£ºÒª·¢ËÍµÄbufferÖ¸Õë£¬Ò»°ãÊÇ×Ö·ûÊý×é***/
/*º¯Êý¹¦ÄÜ£ºUSART2·¢ËÍÊý¾Ý***********************/
/*·µ»Ø²ÎÊý£ºÎÞ***********************************/
/**********************Start*********************/
void USART2_Printfstr(unsigned char *p)
{
	while((*p)!=0)
	{
		USART2_TxBuffer[PTxBufferUSART22++]=*p;
		p++;
		if(PTxBufferUSART22>=USART2TXSIZE)
		{
			PTxBufferUSART22=0;	
		}
	}
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

//USART3Ê±ÖÓ³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART3Ê±ÖÓ³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART3_RCC_Configuration(void)
{
	/*******************´®¿Ú1µÄÊ±ÖÓ³õÊ¼»¯********************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);	//´®¿Ú3Ê¹ÓÃD8 D9
}
/**********************End****************/

//USART1Òý½Å³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1Òý½Å³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART3_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the USART3 Pins Software Remapping */
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);  //ÐèÒªFULL Remap²ÅÄÜÓ³Éäµ½D8 D9ÉÏ

	/* Configure USART3 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure USART3 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
}
/**********************End****************/


//USART1ÖÐ¶Ïº¯Êý³õÊ¼»¯
/*Èë¿Ú²ÎÊý£ºÎÞ******************************/
/*º¯Êý¹¦ÄÜ£ºUSART1ÖÐ¶Ïº¯Êý³õÊ¼»¯*/
/*·µ»Ø²ÎÊý£ºÎÞ******************************/
/**********************Start****************/
void USART3_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */  
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/**********************End****************/
/*********************USART3³õÊ¼»¯**********/  
void USART3_Initial(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART3_RCC_Configuration();
	USART3_GPIO_Configuration();
	USART3_NVIC_Configuration();
	USART_InitStructure.USART_BaudRate =115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

void USART3_Printf(unsigned char *q,unsigned char len)
{
	u8 i=0;

	for(i=0;i<len;i++)
	{
		USART3_TxBuffer[PTxBufferUSART32++]=q[i];
		if(PTxBufferUSART32>=USART3TXSIZE)
		{
			PTxBufferUSART32=0;	
		}
	}
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);	    
}

//USART2·¢ËÍÊý¾Ý
/*Èë¿Ú²ÎÊý£ºÒª·¢ËÍµÄbufferÖ¸Õë£¬Ò»°ãÊÇ×Ö·ûÊý×é***/
/*º¯Êý¹¦ÄÜ£ºUSART2·¢ËÍÊý¾Ý***********************/
/*·µ»Ø²ÎÊý£ºÎÞ***********************************/
/**********************Start*********************/
void USART3_Printfstr(unsigned char *p)
{
	while((*p)!=0)
	{
		USART3_TxBuffer[PTxBufferUSART32++]=*p;
		p++;
		if(PTxBufferUSART32>=USART3TXSIZE)
		{
			PTxBufferUSART32=0;	
		}
	}
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

/*
µç»ú1£ºPA8(pwm),PE9(CW),PE8(ENABLE),MXX:PA11,PA12,PE7
µç»ú2£ºPA0(pwm),PA1(CW),PC3(ENABLE),MXX:PC0,PC1,PC2
µç»ú3£ºPA6(pwm),PA7(CW),PC4(ENABLE),MXX:PA3,PA4,PA5
µç»ú4£ºPB6(pwm,gpio),PB9(CW),PB8(ENABLE),MXX:PD7,PB5,PB7
*/
void Initial_MotorIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_11|GPIO_Pin_12;		  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_11|GPIO_Pin_12);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;		  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_7);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;		  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);


	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_6;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_6);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
}


 /**************************************************************************************
 ³õÊ¼»¯µç»úµÄ²ÎÊý£¬Ö÷ÒªÊÇÏ¸·ÖÑ¡Ôñ£¬Ê¹ÓÃµÄ¶¨Ê±Æ÷£¬Ë³Ê±Õë·½ÏòÖµ£¬µç»úIDµÈ
 **************************************************************************************/
void Initial_Motor(unsigned char MotorID, unsigned char StepDive,unsigned int maxposition)
{
	unsigned int i = 0;
	MOTOR_CONTROL_S    *pmotor = NULL; 
	MOTOR_CONTROL_SPTA *pmotor_spta = NULL; 
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;
	switch(StepDive)
	{
		case 1: i = 0x00; break;
		case 2: i = 0x01; break;
		case 4: i = 0x02; break;
		case 8: i = 0x03; break;
		case 16:  i = 0x04; break;
		case 32:  i = 0x05; break;
		case 64:  i = 0x06; break;
		case 128: i = 0x07; break;
		default : i = 0x00; break;
	}
	
	switch(MotorID)
	{
		case 1:			 
			if(i & 0x01)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_11);
			}
			if(i & 0x02)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_12);
			}

			if(i & 0x04)
			{
				GPIO_SetBits(GPIOE, GPIO_Pin_7);
			}
			GPIO_SetBits(GPIOE, GPIO_Pin_8);

			pmotor =& motor1;
			motor1.id = 1;
			motor1.clockwise = M1_CLOCKWISE;
			motor1.TIMx = TIM1;
			MotorTimeTable = Motor1TimeTable;
			MotorStepTable = Motor1StepTable;			
			break;
		case 2:			 
			if(i&0x01)
			{
				GPIO_SetBits(GPIOC, GPIO_Pin_0);
			}
			if(i&0x02)
			{
				GPIO_SetBits(GPIOC, GPIO_Pin_1);
			}

			if(i&0x04)
			{
				GPIO_SetBits(GPIOC, GPIO_Pin_2);
			} 			
			GPIO_SetBits(GPIOC, GPIO_Pin_3);
			pmotor =& motor2;
			motor2.id = 2;
			motor2.clockwise = M2_CLOCKWISE;
			motor2.TIMx = TIM2;
			MotorTimeTable = Motor2TimeTable;
			MotorStepTable = Motor2StepTable;
			break;
		case 3:
			if(i & 0x01)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
			}
			if(i & 0x02)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_4);
			}

			if(i & 0x04)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_5);
			}
			GPIO_SetBits(GPIOC, GPIO_Pin_4);
			pmotor =& motor3;
			motor3.id = 3;
			motor3.clockwise = M3_CLOCKWISE;
			motor3.TIMx = TIM3;
			MotorTimeTable = Motor3TimeTable;
			MotorStepTable = Motor3StepTable;
			break;
		case 4:			
			if(i & 0x01)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_7);
			}
			if(i & 0x02)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_5);
			}

			if(i & 0x04)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_7);
			}
			GPIO_SetBits(GPIOB, GPIO_Pin_8);
			motor4.id = 4;
			motor4.clockwise = M4_CLOCKWISE;
			motor4.TIMx = TIM4;
			motor4.divnum = StepDive;
			motor4.GPIOBASE = GPIOB;
			motor4.PWMGPIO = GPIO_Pin_6;
			pmotor_spta =& motor4;
			break;		
		default: break;
	}
	if((MotorID <= 3) && (MotorID >= 1))
	{
		pmotor->divnum = StepDive;
		pmotor->MaxPosition = maxposition;
		pmotor->MaxPosition_Pulse = maxposition*StepDive;

		pmotor->CurrentPosition = 0;
		pmotor->CurrentPosition_Pulse = 0;
		pmotor->StartTableLength = STEP_AA + STEP_UA + STEP_RA + 1;
		pmotor->StopTableLength = STEP_AA + STEP_UA + STEP_RA; 
		pmotor->Counter_Table = MotorTimeTable;
		pmotor->Step_Table = MotorStepTable;

		pmotor->CurrentIndex = 0;
		pmotor->speedenbale = 0;
		pmotor->StartSteps = 0;                  //±ØÐëÇåÁã£¬ºóÃæÊÇÀÛ¼Ó£¬·ñÔò»á°ÑÇ°Ò»´ÎµÄ¼ÓÉÏ
		pmotor->StopSteps = 0;                   //Í¬ÉÏ
		for(i = 0; i < pmotor->StartTableLength; i++)
			pmotor->StartSteps += pmotor->Step_Table[i];
		for(i = 0; i < pmotor->StopTableLength; i++)
			pmotor->StopSteps += pmotor->Step_Table[i+pmotor->StartTableLength];

		pmotor->TIMx->ARR  = pmotor->Counter_Table[0];        //ÉèÖÃÖÜÆÚ
		pmotor->TIMx->CCR1 = pmotor->Counter_Table[0] >> 1;   //ÉèÖÃÕ¼¿Õ±È
	}
	if(MotorID==4)
	{
		pmotor_spta->divnum = StepDive;
		pmotor_spta->MaxPosition = maxposition;
		pmotor_spta->MaxPosition_Pulse = maxposition*StepDive;		
	}
}

/*¶àÖáÐ­Í¬Ê¹ÓÃÁËËã·¨Ô­Àí½øÐÐÊ±¼äÔ¤¹À£¬ËùÒÔÐÞ¸Ä¸ÃËã·¨Ê±¼ÇµÃÕâÁ½´¦±£³ÖÍ¬²½*/
/*¼ÆËãSÐÍÇúÏß·´×ªµã£¬SÐÍÇúÏßÔÚÔËÐÐÊ±£¬¼Ó¼õËÙ¹ý³ÌÊÇÍêÈ«¶Ô³ÆµÄ*/
unsigned long long Get_TimeCost_ReverDot_S(unsigned char MotorID)
{
	unsigned long long time_cost  = 0;
	unsigned long long time_cost2 = 0;
	unsigned int pulsecnt = 0;
	int i = 0, j;
	MOTOR_CONTROL_S *pmotor = NULL; 
	switch(MotorID)
	{
		case 1:
			pmotor =& motor1;  break;
		case 2:
			pmotor =& motor2;  break;
		case 3:
			pmotor =& motor3;  break;
		default:  return 0;
	}
	
	if(pmotor->PulsesGiven>=pmotor->StartSteps+pmotor->StopSteps)
	{
		for(i=0;i<pmotor->StartTableLength;i++)
			time_cost += (pmotor->Step_Table[i]*pmotor->Counter_Table[i]);
		for(i=0;i<pmotor->StopTableLength;i++)
			time_cost += (pmotor->Step_Table[i+pmotor->StartTableLength]*pmotor->Counter_Table[i+pmotor->StartTableLength]);		
		time_cost += (pmotor->PulsesGiven-pmotor->StartSteps-pmotor->StopSteps)*pmotor->Counter_Table[pmotor->StartTableLength-1];
		
		pmotor->RevetDot = pmotor->PulsesGiven-pmotor->StopSteps;
	}
	else
	{
		//¿¼ÂÇÕâÖÖÇé¿ö£¬µÚÒ»ÆµÂÊ142 ²½£¬µÚ¶þÆµÂÊ148²½£¬ÒªÊÇÔË¶¯200²½¸ÃÔõÃ´ÔËÐÐ
		//ËùÒÔÕâÀïÒª¸Ä±äµÚ¶þÆµÂÊµÄ²½Êý
		while((pulsecnt+pmotor->Step_Table[i]) <= (pmotor->PulsesGiven>>1))
		{					
			time_cost  += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
			time_cost2 += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
			pulsecnt += pmotor->Step_Table[i];
			i++;
		}
		time_cost += time_cost2;
		if(pmotor->Step_Table[i]<pmotor->PulsesGiven-2*pulsecnt)
		{
			pmotor->Step_Table[i] = pmotor->PulsesGiven-2*pulsecnt;
			pmotor->StartSteps = 0;                  //±ØÐëÇåÁã£¬ºóÃæÊÇÀÛ¼Ó£¬·ñÔò»á°ÑÇ°Ò»´ÎµÄ¼ÓÉÏ
			pmotor->StopSteps = 0;                   //Í¬ÉÏ
			for(j = 0; j < pmotor->StartTableLength; j++)
				pmotor->StartSteps += pmotor->Step_Table[j];
			for(j = 0;j < pmotor->StopTableLength; j++)
				pmotor->StopSteps += pmotor->Step_Table[j+pmotor->StartTableLength];
		}
		time_cost += (pmotor->Counter_Table[i]*(pmotor->PulsesGiven-2*pulsecnt));
		pmotor->RevetDot = pmotor->PulsesGiven-pulsecnt;
	}
	pmotor->Time_Cost_Cal = time_cost;
	return time_cost;
}

unsigned long long Get_Time_Cost2(unsigned char MotorID)
{
	extern void TIM1_UP_IRQHandler(void);
	extern void TIM2_IRQHandler(void);
	extern void TIM3_IRQHandler(void);
	switch(MotorID)
	{
		case 1:
			while(motor1.running == 1)
			{
				TIM1_UP_IRQHandler();
			}
			return motor1.Time_Cost_Act; 
		case 2:
			while(motor2.running == 1)
			{
				TIM2_IRQHandler();
			}
			return motor2.Time_Cost_Act; 
		case 3:
			while(motor3.running == 1)
			{
				TIM3_IRQHandler();
			}
			return motor3.Time_Cost_Act; 
	}
	return 0;
}

/*ÖØÐÂ³õÊ¼»¯µç»úÔËÐÐÊ±Ïà¹Ø²ÎÊý*/
void Motor_Reinitial(unsigned char MotorID)
{
	int i = 0; 
	MOTOR_CONTROL_S *pmotor = NULL;  
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;
	uint16_t *MotorTime23Table;
	uint16_t *MotorStep23Table;
	uint16_t *MotorTime13Table;
	uint16_t *MotorStep13Table;
	
	switch(MotorID)
	{
		case 1:
			pmotor =& motor1;  
		    MotorTimeTable = Motor1TimeTable;
			MotorStepTable = Motor1StepTable;
			MotorTime23Table = Motor1_23TimeTable;
			MotorStep23Table = Motor1_23StepTable;
			MotorTime13Table = Motor1_13TimeTable;
			MotorStep13Table = Motor1_13StepTable;
			break;
		case 2:
			pmotor =& motor2;  
		    MotorTimeTable = Motor2TimeTable;
			MotorStepTable = Motor2StepTable;
			MotorTime23Table = Motor2_23TimeTable;
			MotorStep23Table = Motor2_23StepTable;
			MotorTime13Table = Motor2_13TimeTable;
			MotorStep13Table = Motor2_13StepTable;
			break;
		case 3:
			pmotor =& motor3; 
		    MotorTimeTable = Motor3TimeTable;
			MotorStepTable = Motor3StepTable;
			MotorTime23Table = Motor3_23TimeTable;
			MotorStep23Table = Motor3_23StepTable;
			MotorTime13Table = Motor3_13TimeTable;
			MotorStep13Table = Motor3_13StepTable;
			break;
		default:
			return ;
	}					 
	pmotor->pulsecount = 0;
	pmotor->CurrentIndex = 0;
	pmotor->speedenbale = 0;
	
	pmotor->Counter_Table = MotorTimeTable;  		//Ö¸ÏòÆô¶¯Ê±£¬Ê±¼ä»ùÊý¼ÆÊý±í
    pmotor->Step_Table = MotorStepTable;  			//Ö¸ÏòÆô¶¯Ê±£¬Ã¿¸öÆµÂÊÂö³å¸öÊý±í
	pmotor->StartSteps = 0;                         //±ØÐëÇåÁã£¬ºóÃæÊÇÀÛ¼Ó£¬·ñÔò»á°ÑÇ°Ò»´ÎµÄ¼ÓÉÏ
	pmotor->StopSteps = 0;                          //Í¬ÉÏ
	for(i = 0; i < pmotor->StartTableLength; i++)
		pmotor->StartSteps += pmotor->Step_Table[i];
	for(i = 0; i < pmotor->StopTableLength; i++)
		pmotor->StopSteps += pmotor->Step_Table[i+pmotor->StartTableLength];
	if(pmotor->PulsesGiven<pmotor->StartSteps+pmotor->StopSteps){
		//Èç¹û¸ø¶¨µÄÔËÐÐ²½ÊýÐ¡Óà×î´óSÐÍÇúÏß£¬Ôò³¢ÊÔÑ¡Ôñ2/3SÐÍÇúÏß
		pmotor->Counter_Table = MotorTime23Table;  		
		pmotor->Step_Table = MotorStep23Table;  			
		pmotor->StartSteps = 0;                  
		pmotor->StopSteps = 0;                   
		for(i = 0; i < pmotor->StartTableLength; i++)
			pmotor->StartSteps += pmotor->Step_Table[i];
		for(i = 0; i < pmotor->StopTableLength; i++)
			pmotor->StopSteps += pmotor->Step_Table[i+pmotor->StartTableLength];
		if(pmotor->PulsesGiven<pmotor->StartSteps+pmotor->StopSteps){
			//Èç¹û¸ø¶¨µÄÔËÐÐ²½ÊýÐ¡Óà×î´óSÐÍÇúÏß£¬Ôò³¢ÊÔÑ¡Ôñ1/3SÐÍÇúÏß
			pmotor->Counter_Table = MotorTime23Table;  		
			pmotor->Step_Table = MotorStep23Table;  			
			pmotor->StartSteps = 0;                  
			pmotor->StopSteps = 0;                   
			for(i = 0; i < pmotor->StartTableLength; i++)
				pmotor->StartSteps += pmotor->Step_Table[i];
			for(i = 0;i < pmotor->StopTableLength; i++)
				pmotor->StopSteps += pmotor->Step_Table[i+pmotor->StartTableLength];
		}
	}
	
	pmotor->TIMx->ARR  = pmotor->Counter_Table[0]; //ÉèÖÃÖÜÆÚ
	pmotor->TIMx->CCR1 = pmotor->Counter_Table[0]>>1;       //ÉèÖÃÕ¼¿Õ±È
	pmotor->Time_Cost_Act = pmotor->TIMx->ARR;
	Get_TimeCost_ReverDot_S(MotorID);		 
}
 
/*¸ù¾ÝSÐÍÇúÏß²ÎÊý»ñÈ¡Ä³¸öÊ±¿ÌµÄÆµÂÊ*/
float GetFreAtTime(float fstart,float faa,float taa,float tua,float tra,float t)
{
	//¸ù¾Ý¹«Ê½¼ÆËã´Ó¿ªÊ¼µ½×î¸ßËÙ¹ý³åÖÐ£¬tÊ±¿ÌµÄ×ª¶¯ÆµÂÊ
	if(t>=0&&t<=taa){
	//¼Ó¼ÓËÙ½×¶Î
		return fstart+0.5*faa*t*t;
	}else if(taa<t&&t<=(taa+tua)){
	//ÔÈ¼ÓËÙ½×¶Î
		return fstart+0.5*faa*taa*taa+(t-taa)*faa*taa;
	}else if((taa+tua)<t&&t<=(taa+tua+tra)){
	//¼õ¼ÓËÙ½×¶Î
		return fstart+0.5*faa*taa*taa+(tua)*faa*taa+0.5*faa*taa*tra-0.5*faa*taa*(taa+tua+tra-t)*(taa+tua+tra-t)/(tra);
	}		
	return 0;
}
 
 /*¼ÆËãSÐÍÇúÏßËã·¨µÄÃ¿Ò»²½¶¨Ê±Æ÷ÖÜÆÚ¼°²½½øÊý*/
void CalcMotorPeriStep_CPF(float fstart,float faa,float taa,float tua,float tra,uint16_t MotorTimeTable[],uint16_t MotorStepTable[])
{
	int  i;
	float fi;

	for(i=0;i<STEP_AA;i++)
	{
		fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa/STEP_AA*i);
		MotorTimeTable[i]=F2TIME_PARA/fi;
		MotorStepTable[i]=fi*(taa/STEP_AA)/STEP_PARA;
	}
	for(i=STEP_AA;i<STEP_AA+STEP_UA;i++)
	{
		fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa+(tua/STEP_UA)*(i-STEP_AA));
		MotorTimeTable[i]=F2TIME_PARA/fi;
		MotorStepTable[i]=fi*(tua/STEP_UA)/STEP_PARA;
	}
	for(i=STEP_AA+STEP_UA;i<STEP_AA+STEP_UA+STEP_RA;i++)
	{
		fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa+tua+tra/STEP_RA*(i-STEP_AA-STEP_UA));
		MotorTimeTable[i]=F2TIME_PARA/fi;
		MotorStepTable[i]=fi*(tra/STEP_RA)/STEP_PARA;
	}
	fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa+tua+tra);
	MotorTimeTable[STEP_AA+STEP_UA+STEP_RA]=F2TIME_PARA/fi;
	MotorStepTable[STEP_AA+STEP_UA+STEP_RA]=fi*(tra/STEP_RA)/STEP_PARA;

	for(i=STEP_AA+STEP_UA+STEP_RA+1;i<2*(STEP_AA+STEP_UA+STEP_RA)+1;i++)
	{ 
		MotorTimeTable[i]=MotorTimeTable[2*(STEP_AA+STEP_UA+STEP_RA)-i];
		MotorStepTable[i]=MotorStepTable[2*(STEP_AA+STEP_UA+STEP_RA)-i];
	}
}
 
/**************************************************************************************
µç»úÔËÐÐ²ÎÊý³õÊ¼»¯
**************************************************************************************/
void MotorRunParaInitial(void)
{ 
	/*FIXME:ÓÃ»§¿ÉÒÔ¸Ä±ä¸Ã²ÎÊýÊµÏÖSÐÍÇúÏßµÄÉý½µÌØÐÔ*/ 
	CalcMotorPeriStep_CPF(M_FRE_START,M_FRE_AA,M_T_AA,M_T_UA,M_T_RA,Motor1TimeTable,Motor1StepTable); 
	CalcMotorPeriStep_CPF(M_FRE_START,M_FRE_AA,M_T_AA,M_T_UA,M_T_RA,Motor2TimeTable,Motor2StepTable); 	
	CalcMotorPeriStep_CPF(M_FRE_START,M_FRE_AA,M_T_AA,M_T_UA,M_T_RA,Motor3TimeTable,Motor3StepTable);

	/*¸ü¸Ä²ÎÊý½µÎª2/3Éú³ÉµÄ±í¸ñ*/
	CalcMotorPeriStep_CPF(M_FRE_START*2.0/3,M_FRE_AA*2.0/3,M_T_AA*2.0/3,M_T_UA*2.0/3,M_T_RA*2.0/3,Motor1_23TimeTable,Motor1_23StepTable); 
	CalcMotorPeriStep_CPF(M_FRE_START*2.0/3,M_FRE_AA*2.0/3,M_T_AA*2.0/3,M_T_UA*2.0/3,M_T_RA*2.0/3,Motor2_23TimeTable,Motor2_23StepTable); 	
	CalcMotorPeriStep_CPF(M_FRE_START*2.0/3,M_FRE_AA*2.0/3,M_T_AA*2.0/3,M_T_UA*2.0/3,M_T_RA*2.0/3,Motor3_23TimeTable,Motor3_23StepTable); 

	/*¸ü¸Ä²ÎÊý½µÎª1/3Éú³ÉµÄ±í¸ñ*/
	CalcMotorPeriStep_CPF(M_FRE_START*1.0/3,M_FRE_AA*1.0/3,M_T_AA*1.0/3,M_T_UA*1.0/3,M_T_RA*1.0/3,Motor1_13TimeTable,Motor1_13StepTable); 
	CalcMotorPeriStep_CPF(M_FRE_START*1.0/3,M_FRE_AA*1.0/3,M_T_AA*1.0/3,M_T_UA*1.0/3,M_T_RA*1.0/3,Motor2_13TimeTable,Motor2_13StepTable); 	
	CalcMotorPeriStep_CPF(M_FRE_START*1.0/3,M_FRE_AA*1.0/3,M_T_AA*1.0/3,M_T_UA*1.0/3,M_T_RA*1.0/3,Motor3_13TimeTable,Motor3_13StepTable);  	
}

/**************************************************************************************
Á½¸öµç»úÍ¬Ê±ÔËÐÐÊ±£¬»¨·ÑÊ±¼ä½ÏÉÙµÄµç»úÒª¸ù¾ÝÊ±¼ä³¤µÄµç»úµ÷ÕûÔËÐÐ²ÎÊý
**************************************************************************************/
void Find_BestTimeCost(unsigned char ID,unsigned long long time_cost,unsigned char dir,unsigned int Degree)
{
	int   id = ID;
	float i = 0,j = 0;
	float fi = M_FRE_START, fj = M_FRE_START;
	int   cal_ij = 1;
	unsigned int PulsesGiven = 0;
	uint16_t *MotorTimeTable;
    uint16_t *MotorStepTable;
	MOTOR_CONTROL_S *pmotor = NULL;
	unsigned long long time_cost_tmp = 0;
	unsigned long long time_cost_min = 0xffffffffff;
	float i_o = 0, j_o = 0;
	float fi_o = 0, fj_o = 0;
	switch(ID)
	{
		 case 1:
			 pmotor =& motor1;
			 MotorTimeTable = pmotor->Counter_Table;
			 MotorStepTable = pmotor->Step_Table;
			 break;
		 case 2:
			 pmotor =& motor2;
			 MotorTimeTable = pmotor->Counter_Table;
			 MotorStepTable = pmotor->Step_Table;
			 break;
	}
	if(pmotor == NULL)
	{
		return ;
	}
	j = M_FRE_AA;
	i = 0;
	while(1)
	{		
		if(cal_ij)
		{
			CalcMotorPeriStep_CPF(M_FRE_START,(i+j)/2.0,M_T_AA,M_T_UA,M_T_RA,MotorTimeTable,MotorStepTable);
		}
		else
		{
			CalcMotorPeriStep_CPF((fi+fj)/2,0,M_T_AA,M_T_UA,M_T_RA,MotorTimeTable,MotorStepTable);
		}
		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven*pmotor->divnum;
		//pmotor->PulsesGiven+=300;
		Motor_Reinitial(id);		
		//time_cost_tmp=Get_Time_Cost2(id);
		time_cost_tmp = pmotor->Time_Cost_Cal;
		if(time_cost_tmp<time_cost)
		{
			if(time_cost-time_cost_tmp<time_cost_min)
			{
				time_cost_min = time_cost-time_cost_tmp;
				i_o = i;
				j_o = j;
				fi_o= fi;
				fj_o= fj;
			}
		}
		else
		{
			if(time_cost_tmp-time_cost<time_cost_min)
			{
				time_cost_min = time_cost_tmp-time_cost;
				i_o = i;
				j_o = j;
				fi_o= fi;
				fj_o= fj;
			}
		}
		if(time_cost_tmp>=time_cost-32*4&&time_cost_tmp<=time_cost+32*4)
		{
			break ;
		}
		if(cal_ij)
		{
			if(j<0.1)
			{
				//ËµÃ÷¼´Ê¹ÊÇÊ¹ÓÃ×î»ù±¾µÄÆô¶¯ËÙ¶È¶¼ÎÞ·¨Í¬Ê±Í£Ö¹£¬ÔòÐÞ¸ÄÎª½µµÍËÙ¶ÈµÄÌÝÐÎ
				i_o = 0;
				j_o = 0;
				j = 0;
				i = 0;
				cal_ij = 0;
				fi = 0;
			}
		}
		if(cal_ij)
		{
			if((i>j&&i-j<0.02||(i<j)&&j-i<0.02))
			{
				break;
			}
			if(time_cost_tmp>time_cost)
			{
				i = (i+j)/2.0;
			}
			else
			{
				j = (i+j)/2.0; 
			}
		}
		else
		{
			if((fi>fj&&fi-fj<0.02||(fi<fj)&&fj-fi<0.02))
			{
				break;
			}
			if(time_cost_tmp>time_cost)
			{
				fi = (fi+fj)/2.0;
			}
			else
			{
				fj = (fi+fj)/2.0; 
			}
		}
	}
	CalcMotorPeriStep_CPF((fi_o+fj_o)/2,(i_o+j_o)/2.0,M_T_AA,M_T_UA,M_T_RA,MotorTimeTable,MotorStepTable);
}

/**************************************************************************************
Æô¶¯µç»ú°´ÕÕSÐÍÇúÏß²ÎÊýÔËÐÐ*/
void Start_Motor_S(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
	unsigned int PulsesGiven = 0;
	MOTOR_CONTROL_S *pmotor = NULL; 
	if(Degree == 0)
	{ 		  	 
		return;
	}
	switch(MotorID)
	{
		case 1:
			pmotor = &motor1; 
			if(0 == dir)
			{
				GPIO_SetBits(GPIOE, GPIO_Pin_9);
			}
			else
			{
				GPIO_ResetBits(GPIOE, GPIO_Pin_9);
			} 			
			break;
		case 2:
			pmotor =& motor2; 
			if(1 == dir)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_1);
			}
			else
			{
				GPIO_ResetBits(GPIOA, GPIO_Pin_1);  
			}	
			break;
		case 3:
			pmotor =& motor3; 
			if(0 == dir)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_7);
			}
			else
			{
				GPIO_ResetBits(GPIOA, GPIO_Pin_7);
			}	
			break;
		default:  return;
	}
	pmotor->en=1;
	pmotor->dir=dir;
	pmotor->running=1;
	pmotor->PulsesHaven=0;
	PulsesGiven=Degree;
	pmotor->Time_Cost_Act=0;
	pmotor->PulsesGiven=PulsesGiven*pmotor->divnum;
	Motor_Reinitial(MotorID);		
	pmotor->CurrentIndex=0;
	pmotor->speedenbale=0;
	pmotor->TIMx->ARR =pmotor->Counter_Table[0]; //ÉèÖÃÖÜÆÚ
	pmotor->TIMx->CCR1 =pmotor->Counter_Table[0]>>1;       //ÉèÖÃÕ¼¿Õ±È
	TIM_Cmd(pmotor->TIMx, ENABLE);		  //DISABLE
}

/*Æô¶¯µç»ú°´ÕÕSPTA·½Ê½ÔËÐÐ*/
void Start_Motor_SPTA(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
	unsigned int PulsesGiven=0;
	MOTOR_CONTROL_SPTA *pmotor=NULL; 
	if(Degree==0)
	{ 		  	 
		return;
	}
	switch(MotorID)
	{
		case 4:
			pmotor=&motor4; 
			if(0==dir)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_9);
			}
			else
			{
				GPIO_ResetBits(GPIOB,GPIO_Pin_9);
			}			
			break;		
		default:
			return;
	}

	pmotor->en=1;
	pmotor->dir=dir;
	pmotor->running=1;
	pmotor->speedenbale=0;
	PulsesGiven=Degree;
	pmotor->step_move=PulsesGiven*pmotor->divnum;
	pmotor->step_middle=pmotor->step_move>>1;
	/*FIXME:ÕâÁ½¸ö²ÎÊý¿ÉÒÔÓÉÓÃ»§×ÔÐÐ¸Ä±ä²âÊÔ*/
	pmotor->step_spmax=MAXSPEED_SPTA;
	pmotor->step_accel=ACCSPEED_SPTA;
	pmotor->step_state=ACCELERATING;
	pmotor->step_frac=0;
	pmotor->speed_frac=0;
	pmotor->step_acced=0;
	pmotor->step_speed=0;
	pmotor->step_count=0;
		
	TIM_Cmd(pmotor->TIMx, ENABLE);	
}

/*Æô¶¯µç»ú£¬¸ù¾Ýµç»úºÅ¾ö¶¨µ÷ÓÃÄÄ¸ö*/
void Start_Motor(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
	if(MotorID>3){
		Start_Motor_S(MotorID,dir,Degree);
	}else{
		Start_Motor_SPTA(MotorID,dir,Degree);
	}
}

/*ÖØÐÂ¶¨Î»£¬ÈÃÕýÔÚÔËÐÐµÄµç»úÔËÐÐµ½Ö¸¶¨Î»ÖÃ*/
void Reposition_Motor(unsigned char MotorID,unsigned int NewPos)
{
	MOTOR_CONTROL_S *pmotor_s=NULL;
	MOTOR_CONTROL_SPTA *pmotor_spta=NULL; 
	switch(MotorID)
	{
		case 1:
			pmotor_s =& motor1;
			break;
		case 2:
			pmotor_s =& motor2;
			break;
		case 3:
			pmotor_s =& motor3;
			break;
		case 4:
			pmotor_spta =& motor4;
			break;
		default:
			return;
	} 
	if(pmotor_s!=NULL){
		if(NewPos<=pmotor_s->MaxPosition&&NewPos!=pmotor_s->CurrentPosition)
		{
			if(NewPos>pmotor_s->CurrentPosition)
			{
				Start_Motor_S(MotorID,pmotor_s->clockwise,NewPos-pmotor_s->CurrentPosition); 
			}
			else
			{
				Start_Motor_S(MotorID,!pmotor_s->clockwise,pmotor_s->CurrentPosition-NewPos);  	
			}		 		 
			while(pmotor_s->running==1);
		}
	}
	if(pmotor_spta!=NULL){
		if(NewPos<=pmotor_spta->MaxPosition&&NewPos!=pmotor_spta->CurrentPosition)
		{
			if(NewPos>pmotor_spta->CurrentPosition)
			{
				Start_Motor_SPTA(MotorID,pmotor_spta->clockwise,NewPos-pmotor_spta->CurrentPosition); 
			}
			else
			{
				Start_Motor_SPTA(MotorID,!pmotor_spta->clockwise,pmotor_spta->CurrentPosition-NewPos);  	
			}		 		 
			while(pmotor_s->running==1);
		}
	}
}
/*Í¬Ê±Æô¶¯µç»ú12*/
void Start_Motor12(unsigned char dir1,unsigned int Degree1,unsigned char dir2,unsigned int Degree2)
{
	unsigned int PulsesGiven=0;
	unsigned long long time_cost=0;
	if(Degree1==0||Degree2==0)
	{ 		  	 
		return;
	}	
	MotorRunParaInitial();
	Motor_Reinitial(1);		
	Motor_Reinitial(2);		
	if(Degree1<=Degree2)
	{
		motor2.en=1;
		motor2.dir=dir1;
		motor2.running=1;
		motor2.PulsesHaven=0;
		PulsesGiven=Degree2;
		motor2.Time_Cost_Act=0;
		motor2.PulsesGiven=PulsesGiven*motor2.divnum;
		time_cost=Get_TimeCost_ReverDot_S(2);
		Find_BestTimeCost(1,time_cost,dir1,Degree1);
	}
	else
	{
		motor1.en=1;
		motor1.dir=dir2;
		motor1.running=1;
		motor1.PulsesHaven=0;
		PulsesGiven=Degree1;
		motor1.Time_Cost_Act=0;
		motor1.PulsesGiven=PulsesGiven*motor1.divnum;		 
		time_cost=Get_TimeCost_ReverDot_S(1);
		Find_BestTimeCost(2,time_cost,dir2,Degree2);
	}
	
    Start_Motor_S(1,dir1,Degree1);
	Start_Motor_S(2,dir2,Degree2);
}

/*ÉèÖÃµç»úÔËÐÐËÙ¶È£¬Èë¿Ú²ÎÊýÊÇËÙ¶ÈµÈ¼¶*/
void SetSpeed(unsigned char MotorID,signed char speedindex)
{
	int currentindex,i;
	unsigned int destspeed;
	unsigned int stepstostop=0;
	MOTOR_CONTROL_S *pmotor_s=NULL; 
	MOTOR_CONTROL_SPTA *pmotor_spta=NULL; 
	switch(MotorID)
	{
		case 1:
			pmotor_s=&motor1; 
			break;
		case 2:
			pmotor_s=&motor2; 
			break;
		case 3:
			pmotor_s=&motor3; 
			break;
		case 4:
			pmotor_spta=&motor4; 
			break;
		default:
			return;
	}
	if(pmotor_s!=NULL){
		TIM_Cmd(pmotor_s->TIMx, DISABLE);
		if(speedindex>=0&&speedindex<=STEP_AA+STEP_UA+STEP_RA)
		{		
			//Ö±½ÓÏòÏÂÒ»ËÙ¶È
			currentindex=pmotor_s->CurrentIndex;
			pmotor_s->PulsesHaven=0;
			if(pmotor_s->CurrentIndex>=pmotor_s->StartTableLength )
			{
				currentindex=pmotor_s->StartTableLength+pmotor_s->StopTableLength-pmotor_s->CurrentIndex-1;  
			}
			if(currentindex>speedindex)
			{
				//ÐèÒª¼õËÙ
				pmotor_s->PulsesGiven=pmotor_s->PulsesHaven+pmotor_s->StopSteps-2;
			}
			else
			{
				//ÐèÒª¼ÓËÙ
				pmotor_s->PulsesGiven=0xffffffff;
			}
			pmotor_s->CurrentIndex=currentindex;
			pmotor_s->pulsecount=pmotor_s->Step_Table[pmotor_s->CurrentIndex];
			pmotor_s->TargetIndex=speedindex;
			pmotor_s->speedenbale=1;
			pmotor_s->running=1;
			pmotor_s->en=1;
		}
		else
		{
			//Í£Ö¹µç»ú,pmotor->CurrentIndex=currentindex-1;Ö±½ÓÏòÏÂÒ»¼õËÙ
			pmotor_s->speedenbale=0;
			currentindex=pmotor_s->CurrentIndex; 
			if(pmotor_s->CurrentIndex>=pmotor_s->StartTableLength )
			{
				currentindex=pmotor_s->StartTableLength+pmotor_s->StopTableLength-pmotor_s->CurrentIndex-1;  
			}
			if(1)//currentindex>=1)
			{
				for(i=0;i<currentindex;i++)
				{
					stepstostop+=pmotor_s->Step_Table[i];
				}
				/*½øÈë¼õËÙindex*/
				if(pmotor_s->CurrentIndex<pmotor_s->StartTableLength )
				{
					currentindex=pmotor_s->StartTableLength+pmotor_s->StopTableLength-pmotor_s->CurrentIndex-1;  
				}
				pmotor_s->CurrentIndex=currentindex;
				pmotor_s->pulsecount=pmotor_s->Step_Table[pmotor_s->CurrentIndex];
				pmotor_s->PulsesHaven=0;
				pmotor_s->PulsesGiven=pmotor_s->PulsesHaven+stepstostop;
			}
			else
			{
				pmotor_s->PulsesGiven=pmotor_s->PulsesHaven;
			}
			if(stepstostop==0)
			{
				//ÒÑ¾­Í£Ö¹
				return;
			}
		}
		TIM_Cmd(pmotor_s->TIMx, ENABLE);
	}
	
	if(pmotor_spta!=NULL){
		TIM_Cmd(pmotor_spta->TIMx, DISABLE);
		if(speedindex>0&&speedindex<=STEP_SPTA)
		{		
			destspeed=speedindex*MAXSPEED_SPTA/STEP_SPTA;
			if(destspeed==pmotor_spta->step_spmax)
			{
				TIM_Cmd(pmotor_spta->TIMx, ENABLE);
				return;
			}
			if(pmotor_spta->step_state==IDLE)
			{			
				/*Èç¹ûµç»úÒÑ¾­Í£Ö¹£¬ÄÇÃ´µç»úµÄÔËÐÐ²ÎÊý¾ÍÒª¸´Î»£¬·ñÔò²ÎÊý²»±ä
				ÔÚÏÖÓÐµÄ»ù´¡ÉÏ×öËÙ¶Èµ÷Õû*/
				pmotor_spta->step_frac=0;
				pmotor_spta->speed_frac=0;
				pmotor_spta->step_acced=0;
				pmotor_spta->step_speed=0;
				pmotor_spta->step_count=0;
			}
			if(destspeed<pmotor_spta->step_spmax)
			{
				if(pmotor_spta->step_state!=IDLE)
				{
					pmotor_spta->step_state=DECELERATING;
				}else{
					pmotor_spta->step_state=ACCELERATING;
				}
			}
			else
			{
				pmotor_spta->step_state=ACCELERATING;
			}
			
			pmotor_spta->step_accel=ACCSPEED_SPTA;
			pmotor_spta->step_spmax=destspeed;
			pmotor_spta->speedenbale=1;
			pmotor_spta->running=1;
			pmotor_spta->en=1;
		}
		else
		{			
			//Í£Ö¹µç»ú, 
			if(pmotor_spta->running==0)
			{
				//ÒÑ¾­Í£Ö¹
				return;
			}
			pmotor_spta->speedenbale=0;
			pmotor_spta->step_state=DECELERATING;
			pmotor_spta->step_move=pmotor_spta->step_count+pmotor_spta->step_acced;
		}
		TIM_Cmd(pmotor_spta->TIMx, ENABLE);
	}
}

/*ÉèÖÃµç»úµÄÎ»ÖÃ£¬µç»úÔËÐÐµ½Ö¸¶¨µÄÎ»ÖÃ*/
void SetPosition(unsigned char MotorID,unsigned int dest)
{ 
	MOTOR_CONTROL_S *pmotor_s=NULL; 
	MOTOR_CONTROL_SPTA *pmotor_spta=NULL; 
	switch(MotorID)
	{
		case 1:
			pmotor_s=&motor1; 
			break;
		case 2:
			pmotor_s=&motor2; 
			break;
		case 3:
			pmotor_s=&motor3; 
			break;
		case 4:
			pmotor_spta=&motor4; 
			break;
		default:
			return;
	}
	if(pmotor_s!=NULL){
		if(dest<=pmotor_s->MaxPosition&&dest!=pmotor_s->CurrentPosition)
		{
			if(dest>pmotor_s->CurrentPosition)
			{
				Start_Motor_S(MotorID,pmotor_s->clockwise,dest-pmotor_s->CurrentPosition); 
			}
			else
			{
				Start_Motor_S(MotorID,!pmotor_s->clockwise,pmotor_s->CurrentPosition-dest);  	
			}		 		 
			while(pmotor_s->running==1);
		}
	}
	else if(pmotor_spta!=NULL)
	{
		if(dest<=pmotor_spta->MaxPosition&&dest!=pmotor_spta->CurrentPosition)
		{
			if(dest>pmotor_spta->CurrentPosition)
			{
				Start_Motor_SPTA(MotorID,pmotor_spta->clockwise,dest-pmotor_spta->CurrentPosition); 
			}
			else
			{
				Start_Motor_SPTA(MotorID,!pmotor_spta->clockwise,pmotor_spta->CurrentPosition-dest);  	
			}		 		 
			while(pmotor_spta->running==1);
		}
	}
}

/*¸´Î»µç»ú*/
void Do_Reset(unsigned char MotorID)
{
	MOTOR_CONTROL_S *pmotor_s=NULL;
	MOTOR_CONTROL_SPTA *pmotor_spta=NULL;
	switch(MotorID)
	{
		case 1:
			pmotor_s=&motor1;
			break;
		case 2:
			pmotor_s=&motor2;
			break;
		case 3:
			pmotor_s=&motor3;
			break;
		case 4:
			pmotor_spta=&motor4;
			break;
		default:
			return;
	} 
	/*do reset*/
	if(pmotor_s!=NULL){
		pmotor_s->rstflg=1;
		pmotor_s->running=1;
		SetSpeed(MotorID,16);
		while(pmotor_s->running==1);
	}
	if(pmotor_spta!=NULL){
		pmotor_spta->rstflg=1;
		pmotor_spta->running=1;
		SetSpeed(MotorID,16);
		while(pmotor_spta->running==1);
	}
}


void Deal_Serail_Cmd(char *buf,int len)
{
    unsigned int dest=0;  
	/*
	0xAA 0x55(ÆðÊ¼±êÖ¾£©
	Data0(µç»úºÅ£¬1,2,3,4)
	Data1(0,¸´Î»£¬1£¬ÉèÖÃËÙ¶È£¬2£¬Í£Ö¹µç»ú£¬3 £¬ÈÃµç»úÔËÐÐµ½Ö¸¶¨Î»ÖÃ£
	Data2£¨ÔÚdata1=1Ê±ºò£¬¸Ã×Ö½Ú±íÊ¾ËÙ¶ÈµÈ¼¶£¬ÔÚdata1=3Ê±£¬¸Ã×Ö½Ú±íÊ¾Î»ÖÃµÄ¸ß8Î»
	Data3£¨ÔÚdata1=3Ê±£¬¸Ã×Ö½Ú±íÊ¾Î»ÖÃµÄÖÐ8Î» 
	Data4£¨ÔÚdata1=3Ê±£¬¸Ã×Ö½Ú±íÊ¾Î»ÖÃµÄµÍ8Î» 
	*/
	if(buf[0] == 0xAA)
	{
		if(buf[3]==0)
		{
			Do_Reset(buf[2]);
		}
		else if(buf[3]==1)
		{
			SetSpeed(buf[2],buf[4]);
		}
		else if(buf[3]==2)
		{
			SetSpeed(buf[2],-1);
		}
		else if(buf[3]==3)
		{
			dest=(buf[4]<<16)+(buf[5]<<8)+(buf[6]);
			SetPosition(buf[2],dest);
		}
	}
}

static unsigned char MY_Cmd_Buff[16];
void Deal_Cmd(void)
{  
	unsigned char sync[2]; 
	while(rt_ringbuffer_data_len(&rb_recv)>=7)
	{ 
		if(rt_ringbuffer_peek(&rb_recv,sync,2)==2)
		{
			if(sync[0]==0xaa&&sync[1]==0x55)
			{
				if(rt_ringbuffer_get(&rb_recv,MY_Cmd_Buff,7)==7)
				{
					Deal_Serail_Cmd(MY_Cmd_Buff,7); 
				}	
			}
			else if(sync[1]==0xaa)
			{
				rt_ringbuffer_getchar(&rb_recv,sync);
			}
			else
			{
				rt_ringbuffer_getchar(&rb_recv,sync);
				rt_ringbuffer_getchar(&rb_recv,sync);
			}
		}
	} 
}

/*µç»ú1µÄPWMÊä³ö³õÊ¼»¯£¬Ê¹ÓÃµÄÊÇ¶¨Ê±Æ÷4*/
void Initial_PWM_Motor1(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_DeInit(TIM1);
	 //ÖÐ¶ÏNVICÉèÖÃ£ºÔÊÐíÖÐ¶Ï£¬ÉèÖÃÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;     
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =PWM1_PreemptionPriority;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PWM1_SubPriority;          
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             
	NVIC_Init(&NVIC_InitStructure);
	TIM_BaseInitStructure.TIM_Period =1000;
#ifdef OUTPUT_DATA
	TIM_BaseInitStructure.TIM_Prescaler =710000; //71000
#else
	TIM_BaseInitStructure.TIM_Prescaler =5; 
#endif
	TIM_BaseInitStructure.TIM_ClockDivision = 0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_BaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;       //PWM2Ä£Ê½
	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;  //ÐÅºÅÊä³öµ½¶ÔÓ¦µÄÊä³öÒý½Å
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //»¥²¹ÐÅºÅÊä³öµ½¶ÔÓ¦µÄÊä³öÒý½Å
	TIM_OCInitStructure.TIM_Pulse =50;   //Âö³å¿í¶È
	TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;   //»¥²¹Êä³ö¸ßµçÆ½ÓÐÐ§
	TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;    //»¥²¹Êä³ö¸ßµçÆ½ÓÐÐ§
	TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;  //Êä³ö¿ÕÏÐ×´Ì¬Îª1
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;   //»¥²¹Êä³ö¿ÕÏÐ×´Ì¬Îª0
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);   //OC1Í¨µÀ³õÊ¼»¯

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	//ÇåÖÐ¶Ï£¬ÒÔÃâÒ»ÆôÓÃÖÐ¶ÏºóÁ¢¼´²úÉúÖÐ¶Ï
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	//Ê¹ÄÜTIM1ÖÐ¶ÏÔ´
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE); //Ê¹ÄÜPWMÊä³ö
	
}

void Initial_PWM_Motor2(void)
{
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_DeInit(TIM2);
     //ÖÐ¶ÏNVICÉèÖÃ£ºÔÊÐíÖÐ¶Ï£¬ÉèÖÃÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;    //¸üÐÂÊÂ¼þ 	TIM2_IRQHandler
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =PWM2_PreemptionPriority;   //ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PWM2_SubPriority;          //ÏìÓ¦ÓÅÏÈ¼¶1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //ÔÊÐíÖÐ¶Ï
	NVIC_Init(&NVIC_InitStructure);
	TIM_BaseInitStructure.TIM_Period =1000;
#ifdef OUTPUT_DATA
	TIM_BaseInitStructure.TIM_Prescaler =710000; //71000
#else
	TIM_BaseInitStructure.TIM_Prescaler =5; 
#endif 
	TIM_BaseInitStructure.TIM_ClockDivision = 0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure);	
	
	TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;       //PWM2Ä£Ê½ 
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;  //ÐÅºÅÊä³öµ½¶ÔÓ¦µÄÊä³öÒý½Å 
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //»¥²¹ÐÅºÅÊä³öµ½¶ÔÓ¦µÄÊä³öÒý½Å                  
    TIM_OCInitStructure.TIM_Pulse =50;   //Âö³å¿í¶È 
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;   //»¥²¹Êä³ö¸ßµçÆ½ÓÐÐ§ 
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;    //»¥²¹Êä³ö¸ßµçÆ½ÓÐÐ§      
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;  //Êä³ö¿ÕÏÐ×´Ì¬Îª1 
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;   //»¥²¹Êä³ö¿ÕÏÐ×´Ì¬Îª0   
    TIM_OC1Init(TIM2,&TIM_OCInitStructure);   //OC1Í¨µÀ³õÊ¼»¯
	
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
	//ÇåÖÐ¶Ï£¬ÒÔÃâÒ»ÆôÓÃÖÐ¶ÏºóÁ¢¼´²úÉúÖÐ¶Ï
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    //Ê¹ÄÜTIM1ÖÐ¶ÏÔ´
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM2, DISABLE);
    //TIM_CtrlPWMOutputs(TIM2,ENABLE); //Ê¹ÄÜPWMÊä³ö
}

void Initial_PWM_Motor3(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_DeInit(TIM3);
     //ÖÐ¶ÏNVICÉèÖÃ£ºÔÊÐíÖÐ¶Ï£¬ÉèÖÃÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;    //¸üÐÂÊÂ¼þ 	TIM3_IRQHandler
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =PWM3_PreemptionPriority;   //ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PWM3_SubPriority;          //ÏìÓ¦ÓÅÏÈ¼¶1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //ÔÊÐíÖÐ¶Ï
	NVIC_Init(&NVIC_InitStructure);
	TIM_BaseInitStructure.TIM_Period =1000;
#ifdef OUTPUT_DATA
	TIM_BaseInitStructure.TIM_Prescaler =710000; //71000
#else
	TIM_BaseInitStructure.TIM_Prescaler =5; 
#endif 
	TIM_BaseInitStructure.TIM_ClockDivision = 0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_BaseInitStructure);	
	
	TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;       //PWM2Ä£Ê½ 
	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;  //ÐÅºÅÊä³öµ½¶ÔÓ¦µÄÊä³öÒý½Å 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //»¥²¹ÐÅºÅÊä³öµ½¶ÔÓ¦µÄÊä³öÒý½Å                  
	TIM_OCInitStructure.TIM_Pulse =50;   //Âö³å¿í¶È 
	TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;   //»¥²¹Êä³ö¸ßµçÆ½ÓÐÐ§ 
	TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;    //»¥²¹Êä³ö¸ßµçÆ½ÓÐÐ§      
	TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;  //Êä³ö¿ÕÏÐ×´Ì¬Îª1 
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;   //»¥²¹Êä³ö¿ÕÏÐ×´Ì¬Îª0   
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);   //OC1Í¨µÀ³õÊ¼»¯

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	//ÇåÖÐ¶Ï£¬ÒÔÃâÒ»ÆôÓÃÖÐ¶ÏºóÁ¢¼´²úÉúÖÐ¶Ï
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	//Ê¹ÄÜTIM1ÖÐ¶ÏÔ´
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
	TIM_Cmd(TIM3, DISABLE);
	//TIM_CtrlPWMOutputs(TIM3,ENABLE); //Ê¹ÄÜPWMÊä³ö
}

void Initial_PWM_Motor4(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		//ÖÐ¶ÏNVICÉèÖÃ£ºÔÊÐíÖÐ¶Ï£¬ÉèÖÃÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                 //¸üÐÂÊÂ¼þ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =PWM4_PreemptionPriority;        //ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =PWM4_SubPriority;              //ÏìÓ¦ÓÅÏÈ¼¶1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ÔÊÐíÖÐ¶Ï
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = 29;                        //×Ô¶¯×°ÔØ¼ÆÊýÆ÷ÖÜÆÚÖµ
#ifdef OUTPUT_DATA
	TIM_TimeBaseStructure.TIM_Prescaler = 2800; //2800                    //Ô¤·ÖÆµÖµ	 29
#else
	TIM_TimeBaseStructure.TIM_Prescaler = 14; //2800                    //Ô¤·ÖÆµÖµ	 29
#endif
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  //Ê±ÖÓ·Ö¸ô
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //¼ÆÊýÄ£Ê½
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);               //³õÊ¼»¯
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, DISABLE);
}

void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;        //EXTI³õÊ¼»¯½á¹¹¶¨Òå
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);	 //E0-E5  Ê±ÕëÊ¹ÄÜ

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4;//Ñ¡ÔñÒý½Å
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	EXTI_ClearITPendingBit(EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line4);//Çå³ýÖÐ¶Ï±êÖ¾
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//ÊÂ¼þÑ¡Ôñ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//´¥·¢Ä£Ê½
	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line4; //ÏßÂ·Ñ¡Ôñ 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//Æô¶¯ÖÐ¶Ï
	EXTI_Init(&EXTI_InitStructure);//³õÊ¼»¯		
}
