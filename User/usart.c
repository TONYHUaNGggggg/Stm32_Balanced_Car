
#include "usart.h"
#include <stdarg.h>
#include "misc.h"

uint8_t Uart_Rx[UART_RX_LEN] = {0};

void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	






	  
	
	USART_InitStructure.USART_BaudRate = 115200;
	
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_Cmd(USART1, ENABLE);
}


void USART3_Config(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB	, ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  	
	
	NVIC_Configurationusart3(); 
	
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  

	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART3, ENABLE);
	
}




void NVIC_Configurationusart3(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



int fputc(int ch, FILE *f)
{
	
	USART_SendData(USART1, (unsigned char) ch);

	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	
	return (ch);
}


static char *itoa(int value, char *string, int radix)
{
	int     i, d;
	int     flag = 0;
	char    *ptr = string;
	
	
	if (radix != 10)
	{
	    *ptr = 0;
	    return string;
	}
	
	if (!value)
	{
	    *ptr++ = 0x30;
	    *ptr = 0;
	    return string;
	}
	
	
	if (value < 0)
	{
	    *ptr++ = '-';
	
	    
	    value *= -1;
	}
	
	for (i = 10000; i > 0; i /= 10)
	{
	    d = value / i;
	
	    if (d || flag)
	    {
	        *ptr++ = (char)(d + 0x30);
	        value -= (d * i);
	        flag = 1;
	    }
	}
	
	
	*ptr = 0;
	
	return string;

} 


 
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	
	while ( *Data != 0)     
	{				                          
		if ( *Data == 0x5c )  
	{									  
	switch ( *++Data )
	{
		case 'r':							          
			USART_SendData(USARTx, 0x0d);
			Data ++;
		break;
		
		case 'n':							          
			USART_SendData(USARTx, 0x0a);	
			Data ++;
		break;
		
		default:
			Data ++;
		break;
	}			 
	}
	else if ( *Data == '%')
	{									  
	switch ( *++Data )
	{				
		case 's':										  
			s = va_arg(ap, const char *);
	for ( ; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
		Data++;
		break;
	
	case 'd':										
	d = va_arg(ap, int);
	itoa(d, buf, 10);
	for (s = buf; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
	Data++;
	break;
		 default:
				Data++;
		    break;
	}		 
	} 
	else USART_SendData(USARTx, *Data++);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}

void USART1_Send_Byte(unsigned char byte)   
{
        USART_SendData(USART1, byte);        
        while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  
        
        
}


void USART3_Send_Byte(unsigned char byte)   
{
        USART_SendData(USART3, byte);        
        while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);  
        
        
}
void UART3_Send_Char(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		USART3_Send_Byte(*p);
		p++;
	}	
}


void PrintChar(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		USART1_Send_Byte(*p);
		p++;
	}
}

