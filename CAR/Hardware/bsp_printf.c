#include "bsp_printf.h"

//任意串口打印
char Ux_TxBuff[256];
void any_printf(UART_Regs *uart,char *format,...)
{	
	uint8_t i=0;      
	va_list listdata;                                
	va_start(listdata,format);                 
	vsprintf((char *)Ux_TxBuff,format,listdata); 
	va_end(listdata);          

    while( Ux_TxBuff[i]!='\0' )      
    {
		while( DL_UART_isBusy(uart) == true ){}
        DL_UART_transmitDataBlocking(uart,Ux_TxBuff[i++]);
    }  

}

//printf函数重定义
int fputc(int ch, FILE *stream)
{
	while( DL_UART_isBusy(UART_0_INST) == true );
	DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
	return ch;
}

int fputs(const char* restrict s,FILE* restrict stream)
{
   uint16_t i,len;
   len = strlen(s);
   for(i=0;i<len;i++)
   {
        while( DL_UART_isBusy(UART_0_INST) == true ){}
       DL_UART_Main_transmitDataBlocking(UART_0_INST,s[i]);
   }
   return len;
}

int puts(const char *_ptr)
{
    int count = fputs(_ptr,stdout);
    count += fputs("\n",stdout);
    return count;
}
