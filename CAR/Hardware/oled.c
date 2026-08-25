#include "oled.h"
#include "stdlib.h"
#include "stdio.h"   /* sprintf (OLED_ShowAngle6 用) */
#include "oledfont.h"

u8 OLED_GRAM[144][8];
extern void delay_ms(uint32_t ms);

/* ============================================================
 * 4 线 SPI 软件 GPIO 底层 (SSD1306)
 *   SCL = PB9  时钟
 *   SDA = PB8  数据 (MOSI)
 *   RES = PB10 复位
 *   DC  = PB11 数据/命令选择 (0=命令, 1=数据)
 * 引脚由 syscfg 生成 (GPIO 实例名 OLED), 在 SYSCFG_DL_GPIO_init()
 * 里已完成 initDigitalOutput + enableOutput, 此处只操作电平。
 * SSD1306 4-wire SPI: 上升沿锁存, MSB first。
 * ============================================================ */

/* DC 选择: 0 = 写命令, 1 = 写数据 */
#define OLED_DC_CMD()  DL_GPIO_clearPins(OLED_PORT,  OLED_DC_PIN)
#define OLED_DC_DATA() DL_GPIO_setPins(OLED_PORT,    OLED_DC_PIN)

/* SCL 脉冲: 先拉低再拉高, 产生上升沿让 SSD1306 锁存 SDA */
#define OLED_SCL_LOW()  DL_GPIO_clearPins(OLED_PORT, OLED_SCL_PIN)
#define OLED_SCL_HIGH() DL_GPIO_setPins(OLED_PORT,   OLED_SCL_PIN)

/* SDA 输出一位 */
#define OLED_SDA_LOW()  DL_GPIO_clearPins(OLED_PORT, OLED_SDA_PIN)
#define OLED_SDA_HIGH() DL_GPIO_setPins(OLED_PORT,   OLED_SDA_PIN)

/**
 * @brief 4 线 SPI 复位时序
 * @note  RES 拉低 >=3us 后拉高, 再等十几 ms 让 SSD1306 内部完成复位。
 *        4 线屏有独立 RES 引脚, 比 4 针 I2C 屏(靠上电复位)更可靠。
 */
static void OLED_Reset(void)
{
    DL_GPIO_clearPins(OLED_PORT, OLED_RES_PIN);   /* RES = 0 进入复位 */
    delay_ms(10);
    DL_GPIO_setPins(OLED_PORT,   OLED_RES_PIN);   /* RES = 1 退出复位 */
    delay_ms(10);
}

/**
 * @brief 向 OLED 写入一个字节的数据或命令 (4 线 SPI 软件 IO)
 *
 * @param dat  要写入的 8 位内容 (命令或显示数据)
 * @param mode 0 = 写命令 (DC=0), 1 = 写数据 (DC=1)
 *
 * @note 时序: 先设 DC, 再 MSB first 逐位送 SDA, 每个 SCL 上升沿锁存。
 *       不依赖任何 I2C 外设, 纯 GPIO 翻转, 可移植性最好。
 */
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t i;

    /* 1. DC 选择命令/数据 */
    if (mode) OLED_DC_DATA();
    else      OLED_DC_CMD();

    /* 2. SCL 预置低, 准备发字节 */
    OLED_SCL_LOW();

    /* 3. MSB 先发, 8 个上升沿逐位锁存 */
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80) OLED_SDA_HIGH();
        else            OLED_SDA_LOW();
        dat <<= 1;

        OLED_SCL_HIGH();   /* 上升沿: SSD1306 采样 SDA */
        OLED_SCL_LOW();    /* 拉低, 为下一位做准备 */
    }
}

//反显函数
void OLED_ColorTurn(u8 i)
{
	if(i==0) OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
	if(i==1) OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
}

//屏幕旋转180度
void OLED_DisplayTurn(u8 i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
		OLED_WR_Byte(0xA0,OLED_CMD);
	}
}

//开启OLED显示
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x14,OLED_CMD);//开启电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//点亮屏幕
}

//关闭OLED显示 
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x10,OLED_CMD);//关闭电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//关闭屏幕
}

//更新显存到OLED	
void OLED_Refresh(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
	   OLED_WR_Byte(0xb0+i,OLED_CMD); //设置行起始地址
	   OLED_WR_Byte(0x00,OLED_CMD);   //设置低列起始地址
	   OLED_WR_Byte(0x10,OLED_CMD);   //设置高列起始地址
	   for(n=0;n<128;n++)
		 OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

//清屏函数
void OLED_Clear(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
	   for(n=0;n<128;n++)
		{
			 OLED_GRAM[n][i]=0;//清除所有数据
		}
	}
	OLED_Refresh();//更新显示
}

//画点 
void OLED_DrawPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]|=n;
}

//清除一个点
void OLED_ClearPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	OLED_GRAM[x][i]|=n;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
}

//画线
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2)
{
	u8 i,k,k1,k2;
	if((x1<0)||(x2>128)||(y1<0)||(y2>64)||(x1>x2)||(y1>y2))return;
	if(x1==x2)    //画竖线
	{
		for(i=0;i<(y2-y1);i++) OLED_DrawPoint(x1,y1+i);
	}
	else if(y1==y2)   //画横线
	{
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1);
	}
	else      //画斜线
	{
		k1=y2-y1;
		k2=x2-x1;
		k=k1*10/k2;
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1+i*k/10);
	}
}

//画圆
void OLED_DrawCircle(u8 x,u8 y,u8 r)
{
	int a = 0, b = r, num;
	while(2 * b * b >= r * r)      
	{
		OLED_DrawPoint(x + a, y - b);
		OLED_DrawPoint(x - a, y - b);
		OLED_DrawPoint(x - a, y + b);
		OLED_DrawPoint(x + a, y + b);
		OLED_DrawPoint(x + b, y + a);
		OLED_DrawPoint(x + b, y - a);
		OLED_DrawPoint(x - b, y - a);
		OLED_DrawPoint(x - b, y + a);
		
		a++;
		num = (a * a + b * b) - r*r;
		if(num > 0) { b--; a--; }
	}
}

//显示字符
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1)
{
	u8 i,m,temp,size2,chr1;
	u8 y0=y;
	size2=(size1/8+((size1%8)?1:0))*(size1/2);  
	chr1=chr-' ';  
	for(i=0;i<size2;i++)
	{
		if(size1==12) {temp=asc2_1206[chr1][i];} 
		else if(size1==16) {temp=asc2_1608[chr1][i];} 
		else if(size1==24) {temp=asc2_2412[chr1][i];} 
		else return;
		for(m=0;m<8;m++)           
		{
			if(temp&0x80)OLED_DrawPoint(x,y);
			else OLED_ClearPoint(x,y);
			temp<<=1;
			y++;
			if((y-y0)==size1)
			{
				y=y0;
				x++;
				break;
			}
		}
	}
}

//显示字符串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1)
{
	while((*chr>=' ')&&(*chr<='~'))
	{
		OLED_ShowChar(x,y,*chr,size1);
		x+=size1/2;
		if(x>128-size1)  //换行
		{
			x=0;
			y+=size1; // 修复了原代码的y+=2的bug
		}
		chr++;
	}
}

//m^n
u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--) result*=m;
	return result;
}

//显示数字
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1)
{
	u8 t,temp;
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(temp==0) OLED_ShowChar(x+(size1/2)*t,y,'0',size1);
		else OLED_ShowChar(x+(size1/2)*t,y,temp+'0',size1);
	}
}

//显示汉字
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1)
{
	u8 i,m,n=0,temp,chr1;
	u8 x0=x,y0=y;
	u8 size3=size1/8;
	while(size3--)
	{
		chr1=num*size1/8+n;
		n++;
		for(i=0;i<size1;i++)
		{
			if(size1==16) {temp=Hzk1[chr1][i];}
			else if(size1==24) {temp=Hzk2[chr1][i];}
			else if(size1==32) {temp=Hzk3[chr1][i];}
			else if(size1==64) {temp=Hzk4[chr1][i];}
			else return;
						
			for(m=0;m<8;m++)
			{
				if(temp&0x01)OLED_DrawPoint(x,y);
				else OLED_ClearPoint(x,y);
				temp>>=1;
				y++;
			}
			x++;
			if((x-x0)==size1) {x=x0;y0=y0+8;}
			y=y0;
		}
	}
}

//配置写入数据的起始位置
void OLED_WR_BP(u8 x,u8 y)
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);//设置行起始地址
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}

//显示图片
void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[])
{
	u32 j=0;
	u8 x=0,y=0;
	if(y%8==0)y=0;
	else y+=1;
	for(y=y0;y<y1;y++)
	{
		 OLED_WR_BP(x0,y);
		 for(x=x0;x<x1;x++)
		 {
			 OLED_WR_Byte(BMP[j],OLED_DATA);
			 j++;
		 }
	}
}

//OLED的初始化
void OLED_Init(void)
{
	// 4 线 SPI 屏有独立 RES 引脚 (PB10), 用硬件复位比靠上电 RC 复位更可靠
	OLED_Reset();

	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD);// Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,OLED_CMD);
	OLED_Clear();
}

/**
 * @brief 显示六轴陀螺仪三个角度 (Roll/Pitch/Yaw)
 *
 * @param roll  横滚角 deg  (-180~+180)
 * @param pitch 俯仰角 deg  (-180~+180)
 * @param yaw   航向角 deg  (-180~+180)
 * @note  三行 16 字号显示: R:+045.2 / P:-012.3 / Y:+179.8
 *        内部 Clear + Refresh, 调用方负责节拍 (建议 100ms 一次, I2C 慢)
 */
void OLED_ShowAngle6(float roll, float pitch, float yaw)
{
    /* 角度折回 [-180,180], 避免陀螺仪漂出范围显示溢出 */
    while (roll  >  180.0f) roll  -= 360.0f;
    while (roll  < -180.0f) roll  += 360.0f;
    while (pitch >  180.0f) pitch -= 360.0f;
    while (pitch < -180.0f) pitch += 360.0f;
    while (yaw   >  180.0f) yaw   -= 360.0f;
    while (yaw   < -180.0f) yaw   += 360.0f;

    char buf[16];
    OLED_Clear();
    /* R:+045.2  (符号+3位整数+小数点+1位小数 = 7字符, 16字号每字符8px宽=56px, 居左) */
    sprintf(buf, "R:%+07.1f", roll);
    OLED_ShowString(0, 0, (u8 *)buf, 16);
    sprintf(buf, "P:%+07.1f", pitch);
    OLED_ShowString(0, 16, (u8 *)buf, 16);
    sprintf(buf, "Y:%+07.1f", yaw);
    OLED_ShowString(0, 32, (u8 *)buf, 16);
    OLED_Refresh();
}
