/*
	Copyright (c) [2019] [一只程序缘 jiezhuo]
	[https://gitee.com/jiezhuonew/oledlib] is licensed under the Mulan PSL v1.
	You can use this software according to the terms and conditions of the Mulan PSL v1.
	You may obtain a copy of Mulan PSL v1 at:
		http://license.coscl.org.cn/MulanPSL
	THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR
	PURPOSE.
	See the Mulan PSL v1 for more details.

	图形库原理:其实就是对一个数组进行操作,数组操作完成之后,直接将整个
	数组刷新到屏幕上
	因此此c文件用于配置oled底层 用于单片机与oled的直接且唯一通信

	移植此图形库主要完善单片机与OLED的通信:
	SPI_Configuration() 	配置通信引脚
	WriteCmd()      		写命令
	WriteDat()      		写数据
	OledTimeMsFunc()  oled_config中的函数 为系统提供时基
	此例程仅演示SPI通信方式 需要更改其他通信方式请在[oled_config.h]修改
*/



#include "oled_driver.h"

#if (TRANSFER_METHOD == HW_IIC)	//1.硬件IIC

	//I2C_Configuration，初始化硬件IIC引脚
	void I2C_Configuration(void)
	{
//		I2C_InitTypeDef  I2C_InitStructure;
//		GPIO_InitTypeDef  GPIO_InitStructure;

//		RCC_APB1PeriphClockCmd(IIC_RCC_APB1Periph_I2CX, ENABLE);
//		RCC_APB2PeriphClockCmd(IIC_RCC_APB2Periph_GPIOX, ENABLE);

//		GPIO_InitStructure.GPIO_Pin =  IIC_SCL_Pin_X | IIC_SDA_Pin_X;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//I2C必须开漏输出
//		GPIO_Init(IIC_GPIOX, &GPIO_InitStructure);

//		I2C_DeInit(I2CX);
//		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//		I2C_InitStructure.I2C_OwnAddress1 = 0x30;//主机的I2C地址,随便写
//		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//		I2C_InitStructure.I2C_ClockSpeed = 400000;

//		I2C_Cmd(I2CX, ENABLE);
//		I2C_Init(I2CX, &I2C_InitStructure);
//		delay_ms(200);
	}

	/**
		 @brief  I2C_WriteByte，向OLED寄存器地址写一个byte的数据
		 @param  addr：寄存器地址
						 data：要写入的数据
		 @retval 无
	*/
	void I2C_WriteByte(uint8_t addr, uint8_t data)
	{
//		while (I2C_GetFlagStatus(I2CX, I2C_FLAG_BUSY));

//		I2C_GenerateSTART(I2CX, ENABLE);//开启I2C1
//		while (!I2C_CheckEvent(I2CX, I2C_EVENT_MASTER_MODE_SELECT)); /*EV5,主模式*/

//		I2C_Send7bitAddress(I2CX, OLED_ADDRESS, I2C_Direction_Transmitter);//器件地址 -- 默认0x78
//		while (!I2C_CheckEvent(I2CX, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

//		I2C_SendData(I2CX, addr);//寄存器地址
//		while (!I2C_CheckEvent(I2CX, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//		I2C_SendData(I2CX, data);//发送数据
//		while (!I2C_CheckEvent(I2CX, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//		I2C_GenerateSTOP(I2CX, ENABLE);//关闭I2C1总线
		extern I2C_HandleTypeDef hi2c3;
		HAL_I2C_Mem_Write(&hi2c3, OLED_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	}

	void WriteCmd(unsigned char cmd)//写命令
	{
		I2C_WriteByte(0x00, cmd);
	}

	void WriteDat(unsigned char dat)//写数据
	{
		I2C_WriteByte(0x40, dat);
	}


#elif  (TRANSFER_METHOD == SW_IIC)	//2.软件IIC
	
	void IIC_Start(void);
	void IIC_Send_Byte(uint8_t txd);
	void IIC_Stop(void);
	
	//初始化
	void I2C_Configuration(void)
	{
		GPIO_InitTypeDef  GPIO_InitStructure;

		RCC_APB2PeriphClockCmd(OLED_RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */

		GPIO_InitStructure.GPIO_Pin = OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	/* 开漏输出 */
		GPIO_Init(OLED_GPIO_PORT_I2C, &GPIO_InitStructure);
	
		IIC_SDA = 1;
		IIC_SCL = 1;
	}

	/**
		 @brief  I2C_WriteByte，向OLED寄存器地址写一个byte的数据
		 @param  addr：寄存器地址 data：要写入的数据
		 @retval 无
	*/
	void I2C_WriteByte(uint8_t addr, uint8_t data)
	{
		IIC_Start();
		IIC_Send_Byte(OLED_ADDRESS);
		IIC_Send_Byte(addr);		//write data
		IIC_Send_Byte(data);
		IIC_Stop();
	}

	void WriteCmd(unsigned char cmd)//写命令
	{
		I2C_WriteByte(0x00, cmd);
	}

	void WriteDat(unsigned char dat)//写数据
	{
		I2C_WriteByte(0x40, dat);
	}

	//以下函数开始为软件IIC驱动
	//产生IIC起始信号
	void IIC_Start(void)
	{
		IIC_SCL = 1;
		IIC_SDA = 1;
		//delay_us(1);
		IIC_SDA = 0;
		//delay_us(1);
		IIC_SCL = 0;
	}

	//IIC发送一个字节
	//返回从机有无应答
	//1，有应答
	//0，无应答
	void IIC_Send_Byte(uint8_t IIC_Byte)
	{
		unsigned char i;
		for (i = 0; i < 8; i++)
		{
			if (IIC_Byte & 0x80)
				IIC_SDA = 1;
			else 
				IIC_SDA = 0;
			//delay_us(1);
			IIC_SCL = 1;
			delay_us(1);  //必须有保持SCL脉冲的延时
			IIC_SCL = 0;
			IIC_Byte <<= 1;
		}
		//原程序这里有一个拉高SDA，根据OLED的要求，此句必须去掉。
		IIC_SCL = 1;
		delay_us(1);
		IIC_SCL = 0;
	}

	//产生IIC停止信号
	void IIC_Stop(void)
	{
		IIC_SDA = 0;
		//delay_us(1);
		IIC_SCL = 1;
		//delay_us(1);
		IIC_SDA = 1;
	}

#elif  (TRANSFER_METHOD ==HW_SPI)	//3.硬件SPI

	#define OLED_RESET_LOW()          GPIO_ResetBits(SPI_RES_GPIOX, SPI_RES_PIN)  //低电平复位
	#define OLED_RESET_HIGH()         GPIO_SetBits(SPI_RES_GPIOX, SPI_RES_PIN)

	#define OLED_CMD_MODE()           GPIO_ResetBits(SPI_DC_GPIOX, SPI_DC_PIN)    //命令模式
	#define OLED_DATA_MODE()          GPIO_SetBits(SPI_DC_GPIOX, SPI_DC_PIN)      //数据模式

	#define OLED_CS_HIGH()            GPIO_SetBits(SPI_CS_GPIOX, SPI_CS_Pin_X)
	#define OLED_CS_LOW()             GPIO_ResetBits(SPI_CS_GPIOX, SPI_CS_Pin_X)


	void SPI_Configuration(void)
	{
		SPI_InitTypeDef  SPI_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	#if   (USE_HW_SPI==SPI_2)
		RCC_APB1PeriphClockCmd(SPI_RCC_APB1Periph_SPIX, ENABLE);
	#elif (USE_HW_SPI==SPI_1)
		RCC_APB2PeriphClockCmd(SPI_RCC_APB2Periph_SPIX, ENABLE);
	#endif
		RCC_APB2PeriphClockCmd(SPI_RCC_APB2Periph_GPIOX | RCC_APB2Periph_AFIO, ENABLE);

		GPIO_InitStructure.GPIO_Pin = SPI_CS_Pin_X;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(SPI_CS_GPIOX, &GPIO_InitStructure);
		OLED_CS_HIGH();

		GPIO_InitStructure.GPIO_Pin = SPI_HW_ALL_PINS;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(SPI_HW_ALL_GPIOX, &GPIO_InitStructure);


		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Tx;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;

		SPI_Init(SPIX, &SPI_InitStructure);
		SPI_Cmd(SPIX, ENABLE);

		GPIO_InitStructure.GPIO_Pin = SPI_RES_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(SPI_RES_GPIOX, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = SPI_DC_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(SPI_DC_GPIOX, &GPIO_InitStructure);
		OLED_DATA_MODE();

		OLED_RESET_HIGH();
		WaitTimeMs(100);
		OLED_RESET_LOW();
		WaitTimeMs(100);
		OLED_RESET_HIGH();
	}

	void SPI_WriterByte(unsigned char dat)
	{

		while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_TXE) == RESET ) {}; //检查指定的SPI标志位设置与否:发送缓存空标志位
		SPI_I2S_SendData(SPIX, dat); //通过外设SPIx发送一个数据
		while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_RXNE) == RESET) {}; //检查指定的SPI标志位设置与否:接受缓存非空标志位
		SPI_I2S_ReceiveData(SPIX); //返回通过SPIx最近接收的数据

	}

	void WriteCmd(unsigned char cmd)
	{
		OLED_CMD_MODE();
		OLED_CS_LOW();
		SPI_WriterByte(cmd);
		OLED_CS_HIGH();
		OLED_DATA_MODE();
	}

	void WriteDat(unsigned char dat)
	{
		OLED_DATA_MODE();
		OLED_CS_LOW();
		SPI_WriterByte(dat);
		OLED_CS_HIGH();
		OLED_DATA_MODE();
	}


#elif (TRANSFER_METHOD ==SW_SPI)	//4.软件SPI

	//引脚初始化
	void SPI_Configuration(void)
	{
		GPIO_InitTypeDef  GPIO_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);     	//使能A端口时钟
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        	//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       	//速度50MHz
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_4);

		OLED_RST_Set();
		delay_ms(100);
		OLED_RST_Clr();
		delay_ms(200);
		OLED_RST_Set();
	}
	
	//写字节
	void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
	{
		uint8_t i;
		if (cmd)
		OLED_DC_Set();
		else
		OLED_DC_Clr();
		OLED_CS_Clr();
		for (i = 0; i < 8; i++)
		{
		OLED_SCLK_Clr();
		if (dat & 0x80)
		OLED_SDIN_Set();
		else
		OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat <<= 1;
		}
		OLED_CS_Set();
		OLED_DC_Set();
	}
	
	//写命令
	void WriteCmd(unsigned char cmd)
	{
		OLED_WR_Byte(cmd, 0);
	}
	
	//写数据
	void WriteDat(unsigned char dat)
	{
		OLED_WR_Byte(dat, 1);
	}

#endif //(TRANSFER_METHOD ==HW_IIC)


void OLED_Init(void)
{
	HAL_Delay(800);
	WriteCmd(0xAE); //display off

	WriteCmd(0x20); //Set Memory Addressing Mode
//	WriteCmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0x00); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	
//The setting of the start and end addresses of SSD1306 (SSD1306 and SSD1315 are compatible)
	WriteCmd(0xb0); //Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8); //Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address


//The setting of the start and end addresses of SSD1315	(SSD1306 and SSD1315 are compatible)
	WriteCmd(0X21);	//---set column start and end address
  WriteCmd(0X00);	//---set start column address 0
  WriteCmd(0X7F);	//---set end column address 127
  WriteCmd(0X22);	//--set page start and end address
  WriteCmd(0X00);	//--set start address 0
  WriteCmd(0X07);	//--set end address 7

	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //亮度调节 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
	OLED_CLS();
}

void OLED_CLS(void)//清屏
{
//	unsigned char m, n;
//	for (m = 0; m < 8; m++)
//	{
//		WriteCmd(0xb0 + m); //page0-page1
//		WriteCmd(0x00);   //low column start address
//		WriteCmd(0x10);   //high column start address
//		for (n = 0; n < 128; n++)
//		{
//			WriteDat(0x00);
//		}
//	}

	extern unsigned char ScreenBuffer[SCREEN_PAGE_NUM][SCREEN_COLUMN];

    uint16_t i;
    for (i = 0; i < SCREEN_PAGE_NUM * SCREEN_COLUMN; ++i)
    {
        ScreenBuffer[0][i] = 0;
    }
	OLED_FILL(ScreenBuffer[0]);
}

void OLED_ON(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED唤醒
}

void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
}

void OLED_FILL(unsigned char BMP[])
{
//	uint8_t i, j;
	unsigned char *p;
	p = BMP;

//	for (i = 0; i < 8; i++)
//	{
//		WriteCmd(0xb0 + i); //page0-page1
//		WriteCmd(0x00);   //low column start address
//		WriteCmd(0x10);

//		for (j = 0; j < 128; j++)
//		{
//			WriteDat(*p++);
//		}
//	}
	extern I2C_HandleTypeDef hi2c3;
//    while(hi2c1.State != HAL_I2C_STATE_READY);
    HAL_I2C_Mem_Write_DMA(&hi2c3, OLED_ADDRESS, 0x40, I2C_MEMADD_SIZE_8BIT, p, SCREEN_PAGE_NUM * SCREEN_PAGEDATA_NUM);
}



