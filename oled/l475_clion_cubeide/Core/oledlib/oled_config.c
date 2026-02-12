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
	
	此c文件用于配置oled
	例如初始化oled引脚 刷新oled
	注意!!! 
	如需演示动画效果 OledTimeMsFunc()需放入1ms中断等中1ms调用一次为库提供时间基准 原作是放入滴答定时器中
*/

#include "oled_config.h"
#include "oled_driver.h"


extern unsigned char ScreenBuffer[SCREEN_PAGE_NUM][SCREEN_COLUMN];
extern unsigned char TempBuffer[SCREEN_PAGE_NUM][SCREEN_COLUMN];
unsigned int OledTimeMs=0;												//时间基准

//初始化图形库，请将硬件初始化信息放入此中
void DriverInit(void)
{
	#if (TRANSFER_METHOD == HW_IIC) || (TRANSFER_METHOD == SW_IIC)
	I2C_Configuration();
	#elif (TRANSFER_METHOD == HW_SPI) || (TRANSFER_METHOD == SW_SPI)
	SPI_Configuration();	//初始化接口
	#endif
	OLED_Init();			//初始化配置oled
}

//将ScreenBuffer屏幕缓存的内容显示到屏幕上
void UpdateScreenBuffer(void)
{
	OLED_FILL(ScreenBuffer[0]);
}
//将TempBuffer临时缓存的内容显示到屏幕上
void UpdateTempBuffer(void)
{
	OLED_FILL(TempBuffer[0]);
}

//////////////////////////////////////////////////////////
//请将此函数放入1ms中断里，为图形提供时基
//系统时间基准主要用于FrameRateUpdateScreen()中固定帧率刷新屏幕
void OledTimeMsFunc(void)
{
	if(OledTimeMs != 0x00)
	{ 
		OledTimeMs--;
	}
}
//图形库普通的延时函数 需要用户自己配置 这里在delay.c中调用
void DelayMs(uint16_t ms)
{
	HAL_Delay(ms);
}

