#include "vision.h"
#include "usart.h"
#include <string.h>
#define data_len 14
uint8_t RxBuffer;
Vision_Data vision_info;

static void Data_Analysis(const uint8_t *datas);
static float Byte_to_Float(const unsigned char *p);
static void Receive(uint8_t bytedata);

void usart1_vision_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //开启串口中断
}

/**
 * @brief          数据解析
 * @param[in]      type:数据类型
 * @param[in]      datas:数据地首地址
 * @retval         none
 */
static void Data_Analysis(const uint8_t *datas)
{
	vision_info.Data_CMD = datas[0];
	vision_info.Data_ID = datas[1];
	vision_info.z = Byte_to_Float(&datas[2]);
	vision_info.x = Byte_to_Float(&datas[6]);
	vision_info.y = Byte_to_Float(&datas[10]);
}

// 四字节转浮点
float Byte_to_Float(const unsigned char *p)
{
	uint32_t uint_num;
	memcpy(&uint_num, p, sizeof(uint32_t));

	float float_data;
	memcpy(&float_data, &uint_num, sizeof(float));

	return float_data;
}

/**
 * @brief          接收数据
 * @param[in]      bytedata:每次接收到的数据
 * @retval         none
 */
static void Receive(uint8_t bytedata)
{
	static uint8_t step = 0;		  // 状态变量初始化为0 在函数中必须为静态变量
	static uint8_t cnt = 0, Buf[300]; // 数据暂存
	static uint8_t *data_ptr;		  // 实际接收数据的地址
	//进行数据解析 状态机
	switch (step)
	{
	case 0: // 接收帧头1
		if (bytedata == 0xAA)
		{
			step++;
			cnt = 0;
			Buf[cnt++] = bytedata;
		}
		break;
	case 1: // 接收帧头2
		if (bytedata == 0xDD)
		{
			step++;
			Buf[cnt++] = bytedata;
			data_ptr = &Buf[cnt]; // 记录数据指针首地址
		}
		else if (bytedata == 0xAA)
		{
			step = 1;
		}
		else
		{
			step = 0;
		}
		break;
	case 2: // 接收len字节数据
		Buf[cnt++] = bytedata;
		if (data_ptr + data_len == &Buf[cnt]) //利用指针地址偏移判断是否接收完len位数据
		{
			step++;
		}
		break;
	case 3: // 数据解析
		Data_Analysis(data_ptr);
		step = 0;
		break;
	default:
		step = 0;
		break;
	}
}

void USART1_IRQHandler(void)
{
	if (huart1.Instance->SR & UART_FLAG_RXNE)
	{
		RxBuffer = huart1.Instance->DR;
		Receive(RxBuffer);
	}
}

/**
 * @brief          获取视觉数据指针
 * @param[in]      none
 * @retval         视觉数据指针
 */
const Vision_Data *get_vision_data_point(void)
{
	return &vision_info;
}
