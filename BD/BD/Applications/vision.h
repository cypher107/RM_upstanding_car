#ifndef __VISION_H
#define __VISION_H
#include "main.h"

typedef struct
{
	uint8_t Data_CMD; //标志位(0未识别，1识别)
	uint8_t Data_ID;  //识别对象
	float x;		  //对象x坐标
	float y;		  //对象y坐标
	float z;		  //对象深度

} Vision_Data;

void usart1_vision_init(void);
const Vision_Data *get_vision_data_point(void);

#endif
