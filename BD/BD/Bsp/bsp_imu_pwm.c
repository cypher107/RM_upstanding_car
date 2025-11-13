/**
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 * @file       bsp_imu_pwm.c/h
 * @brief
 * @note
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     Oct-26-2023     Shidong Wu	   初步完成
 *
 @verbatim
 ==============================================================================

 ==============================================================================
 @endverbatim
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 */
#include "bsp_imu_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
	__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
