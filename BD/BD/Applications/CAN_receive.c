/**
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 * @file       CAN_receive.c/h
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
#include "CAN_receive.h"
#include "can.h"
static void get_moto_offset(motor_measure_t *ptr, uint8_t *data);
static void get_motor_measure(motor_measure_t *ptr, uint8_t *data);
static void get_total_angle(motor_measure_t *ptr);

static motor_measure_t motor_chassis[4];	// 0-3:3508 chassis motor
static motor_measure_t motor_steer[4];		// 0-3:6020 steer motor
static motor_measure_t trigger_motor = {0}; // 2006 trigger motor
static motor_measure_t fric_motor[2];		// 3508 fric motor
static motor_measure_t gimbal_motor[3];		// 0-2:6020 gimbal motor

static CAN_TxHeaderTypeDef gimbal_tx_message; //云台CAN发送结构体
static uint8_t gimbal_can_send_data[8];		  //云台CAN发送数据

static CAN_TxHeaderTypeDef chassis_tx_message; //底盘CAN发送结构体
static uint8_t chassis_can_send_data[8];	   //底盘CAN发送数据

//拨弹CAN发送结构体
static CAN_TxHeaderTypeDef shoot_tx_message;
static uint8_t shoot_can_send_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
		{
		case CAN1_FRIC_M1_ID:
			get_motor_measure(&fric_motor[0], rx_data);
			break;
		case CAN1_FRIC_M2_ID:
			get_motor_measure(&fric_motor[1], rx_data);
			break;
		case CAN1_TRIGGER_MOTOR_ID:
			if (trigger_motor.msg_cnt < 50)
			{
				get_moto_offset(&trigger_motor, rx_data);
				trigger_motor.msg_cnt++;
			}
			else
			{
				get_motor_measure(&trigger_motor, rx_data);
			}
			get_total_angle(&trigger_motor);
			break;
		case CAN1_YAW_MOTOR_ID:
			get_motor_measure(&gimbal_motor[0], rx_data);
			break;
		case CAN1_PITCH_G1_MOTOR_ID:
			get_motor_measure(&gimbal_motor[1], rx_data);
			break;
		case CAN1_PITCH_G2_MOTOR_ID:
			get_motor_measure(&gimbal_motor[2], rx_data);
			break;
		default:
			break;
		}
	}
	else if (hcan == &hcan2)
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
		{
		case CAN2_3508_M1_ID:
		case CAN2_3508_M2_ID:
		case CAN2_3508_M3_ID:
		case CAN2_3508_M4_ID:
		{
			static uint8_t i = 0;
			// get motor id
			i = rx_header.StdId - CAN2_3508_M1_ID;
			get_motor_measure(&motor_chassis[i], rx_data);
			break;
		}
		default:
			break;
		}
	}
}

// 电机数据读取
void get_motor_measure(motor_measure_t *ptr, uint8_t *data)
{
	ptr->last_ecd = ptr->ecd;
	ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);
	ptr->speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
	ptr->given_current = (uint16_t)(data[4] << 8 | data[5]);
	ptr->temperate = data[6];
}

// 电机角度初值
void get_moto_offset(motor_measure_t *ptr, uint8_t *data)
{
	ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);
	ptr->offset_angle = ptr->ecd;
}

// 拨弹电机角度换算
void get_total_angle(motor_measure_t *ptr)
{
	if (ptr->ecd - ptr->last_ecd > 4096)
		ptr->round_cnt--;
	else if (ptr->ecd - ptr->last_ecd < -4096)
		ptr->round_cnt++;
	if (ptr->round_cnt == -18)
		ptr->round_cnt = 18;
	else if (ptr->round_cnt == 19)
		ptr->round_cnt = -17;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_angle;
	if (ptr->total_angle > (147456 - 1) || ptr->total_angle < -147456)
		ptr->total_angle += (ptr->total_angle > 0 ? -294912 : 294912);
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
	uint32_t send_mail_box;						 //发送邮箱
	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID; //发送ID
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = (yaw >> 8);
	gimbal_can_send_data[1] = yaw;
	gimbal_can_send_data[2] = (pitch >> 8);
	gimbal_can_send_data[3] = pitch;
	gimbal_can_send_data[4] = 0x00;
	gimbal_can_send_data[5] = 0x00;
	gimbal_can_send_data[6] = 0x00;
	gimbal_can_send_data[7] = 0x00;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = 0x700;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = 0;
	chassis_can_send_data[1] = 0;
	chassis_can_send_data[2] = 0;
	chassis_can_send_data[3] = 0;
	chassis_can_send_data[4] = 0;
	chassis_can_send_data[5] = 0;
	chassis_can_send_data[6] = 0;
	chassis_can_send_data[7] = 0;

	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508底盘电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508底盘电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508配重电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508配重电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送发射装置电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      fric1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      fric2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      trigger: (0x203) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]      rev: (0x204) 保留，电机控制电流
 */
void CAN_cmd_shoot(int16_t fric1, int16_t fric2, int16_t trigger, int16_t rev)
{
	uint32_t send_mail_box;
	shoot_tx_message.StdId = CAN1_MOTOR_ALL_ID;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	shoot_can_send_data[0] = fric1 >> 8;
	shoot_can_send_data[1] = fric1;
	shoot_can_send_data[2] = fric2 >> 8;
	shoot_can_send_data[3] = fric2;
	shoot_can_send_data[4] = trigger >> 8;
	shoot_can_send_data[5] = trigger;
	shoot_can_send_data[6] = rev >> 8;
	shoot_can_send_data[7] = rev;
	HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
	return &gimbal_motor[0];
}

/**
 * @brief          返回左pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_left_pitch_gimbal_motor_measure_point(void)
{
	return &gimbal_motor[1];
}

/**
 * @brief          返回右pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_right_pitch_motor_measure_point(void)
{
	return &gimbal_motor[2];
}

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
	return &trigger_motor;
}

/**
 * @brief          返回摩擦轮电机1 3508电机数据指针
 * @param[in]      none
 * @retval         摩擦轮电机1数据指针
 */
const motor_measure_t *get_fric1_motor_measure_point(void)
{

	return &fric_motor[0];
}

/**
 * @brief          返回摩擦轮电机2 3508电机数据指针
 * @param[in]      none
 * @retval         摩擦轮电机2数据指针
 */
const motor_measure_t *get_fric2_motor_measure_point(void)
{
	return &fric_motor[1];
}

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}

/**
 * @brief          返回舵轮电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_steer_motor_measure_point(uint8_t i)
{
	return &motor_steer[(i & 0x03)];
}
