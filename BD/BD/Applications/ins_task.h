#ifndef INS_TASk_H
#define INS_TASk_H
#include "struct_typedef.h"

#define SPI_DMA_GYRO_LENGHT 8
#define SPI_DMA_ACCEL_LENGHT 9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS 0
#define IMU_SPI_SHFITS 1
#define IMU_UPDATE_SHFITS 2

#define BMI088_GYRO_RX_BUF_DATA_OFFSET 1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f	   //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f	   //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT 4500.0f	 //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 // mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

/**
 * @brief          imu task, init bmi088, ist8310, calculate the euler angle
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void INS_task(void);

/**
 * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
 * @param[in]      none
 * @retval         the point of INS_angle
 */
/**
 * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
 * @param[in]      none
 * @retval         INS_angle的指针
 */
extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_gyro_data_point(void);

#endif
