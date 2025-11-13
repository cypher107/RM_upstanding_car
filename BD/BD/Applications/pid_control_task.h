/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_CONTROL_TASK_H
#define PID_CONTROL_TASK_H
#include "struct_typedef.h"

typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Params_t;

typedef PID_Params_t VelocityPID_t;
typedef PID_Params_t AttitudePID_t;
typedef PID_Params_t YawFollowPID_t;


extern AttitudePID_t attitude_pid;
extern YawFollowPID_t yaw_pid;


#define VELOCITY_LOW_PASS_K 0.7f
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define WHEEL_RADIUS_M 0.05f
#define PID_TIM_DT 0.005f
#define COMP_ALPHA 0.98f
#define BMI_GYRO_IN_DEG 1
#define YAW_TO_CURRENT_GAIN 800.0f
int16_t velocity_pid_control(int16_t current_speed_L, int16_t current_speed_R, int16_t target_speed, VelocityPID_t *pid, float roll_angle);

void pid_tim1_control(void);
/* reset internal state of velocity PID (clear integrator / filters) */
void velocity_pid_reset(void);
// ????????,??? yaw ????? 0
#define DEFAULT_YAW_ANGLE 0.0f
// ???????????,???????????????(??:???????)
#define DEFAULT_TURN_CURRENT 2000.0f
// ?????????????(-1..1 ???),???????? DEFAULT_TURN_CURRENT
#define TURN_INPUT_OVERRIDE_THRESHOLD 0.6f

#endif // PID_H
