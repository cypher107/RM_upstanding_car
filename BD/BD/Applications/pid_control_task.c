#include "pid_CONTROL_TASK.h"
#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "tim.h"
#include <math.h>
#include "INS_task.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include <string.h>


#define SPEED_FORWARD_DEADBAND 50.0f
#define SPEED_LEFT_RIGHT_DEADBAND 30.0f
#define DEFAULT_ANGLE 0.0f
// 速度单位为 rpm
#define RC_CH_MAX 660                // 约为 660
#define MAX_FORWARD_SPEED 5000   // 目标速度最大值（单位：rpm，按需调整）
#define MAX_YAW_SPEED 3000        // 转向分量最大值（单位：rpm，相对量）
#define MAX_CHASSIS_CURRENT 7500 // 映射到电流输出的最大值（CAN 电流量程内）
#define RAD_TO_DEG 57.3f
/* Safety / mode configuration */
/* If either RC switch is in MID position we'll treat it as "zero-stand" mode
    (不驱动电机）。可按需修改为只检测某个开关，例如只检测右开关 s[1]。 */
#define ZERO_STAND_DETECT_USING_EITHER_SWITCH 1
/* Angle threshold (degrees). If |pitch| > ANGLE_CUTOFF_DEG, disable motor output. */
#define ANGLE_CUTOFF_DEG 19.0f
//自定义数学函数这一块
int16_t max(int16_t a, int16_t b)
{
	return ((a)>(b) ? (a): (b));
}
int16_t min(int16_t a, int16_t b)
{
	return ((a)>(b) ? (b): (a));
}

int16_t output[2];
fp32 condination_angle[3];

/* ================ 全局状态量 ================ */
/* -------------------- PID 控制器实现 -------------------- */

YawFollowPID_t yaw_pid = 
{
    .kp = 2.0f,
    .ki = 0.0f,
    .kd = 0.5f,
};
VelocityPID_t balance_velocity_pid = {
    .kp = 1.0f,
    .ki = 0.05f,
    .kd = 0.0f,
};

/* velocity PID internal state (moved from function-local statics so we can reset) */
static int16_t velocity_ERR_LowOut_last = 0;
static int16_t velocity_ENCODER_s = 0;

// ?????(??)
AttitudePID_t attitude_pid = {
    .kp = 180.0f,
    .ki = 0.0f,
    .kd = 5.0f,
};
int16_t velocity_pid_control(int16_t current_speed_L, int16_t current_speed_R, int16_t target_speed, VelocityPID_t *pid, float pitch_angle)
{
    // velocity PID state variables were moved to file-scope so they can be reset
    const fp32 a = 0.7f; 
    int16_t ERR, ERR_LowOut, temp;


    int16_t current_speed_avg = (current_speed_L + current_speed_R)/2;
    ERR = target_speed - current_speed_avg;


    ERR_LowOut = a * velocity_ERR_LowOut_last + (1.0f - a) * ERR;
    velocity_ERR_LowOut_last = ERR_LowOut;


    if (fabsf(pitch_angle) < 20.0f)
    {

#ifdef PID_TIM_DT
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        velocity_ENCODER_s += ERR_LowOut * PID_TIM_DT;
#pragma GCC diagnostic pop
#else
        velocity_ENCODER_s += ERR_LowOut;
#endif
    }
    else
    {

        velocity_ENCODER_s = 0;
    }


    if (velocity_ENCODER_s > 5000)
    {
        velocity_ENCODER_s = 5000;
    }
    else if (velocity_ENCODER_s < -5000)
    {
        velocity_ENCODER_s = -5000;
    }


    temp = pid->kp * ERR_LowOut + pid->ki * velocity_ENCODER_s;

    return temp;
}

/**
 * @brief Clear velocity PID internal state (integrator / low-pass filter history)
 */
void velocity_pid_reset(void)
{
    velocity_ERR_LowOut_last = 0;
    velocity_ENCODER_s = 0;
}


int16_t attitude_pid_control(float current_angle, float target_angle, AttitudePID_t *pid, float gyro_y)
{
    int16_t temp;
    temp = pid->kp * (target_angle - current_angle) - pid->kd * gyro_y;
    return temp;
}


int16_t yaw_follow_pid_control(float gyro_z, float target_turn, YawFollowPID_t *pid)
{
    int16_t temp;

    temp = pid->kp * (target_turn - gyro_z);
    return temp;
}
void pid_control_task(void const * argument)
{
    (void)argument;
    while(1)
    {
        const motor_measure_t *motor_left = get_chassis_motor_measure_point(0);
        const motor_measure_t *motor_right = get_chassis_motor_measure_point(1);
        const RC_ctrl_t *rc = get_remote_control_point();
        const fp32 *angles_ptr;
        fp32 local_angles[3] = {0};
        int angles_valid = 0; /* 标记是否成功读取到 INS 角度 */

        /* read INS angles (rad) in short critical section and copy to local buffer */
        taskENTER_CRITICAL(); // inhibit context switch while reading shared data
        angles_ptr = get_INS_angle_point();
        if (angles_ptr != NULL)
        {
            memcpy(local_angles, angles_ptr, sizeof(local_angles));
            angles_valid = 1;
					
        }
        taskEXIT_CRITICAL();

        /* if RC or INS not ready, skip this loop */
        if (rc == NULL || !angles_valid)
        {
            osDelay(5);
            continue;
        }

    /* read remote stick values
           Note: mapping may vary on your transmitter. Here we assume:
           rc->rc.ch[2] : right stick up/down (forward/back)
           rc->rc.ch[3] : right stick left/right (turn)
        */
        int16_t rc_forward = rc->rc.ch[2]; // -RC_CH_MAX .. RC_CH_MAX
        int16_t rc_turn = rc->rc.ch[3];    // -RC_CH_MAX .. RC_CH_MAX


    /* convert INS pitch from rad to deg for PID (code elsewhere expects deg) */
    /* 使用文件顶部定义的 RAD_TO_DEG 宏，避免重复魔法常数 */
    fp32 pitch_deg = local_angles[1] * RAD_TO_DEG;

    /* Angle cutoff protection: if tilt too large, cut motor outputs for safety */
    if (fabsf(pitch_deg) > ANGLE_CUTOFF_DEG)
    {
        /*断电保护：发送0电流并跳过控制循环。
         * 同时清除速度 PID 的积分与低通滤波历史，防止恢复时突变。
         */
        velocity_pid_reset();
        CAN_cmd_chassis(0, 0, 0, 0);
        /* 给控制器和上位机一点时间来响应并记录事件 */
        osDelay(10);
        continue;
    }

        /* map RC sticks to targets */
        /* target angle (deg) from forward stick: small max angle for velocity intention */
        const fp32 MAX_TARGET_ANGLE_DEG = 5.0f; /* tune if necessary */
        fp32 target_angle_deg = ( (fp32)rc_forward ) * MAX_TARGET_ANGLE_DEG / (fp32)RC_CH_MAX;

        /* yaw/turn target (used as differential speed) */
        int16_t target_turn = (int16_t)( (int32_t)rc_turn * MAX_YAW_SPEED / RC_CH_MAX );

        /* Outer loop: attitude control -> produce a speed command */
        /* gyro term not available here reliably, pass 0 (D-term disabled) */
        float gyro_y = 0.0f;
        int16_t attitude_output = attitude_pid_control(pitch_deg, target_angle_deg, &attitude_pid, gyro_y);

        /* attitude_output is used as target forward speed for inner speed PID */
        int16_t target_speed = attitude_output; /* units tuned by PID gains */

        /* add user requested forward/back from stick (optional): map rc_forward -> feedforward speed */
        int16_t user_speed = (int16_t)( (int32_t)rc_forward * MAX_FORWARD_SPEED / RC_CH_MAX );
        /* combine (tweak blending as needed). Here we mix small fraction of user_speed */
        target_speed += (int16_t)(user_speed * 0.2f);

        /* differential targets for left and right wheels */
        int16_t target_speed_L = target_speed - target_turn;
        int16_t target_speed_R = target_speed + target_turn;

        /* current wheel speeds (RPM) */
        int16_t current_speed_L = 0;
        int16_t current_speed_R = 0;
        if (motor_left)  current_speed_L = motor_left->speed_rpm;
        if (motor_right) current_speed_R = motor_right->speed_rpm;

        /* Inner loop: velocity PID -> produce motor current commands */
        int16_t current_L = velocity_pid_control(current_speed_L, current_speed_R, target_speed_L, &balance_velocity_pid, pitch_deg);
        int16_t current_R = velocity_pid_control(current_speed_L, current_speed_R, target_speed_R, &balance_velocity_pid, pitch_deg);

        /* limit outputs */
        if (current_L > MAX_CHASSIS_CURRENT) current_L = MAX_CHASSIS_CURRENT;
        if (current_L < -MAX_CHASSIS_CURRENT) current_L = -MAX_CHASSIS_CURRENT;
        if (current_R > MAX_CHASSIS_CURRENT) current_R = MAX_CHASSIS_CURRENT;
        if (current_R < -MAX_CHASSIS_CURRENT) current_R = -MAX_CHASSIS_CURRENT;

        
                /* Right switch (s[1]) to disable motors when in DOWN position */
                if (rc->rc.s[1] == RC_SW_DOWN)
                {
                    CAN_cmd_chassis(0, 0, 0, 0);
                    osDelay(5);
                    continue;
                }

				/* send motor commands: map to motor1 and motor2; keep others zero */
				
        CAN_cmd_chassis(current_L, -current_R, 0, 0);
//				CAN_cmd_chassis(0, 0, 0, 0);
        osDelay(10); /* control loop period (ms) */
    }
}


