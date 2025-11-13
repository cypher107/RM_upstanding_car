/**
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 * @file       pid.c/h
 * @brief      pid实现函数，包括初始化，PID计算函数
 * @note
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     Oct-26-2023	   Shidong Wu      增加离散PID计算函数
 *
 @verbatim
 ==============================================================================

 ==============================================================================
 @endverbatim
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 */
#include "pid.h"
#include "main.h"
#define LimitMax(input, max)   \
	{                          \
		if (input > max)       \
		{                      \
			input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
			input = -max;      \
		}                      \
	}

/**
 * @brief          pid初始化
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION或PID_DELTA
 * @param[in]      kp: Kp
 * @param[in]      ki: Ki
 * @param[in]      kd: Kd
 * @param[in]      max_out: 最大输出
 * @param[in]      max_iout: 最大积分输出
 * @retval         none
 */
void PID_init(pid_type_def *pid, uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout)
{
	if (pid == NULL)
	{
		return;
	}
	pid->mode = mode;
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算（连续型）
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;
	if (pid->mode == PID_POSITION)
	{
		pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
		pid->Dout = pid->Kd * pid->Dbuf[0];
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	else if (pid->mode == PID_DELTA)
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
		pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
		pid->Dout = pid->Kd * pid->Dbuf[0];
		pid->out += pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	return pid->out;
}

/**
 * @brief          pid计算（离散型）
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @param[in]      dt: 采样时间
 * @retval         pid输出
 */
/**
 * @brief          pid calculate (discrete)
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @param[in]      dt: sample time
 * @retval         pid out
 */

fp32 Discrete_PID_calc(pid_type_def *pid, fp32 ref, fp32 set, fp32 dt)//目标位置set，现在位置ref
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;
	if (pid->mode == PID_POSITION) //位置式PID
	{
		pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0] * dt; //离散积分项
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
		pid->Dout = pid->Kd * pid->Dbuf[0] / dt; //离散微分项
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout + pid->Iout + pid->Dout; //位置式PID输出
		LimitMax(pid->out, pid->max_out);
	}
	else if (pid->mode == PID_DELTA) //增量式PID
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
		pid->Iout = pid->Ki * pid->error[0] * dt; //离散积分项
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
		pid->Dout = pid->Kd * pid->Dbuf[0] / dt;	   //离散微分项
		pid->out += pid->Pout + pid->Iout + pid->Dout; //增量式PID输出
		LimitMax(pid->out, pid->max_out);
	}
	return pid->out;
}

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
void PID_clear(pid_type_def *pid)
{
	if (pid == NULL)
	{
		return;
	}

	pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
	pid->fdb = pid->set = 0.0f;
}
