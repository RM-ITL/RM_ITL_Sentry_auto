/**
 * @file pid.c/h
 * @author 何清华
 * @brief pid实现函数，包括初始化，PID计算函数
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022 SPR
 *
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
 * @brief          pid结构数据初始化
 * @param[out]     pid: PID结构数据指针
 * @param[in]      kd
 * @param[in]      ki
 * @param[in]      kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 */
extern int c_switch;
void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float max_out, float max_iout)
{
	if (pid == NULL)
	{
		return;
	}
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
float PID_calc(pid_type_def *pid, float ref, float set)
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

	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
	return pid->out;
}
// float PID_calc_1(pid_type_def *pid, float ref, float set)
// {
// 	if (pid == NULL)
// 	{
// 		return 0.0f;
// 	}

// 	pid->error[2] = pid->error[1];
// 	pid->error[1] = pid->error[0];
// 	pid->set = set;
// 	pid->fdb = ref;
// 	pid->error[0] = set - ref;

// 	pid->Pout = pid->Kp * pid->error[0];
// 	pid->Iout += pid->Ki * pid->error[0];
// 	pid->Dbuf[2] = pid->Dbuf[1];
// 	pid->Dbuf[1] = pid->Dbuf[0];
// 	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
// 	pid->Dout = pid->Kd * pid->Dbuf[0];
// 	LimitMax(pid->Iout, pid->max_iout);
// 	pid->out = pid->Pout + pid->Iout + pid->Dout;
// 	LimitMax(pid->out, pid->max_out);
// 	// if (c_switch == 7) //关
// 	// {
// 	// pid->out *= limit_k;
// 	// }
// 	return pid->out;
// }
// float PID_calc_2(pid_type_def *pid, float ref, float set)
// {
// 	if (pid == NULL)
// 	{
// 		return 0.0f;
// 	}

// 	pid->error[2] = pid->error[1];
// 	pid->error[1] = pid->error[0];
// 	pid->set = set;
// 	pid->fdb = ref;
// 	pid->error[0] = set - ref;

// 	pid->Pout = pid->Kp * pid->error[0];
// 	pid->Iout += pid->Ki * pid->error[0];
// 	pid->Dbuf[2] = pid->Dbuf[1];
// 	pid->Dbuf[1] = pid->Dbuf[0];
// 	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
// 	pid->Dout = pid->Kd * pid->Dbuf[0];
// 	LimitMax(pid->Iout, pid->max_iout);
// 	pid->out = pid->Pout + pid->Iout + pid->Dout;
// 	LimitMax(pid->out, pid->max_out);
// 	if (c_switch == 7) //关
// 	{
// 		pid->out *= limit_k*0.2;
// 	}
// 	return pid->out;
// }
// float PID_calc_3(pid_type_def *pid, float ref, float set)
// {
// 	if (pid == NULL)
// 	{
// 		return 0.0f;
// 	}

// 	pid->error[2] = pid->error[1];
// 	pid->error[1] = pid->error[0];
// 	pid->set = set;
// 	pid->fdb = ref;
// 	pid->error[0] = set - ref;

// 	pid->Pout = pid->Kp * pid->error[0];
// 	pid->Iout += pid->Ki * pid->error[0];
// 	pid->Dbuf[2] = pid->Dbuf[1];
// 	pid->Dbuf[1] = pid->Dbuf[0];
// 	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
// 	pid->Dout = pid->Kd * pid->Dbuf[0];
// 	LimitMax(pid->Iout, pid->max_iout);
// 	pid->out = pid->Pout + pid->Iout + pid->Dout;
// 	LimitMax(pid->out, pid->max_out);
// 	if (c_switch == 7) //关
// 	{
// 		pid->out *= limit_k * 5.0;
// 	}
// 	return pid->out;
// }
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

//位置式PID
float Position_Motor_PID_Calc(pid_type_def *pid, int SetPoint, float currentPoint, float acceptError, int limitValue)
{
	static float Bias, Last_bias, Integral_bias, outPut;
	pid->set = SetPoint;
	Bias = pid->set - currentPoint;
	Integral_bias += Bias;
	//死区
	if (-acceptError <= Bias && Bias <= acceptError)
	{
		outPut = 0;
	}
	else
	{
		outPut = (pid->Kp * Bias) +
				 (pid->Ki * Integral_bias) +
				 (pid->Kd * (Bias - Last_bias));
		Last_bias = Bias;
	}
	pid->max_out = outPut;
	//输出限幅
	if (pid->max_out >= limitValue)
	{
		pid->max_out = limitValue;
	}
	else if (pid->max_out <= (-limitValue))
	{
		pid->max_out = -limitValue;
	}
}
