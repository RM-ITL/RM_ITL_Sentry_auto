/**
 * @file gimbal_behaviour.h
 * @author 何清华
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"
#include "gimbal_task.h"

//自瞄pid


#define YAW_DEADBAND 2.0f//2.0f//2.0f  // yaw轴死区范围
#define YAW_MAX_SPEED 0.05f  // yaw最大旋转速度
#define YAW_KP 0.00003f//0.00001f //0.00007f 
#define YAW_KI 0.000000000000041f//0.00000041f 
#define YAW_KD 0.00013f//0.0000280f 
#define YAW_INTEGRAL_LIMIT 0.00000000005f//0.0000005f // 减小积分限幅   
#define YAW_D_FILTER 0.8f  // 微分项低通滤波系数
#define PITCH_DEADBAND 1.0f  // pitch轴死区范围
#define PITCH_MAX_SPEED 0.0002f  // pitch最大转动速度
#define PITCH_KP 0.00006f//0.00009f  // pitch轴比例系数
#define PITCH_KI 0.0000000004f//0.0000003f // pitch轴积分系数
#define PITCH_KD 0.011f//0.00008f  // pitch轴微分系数
#define PITCH_INTEGRAL_LIMIT 0.000002f // pitch积分限幅
#define PITCH_D_FILTER 0.85f  // pitch微分项低通滤波系数
static float yaw_error_sum = 0.0f;  // 误差累积
static float yaw_last_error = 0.0f; // 上一次误差
static float yaw_d_filter = 0.0f; // 用于微分项滤波
static float pitch_error_sum = 0.0f;  // pitch误差累积
static float pitch_last_error = 0.0f; // pitch上一次误差
static float pitch_d_filter = 0.0f;   // pitch微分项滤波
extern uint8_t find_target;

typedef enum
{
  GIMBAL_ZERO_FORCE = 0,
  GIMBAL_INIT,
  GIMBAL_ABSOLUTE_ANGLE,
  GIMBAL_RELATIVE_ANGLE,
  GIMBAL_MOTIONLESS,
  GIMBAL_AUTO
} gimbal_behaviour_e;

/**
 * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */
extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
 * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
 * @param[in]      gimbal_mode_set:云台数据指针
 * @retval         none
 */
extern void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台在某些行为下，需要底盘不动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
extern bool_t gimbal_cmd_to_chassis_stop(void);

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
extern bool_t gimbal_cmd_to_shoot_stop(void);

//float accpetError = 0.01f;
//float limit_me = 0.7853981633974f
#endif
