/**
 * @file gimbal_behaviour.h
 * @author ���廪
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

//����pid


#define YAW_DEADBAND 2.0f//2.0f//2.0f  // yaw��������Χ
#define YAW_MAX_SPEED 0.05f  // yaw�����ת�ٶ�
#define YAW_KP 0.00003f//0.00001f //0.00007f 
#define YAW_KI 0.000000000000041f//0.00000041f 
#define YAW_KD 0.00013f//0.0000280f 
#define YAW_INTEGRAL_LIMIT 0.00000000005f//0.0000005f // ��С�����޷�   
#define YAW_D_FILTER 0.8f  // ΢�����ͨ�˲�ϵ��
#define PITCH_DEADBAND 1.0f  // pitch��������Χ
#define PITCH_MAX_SPEED 0.0002f  // pitch���ת���ٶ�
#define PITCH_KP 0.00006f//0.00009f  // pitch�����ϵ��
#define PITCH_KI 0.0000000004f//0.0000003f // pitch�����ϵ��
#define PITCH_KD 0.011f//0.00008f  // pitch��΢��ϵ��
#define PITCH_INTEGRAL_LIMIT 0.000002f // pitch�����޷�
#define PITCH_D_FILTER 0.85f  // pitch΢�����ͨ�˲�ϵ��
static float yaw_error_sum = 0.0f;  // ����ۻ�
static float yaw_last_error = 0.0f; // ��һ�����
static float yaw_d_filter = 0.0f; // ����΢�����˲�
static float pitch_error_sum = 0.0f;  // pitch����ۻ�
static float pitch_last_error = 0.0f; // pitch��һ�����
static float pitch_d_filter = 0.0f;   // pitch΢�����˲�
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
 * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
 * @param[out]     gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
 * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
 * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
 * @param[in]      gimbal_mode_set:��̨����ָ��
 * @retval         none
 */
extern void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
extern bool_t gimbal_cmd_to_chassis_stop(void);

/**
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
extern bool_t gimbal_cmd_to_shoot_stop(void);

//float accpetError = 0.01f;
//float limit_me = 0.7853981633974f
#endif
