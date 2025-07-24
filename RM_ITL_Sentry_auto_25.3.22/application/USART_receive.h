/**
 * @file USART_receive.h
 * @author ���廪
 * @brief �����жϽ��պ�������������Ƭ���������豸�Ĵ���ͨ������
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"

#define USART1_RX_BUF_NUM 24u//52u // 18u��18�ֽڳ���
#define USER_FRAME_LENGTH 12u//26u

#define USART_PI 3.1416f
#define length 8 //�ݶ�
typedef struct
{
    float pitch_add;
    float yaw_add;
    bool_t new_usart_data_flag;
    float auto_yaw;
    float auto_pitch;
} auto_shoot_t;

typedef struct
{
    float auto_vx;
    float auto_vy;
    float auto_wz;
} auto_move_t;


typedef struct
{
    float *gimbal_pitch_angle;
    float *gimbal_yaw_gyro;
    uint8_t enemy_color;
} user_send_data_t;

//����--��������--�¼�//
//typedef struct
//{
//    uint8_t bullet_type;
//    uint8_t shooter_id;
//    uint8_t bullet_freq;
//    float bullet_speed;
//} ext_shoot_data_t;

//typedef struct
//{
//    // ��̬��Ԫ��
//    float ori_x;
//    float ori_y;
//    float ori_z;
//    float ori_w;
//    // ���ٶ�
//    float angular_v_x;
//    float angular_v_y;
//    float angular_v_z;
//    // ���ٶ�
//    float acc_x;
//    float acc_y;
//    float acc_z;
//} imu_t;

//����--�������ݡ����¼�//
//typedef struct
//{
//    uint8_t type;           // 0:ֹͣ���; 1: һ�����; 2: �������
//    uint8_t projectile_num; // (��ѡ����) ����ӵ���Ŀ
//} shoot_cmd_t;

//typedef struct
//{
//    uint8_t yaw_type;   // 1: ���ԽǶȿ��ƣ������̨imu; 2: ��ԽǶȿ���; 3: ���ٶȿ���
//    uint8_t pitch_type; // ͬ��
//    float position_yaw; // ��λ rad
//    float position_pitch;
//    float velocity_yaw; // ��λ rad/s
//    float velocity_pitch;
//} gimbal_cmd_t;
//����--��������--�¼�//
//typedef struct
//{
//    uint8_t ID;
//    uint8_t buff[length];
//} sendData;
extern auto_shoot_t auto_shoot;
extern auto_move_t auto_move;
extern user_send_data_t user_send_data;

extern void user_data_pack_handle(void);

#endif
