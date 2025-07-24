/**
 * @file USART_receive.h
 * @author 何清华
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"

#define USART1_RX_BUF_NUM 24u//52u // 18u是18字节长度
#define USER_FRAME_LENGTH 12u//26u

#define USART_PI 3.1416f
#define length 8 //暂定
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

//发送--自瞄数据--新加//
//typedef struct
//{
//    uint8_t bullet_type;
//    uint8_t shooter_id;
//    uint8_t bullet_freq;
//    float bullet_speed;
//} ext_shoot_data_t;

//typedef struct
//{
//    // 姿态四元数
//    float ori_x;
//    float ori_y;
//    float ori_z;
//    float ori_w;
//    // 角速度
//    float angular_v_x;
//    float angular_v_y;
//    float angular_v_z;
//    // 线速度
//    float acc_x;
//    float acc_y;
//    float acc_z;
//} imu_t;

//接收--自瞄数据——新加//
//typedef struct
//{
//    uint8_t type;           // 0:停止射击; 1: 一次射击; 2: 连续射击
//    uint8_t projectile_num; // (可选参数) 射击子弹数目
//} shoot_cmd_t;

//typedef struct
//{
//    uint8_t yaw_type;   // 1: 绝对角度控制，相对云台imu; 2: 相对角度控制; 3: 纯速度控制
//    uint8_t pitch_type; // 同上
//    float position_yaw; // 单位 rad
//    float position_pitch;
//    float velocity_yaw; // 单位 rad/s
//    float velocity_pitch;
//} gimbal_cmd_t;
//接收--自瞄数据--新加//
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
