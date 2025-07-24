/**
 * @file chassis_behaviour.h
 * @author 何清华
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"
#include "USART_receive.h"

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 0
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 1
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL LEFT_SWITCH
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f
//摇杆死区
#define CHASSIS_RC_DEADLINE 10
#define CHASSIS_OPEN_RC_SCALE 10 //在chassis_open 模型下，遥控器乘以该比例发送到can上


#define AUTO_CHASSIS_TOP 0
#define AUTO_MOVE 1
#define AUTO_IN_HOME 2

/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */
extern void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector);
extern int auto_move_flag;

#endif
