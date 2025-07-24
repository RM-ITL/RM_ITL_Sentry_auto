/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "BMI088driver.h"
/* ******************************************************************
                        AGV_CHASIS
****************************************************************** */
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//云台任务控制间隔 1ms
#define GIMBAL_CONTROL_TIME 1

// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL LEFT_SWITCH

//掉头云台速度
#define TURN_SPEED 0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10

//灵敏度
#define YAW_RC_SEN -0.000005f
#define PITCH_RC_SEN -0.000005f // 0.005
#define YAW_MOUSE_SEN 0.00007f
//0.00005
#define PITCH_MOUSE_SEN 0.00012f
//0.00015
#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.015f

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

///////////////////////////电机，陀螺仪安装方向////////////////////////////////
#define PITCH_TURN 1
#define PITCH_OUT_TURN 1
#define YAW_TURN 1
// 1
#define YAW_OUT_TURN 0

// PITCH云台PID参数
//步兵1 Kp 25.0 Ki 0.0 Kd 0.0
//步兵2 Kp 18.0 Ki 0.015 Kd 0.2
#define PITCH_ABSOLUTE_ANGLE_KP 20.0f//15.0f//10.0f//20.0f//40.0f
#define PITCH_ABSOLUTE_ANGLE_KI 0.02f
#define PITCH_ABSOLUTE_ANGLE_KD 0.35f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT 20.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT 0.2f
//步兵1 Kp 10000.0 Ki 0.0 Kd 0.0 MAX_OUT 20000 MAX_IOUT 0.0
//步兵2 Kp 10000.0 Ki 0.0 Kd 0.0 MAX_OUT 20000 MAX_IOUT 0.0 
//步兵3 Kp 10000.0
#define PITCH_GYRO_KP 6000.0f
#define PITCH_GYRO_KI 2.0f
#define PITCH_GYRO_KD 40.0f
#define PITCH_GYRO_MAX_OUT 18000.0f//18000.0f
#define PITCH_GYRO_MAX_IOUT 1000.0f

// YAW云台PID参数
//步兵1 Kp 20.0 Ki 0.0 Kd 0.0
//步兵2 Kp 20.0 Ki 0.05 Kd 0.0
#define YAW_ABSOLUTE_ANGLE_KP 22.0f//13.0f//15.0f//25.0f
#define YAW_ABSOLUTE_ANGLE_KI 10.0f//0.2f
#define YAW_ABSOLUTE_ANGLE_KD 0.01f//0.05f
#define YAW_ABSOLUTE_ANGLE_KP_TUOLUO 25.0f//25.0f
#define YAW_ABSOLUTE_ANGLE_KD_TUOLUO 0.2f
// 0.05
#define YAW_ABSOLUTE_ANGLE_MAX_OUT 5.0//10.0f // 9
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT 0.0f
//步兵1 Kp 11000.0 Ki 0.0 Kd 0.0
//步兵2 Kp 10000.0 Ki 0.01 Kd 0.0
//步兵3 Kp 10000.0 Ki 0 Kd 0.0
#define YAW_GYRO_KP 6000.0f//6000.0f//12000.0f
#define YAW_GYRO_KI 0.3f//0.3f
#define YAW_GYRO_KD 0.4f//0.4f
#define YAW_GYRO_MAX_OUT 9000.0f//18000.0f
#define YAW_GYRO_MAX_IOUT 1000.0f//500.0f//1000.0f


//云台限位及中值参数设定
#define GIMBAL_YAW_OFFSET_ECD 0.8f//浮点数角度值  0---6.28rad
#define SHORT_OF_GIMBAL_YAW_OFFSET_ECD 0.063f     //对应yaw电机的relative_angle
//步兵1 -0.93f 步兵2 0.89f
#define GIMBAL_YAW_MAX_ECD 3.0f
#define GIMBAL_YAW_MIN_ECD -3.0f
#define GIMBAL_PITCH_OFFSET_ECD 0x812//0x812//0x1897
//步兵1 -0.93f 步兵2 0.89f
#define GIMBAL_PITCH_MAX_ECD 0.7f//0.5f//0.5//0.42f//0.002f//0.22f//0.12  
//步兵1 -0.67f 步兵2 -0.81f
#define GIMBAL_PITCH_MIN_ECD -0.25f//-0.13////-0.2f//-0.4f//-0.15f//-0.12f//-0.55//-0.55f//-0.62
//步兵11 -1.15f 步兵2 0.50f
/********************正常跟随模式**************************/
// #define YAW_ABSOLUTE_ANGLE_KP_Normal 12.0f.
// #define YAW_ABSOLUTE_ANGLE_KD_Normal 0.2f
// #define YAW_GYRO_KP_Normal 4000.0f
// #define PITCH_ABSOLUTE_ANGLE_KP_Normal 12.0f
// #define PITCH_GYRO_KP_Normal 4000.0f

/*******************自瞄模式*****************************/
// PITCH云台PID参数
//步兵1 Kp 25.0 Ki 0.0 Kd 0.0
//步兵2 Kp 18.0 Ki 0.015 Kd 0.2
#define PITCH_ABSOLUTE_ANGLE_KP_zimiao 18.0f
#define PITCH_ABSOLUTE_ANGLE_KI_zimiao 0.0f
#define PITCH_ABSOLUTE_ANGLE_KD_zimiao -1.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT_zimiao 1.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT_zimiao 0.0f
//步兵1 Kp 10000.0 Ki 0.0 Kd 0.0 MAX_OUT 20000 MAX_IOUT 0.0
//步兵2 Kp 10000.0 Ki 0.0 Kd 0.0 MAX_OUT 20000 MAX_IOUT 0.0 
//步兵3 Kp 10000.0
#define PITCH_GYRO_KP_zimiao 8000.0f
#define PITCH_GYRO_KI_zimiao 0
#define PITCH_GYRO_KD_zimiao 0
#define PITCH_GYRO_MAX_OUT_zimiao 20000.0f
#define PITCH_GYRO_MAX_IOUT_zimiao 1.0f

// YAW云台PID参数
//步兵1 Kp 20.0 Ki 0.0 Kd 0.0
//步兵2 Kp 20.0 Ki 0.05 Kd 0.0
#define YAW_ABSOLUTE_ANGLE_KP_zimiao 10.0f
#define YAW_ABSOLUTE_ANGLE_KI_zimiao 0.0f
#define YAW_ABSOLUTE_ANGLE_KD_zimiao -1.0f
// 0.05
#define YAW_ABSOLUTE_ANGLE_MAX_OUT_zimiao 1.0f // 9
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT_zimiao 0.0f
//步兵1 Kp 11000.0 Ki 0.0 Kd 0.0
//步兵2 Kp 10000.0 Ki 0.01 Kd 0.0
//步兵3 Kp 10000.0 Ki 0 Kd 0.0
#define YAW_GYRO_KP_zimiao 10000.0f
#define YAW_GYRO_KI_zimiao 0.0f
#define YAW_GYRO_KD_zimiao 0.0f
#define YAW_GYRO_MAX_OUT_zimiao 20000.0f
#define YAW_GYRO_MAX_IOUT_zimiao 1000.0f 

extern float my_yaw;
extern uint8_t game_begin_flag;

//电机编码值转化成弧度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
  GIMBAL_MOTOR_RAW = 0, //电机原始值控制
  GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
  GIMBAL_MOTOR_ENCONDE  //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
  float kp;
  float ki;
  float kd;

  float set;
  float get;
  float err;

  float max_out;
  float max_iout;

  float Pout;
  float Iout;
  float Dout;

  float out;
} gimbal_PID_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  gimbal_PID_t gimbal_motor_absolute_angle_pid;
	gimbal_PID_t gimbal_motor_absolute_angle_pid_tuolo;
	gimbal_PID_t gimbal_motor_absolute_angle_pid_zimiao;
  gimbal_PID_t gimbal_motor_relative_angle_pid;
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  float offset_ecd;
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s
  float motor_gyro_set;
  float motor_speed;
  float raw_cmd_current;
  float current_set;
  int16_t given_current;
} gimbal_motor_t;

typedef struct
{
  gimbal_motor_t gimbal_yaw_motor;
  gimbal_motor_t gimbal_pitch_motor;
//  gimbal_motor_t gimbal_yaw_moter;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief 云台姿态数据发送到上位机
 *
 */
extern void gimbal_data_send(void);

extern void gimbal_task(void const *pvParameters);

#endif
