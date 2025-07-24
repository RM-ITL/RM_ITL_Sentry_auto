/**
 * @file chassis_task.c/h
 * @author ���廪��������
 * @brief ���̿��������߳�
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
/********************************************************************************************************************
//                                                          _ooOoo_
//                                                         o8888888o
//                                                         88" . "88
//                                                         (| -_- |)
//                                                          O\ = /O
//                                                      ____/`---'\____
//                                                    .   ' \\| |// `.
//                                                     / \\||| : |||// \
//                                                   / _||||| -:- |||||- \
//                                                     | | \\\ - /// | |
//                                                   | \_| ''\---/'' | |
//                                                    \ .-\__ `-` ___/-. /
//                                                 ___`. .' /--.--\ `. . __
//                                              ."" '< `.___\_<|>_/___.' >'"".
//                                             | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                                               \ \ `-. \_ __\ /__ _/ .-` / /
//                                       ======`-.____`-.___\_____/___.-`____.-'======
//                                                          `=---='
//
//                                              ���汣��            ����BUG
********************************************************************************************************************/
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"
#include "detect_task.h"
#include "INS_task.h"
#include "custom_ui_draw.h"
#include "CAN_receive.h"
#include "USART_receive.h"
#include "referee.h"

#include "usart.h"
#include <stdio.h>

//#ifdef __GNUC__
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
// PUTCHAR_PROTOTYPE
//{
//  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//������ʽ��ӡ,����1
//  return ch;
//}

float chassis[10];
extern tank_ui_t tank;
extern super_capacity_t super_capacity;
extern ext_game_robot_state_t robot_state;
extern auto_move_t auto_move;
/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void chassis_init(void);

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void chassis_set_mode(void);

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(void);

/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @retval         none
 */
static void chassis_set_contorl(void);

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(void);

//�����˶�����
chassis_move_t chassis_move;
/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         nonex
 */
void chassis_data_send(void);

uint16_t blind_walk_count=0;
extern uint8_t blind_walk_flag;

void chassis_task(void const *pvParameters)
{
  //����һ��ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  //���̳�ʼ��
  chassis_init();
  //�жϵ��̵���Ƿ�����
  while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE))// || toe_is_error(DBUS_TOE)
  {
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }

  while (1)
  {
    //1.���õ��̿���ģʽ
    chassis_set_mode();
    
    //2.�������ݸ���
    chassis_feedback_update();
  
    //3.���̿���������
    chassis_set_contorl();
    
    
//    if(blind_walk_count<30000&&toe_is_error(DBUS_TOE))
//    {
//        chassis_move.vx_set=0.0f;
//        blind_walk_count+=1;
//    }
//    else 
//    if(blind_walk_flag==1&&toe_is_error(DBUS_TOE))
//    {
//        chassis_move.vx_set=-2.0f;
//        blind_walk_count+=1;
//    }
    
//    if(!toe_is_error(DBUS_TOE))
//    {
//        blind_walk_count=0;
//    }
//  
    //4.���̿���PID����
    chassis_control_loop();
    
    // cal_draw_gimbal_relative_angle_tangle(&tank);
    // cal_capacity(&super_capacity);
    // chassis_data_send();

    if (1)//rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT
    {
      for (int i = 0; i < 4; i++)
      {
        chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move.motor_speed_pid[i], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_OUT_MAX, CHASSIS_MOTOR_SPEED_MAX_IOUT);
      }
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move.motor_speed_pid[i], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_OUT, CHASSIS_MOTOR_SPEED_MAX_IOUT);
      }
    }

    //_202_given_current=chassis_move.motor_chassis[1].give_current;

    //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
    // if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
    // {
    //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
    chassis[0] = chassis_move.motor_chassis[0].speed;
    chassis[1] = chassis_move.motor_chassis[1].speed;
    chassis[2] = chassis_move.motor_chassis[2].speed;
    chassis[3] = chassis_move.motor_chassis[3].speed;
    chassis[4] = chassis_move.motor_chassis[0].speed_set;
    chassis[5] = chassis_move.motor_chassis[1].speed_set;
    chassis[6] = chassis_move.motor_chassis[2].speed_set;
    chassis[7] = chassis_move.motor_chassis[3].speed_set;
    if (toe_is_error(DBUS_TOE))
    {
        CAN_cmd_chassis(0, 0, 0, 0);
//    //���Ϳ��Ƶ��� 2024.2.28����Դ����С����
//      CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
//                      chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
    }
    else
    {
      //���Ϳ��Ƶ���
      CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                      chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
    }
    // }

    //ϵͳ��ʱ
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
}

/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void chassis_init(void)
{
  const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  uint8_t i;

  //��ȡ��̨�������ָ��
  chassis_move.chassis_yaw_motor = get_yaw_motor_point();
  chassis_move.chassis_pitch_motor = get_pitch_motor_point();
  //��ȡ���̵������ָ�룬��ʼ��PID
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move.motor_speed_pid[i], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_OUT, CHASSIS_MOTOR_SPEED_MAX_IOUT);
  }

  //��ʼ���Ƕ�PID(��ת������̨)
  PID_init(&chassis_move.chassis_angle_pid, CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD, CHASSIS_ANGLE_MAX_OUT, CHASSIS_ANGLE_MAX_IOUT);

  //��һ���˲�����б����������
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

  chassis_move.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

  chassis_move.top_max_speed = TOP_MAX_CHASSIS_SPEED;
  chassis_move.top_min_speed = -TOP_MAX_CHASSIS_SPEED;

  //����һ������
  chassis_feedback_update();
}

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void chassis_set_mode()
{
  chassis_behaviour_mode_set(&chassis_move);
}

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @retval         none
 */
static void chassis_feedback_update(void)
{
  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
    chassis_move.motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move.motor_chassis[i].accel = chassis_move.motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
  }

  //���µ��������ٶ� x��ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ��ʵ������BMI088��ȡ���ݣ���Щ�����ô�����
  chassis_move.vx = (chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed + chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move.vy = (chassis_move.motor_chassis[0].speed + chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move.wz = (-chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

  //���������̬�Ƕ�
  chassis_move.chassis_yaw = rad_format(INS_data.angle_yaw - gimbal_control.gimbal_yaw_motor.relative_angle);
  chassis_move.chassis_pitch = rad_format(INS_data.angle_pitch - gimbal_control.gimbal_pitch_motor.relative_angle);
  chassis_move.chassis_roll = INS_data.angle_roll;
}

/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @retval         none
 */

static void chassis_set_contorl(void)
{
  chassis_behaviour_control_set(&chassis_move);
}
////////////////���ID/////////////////
/////////1********ǰ*******2///////////
/////////*                 *///////////
/////////*        x        *///////////
/////////��       ����y      ��//////////
/////////*                 *///////////
/////////*                 *///////////
/////////4********��********3//////////
///////////////////////////////////////
/**
 * @brief          �ĸ�ȫ�����ٶ���ͨ�������������������
 * @param[in]      vx_set: �����ٶ�
 * @param[in]      vy_set: �����ٶ�
 * @param[in]      wz_set: ��ת�ٶ�
 * @param[out]     wheel_speed: �ĸ�ȫ�����ٶ�
 * @param[in]			 WHEEL_RADIUS ȫ���ְ뾶
 * @retval         none

 */
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
  wheel_speed[0] = WHEEL_RADIUS * (-SQRT2 * vx_set - SQRT2 * vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set);
  wheel_speed[1] = WHEEL_RADIUS * (-SQRT2 * vx_set + SQRT2 * vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set);
  wheel_speed[2] = WHEEL_RADIUS * (SQRT2 * vx_set + SQRT2 * vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set);
  wheel_speed[3] = WHEEL_RADIUS * (SQRT2 * vx_set - SQRT2 * vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set);
//  wheel_speed[0] = 1.5;
//  wheel_speed[1] = -1.5;
//  wheel_speed[2] = -1.5;
//  wheel_speed[3] = 1.5;
}

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(void)
{
  float max_vector = 0.0f, vector_rate = 0.0f;
  float temp = 0.0f;
  float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;

  //�����˶��ֽ�
  chassis_vector_to_mecanum_wheel_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed);
  //	if(wheel_speed[i]<1.5||wheel_speed[i]>-1.5){
  //		chassis_move.motor_chassis[i].give_current=0;
  //	}

  if (chassis_move.chassis_mode == CHASSIS_OPEN || chassis_move.chassis_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_move.motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
    // raw����ֱ�ӷ���
    return;
  }

  //�������ӿ�������ٶȣ�������������ٶ�
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].speed_set = wheel_speed[i];
    temp = fabs(chassis_move.motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_move.motor_chassis[i].speed_set *= vector_rate;
    }
  }

  //����PID

  // if (chassis_move.chassis_pitch < -1.30)
  // {
  //   for (i = 0; i < 2; i++)
  //   {
  //     PID_calc_2(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, chassis_move.motor_chassis[i].speed_set);
  //   }
  //   for (i = 2; i < 4; i++)
  //   {
  //     PID_calc_3(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, chassis_move.motor_chassis[i].speed_set);
  //   }
  // }
  // else
  // {
  for (i = 0; i < 4; i++)
  {
    PID_calc(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, chassis_move.motor_chassis[i].speed_set);
  }
  // }
	//int robot_level=2;
  //���ʿ���
  //chassis_power_control(&chassis_move);
  //printf("%d,%d,%d,%d",a,b,c,d);
  //��ֵ����ֵ
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].give_current = (int16_t)(chassis_move.motor_speed_pid[i].out);
  }
}
void chassis_data_send(void)
{
  static uint8_t send_time;
  //ÿ5���뷢��һ��  500Hz
  if (send_time == 50)
  {
    // if (robot_state.robot_level == 1)
    //   super_cap_send_power(5000);
    // if (robot_state.robot_level == 2)
    //   super_cap_send_power(7000);
    // if (robot_state.robot_level == 3)
    //   super_cap_send_power(9000);
    super_cap_send_power(robot_state.chassis_power_limit * 100);
    send_time = 0;
  }
  send_time++;
}


//static void chassis_level_power(void)
//{
//	switch(robot_state.robot_level)
//	{
//		case 1:{CHASSIS_MOTOR_SPEED_MAX_OUT=1800;NORMAL_MAX_CHASSIS_SPEED_X=1.4;}break;//2000//1.0//ǰ��60.8w//����70.3//1800
//		case 2:{CHASSIS_MOTOR_SPEED_MAX_OUT=1900;NORMAL_MAX_CHASSIS_SPEED_X=1.4;}break;//ֱ��60.8//75
//		case 3:{CHASSIS_MOTOR_SPEED_MAX_OUT=2100;NORMAL_MAX_CHASSIS_SPEED_X=1.5;}break;//2250//1.0//2.0(���ҹ���81)//1.5(����81ֱ��75)
//		case 4:{CHASSIS_MOTOR_SPEED_MAX_OUT=2300;NORMAL_MAX_CHASSIS_SPEED_X=1.5;}break;//  1.3ֱ��76//����81.7                 �������Ͽ��Կ�����С���ݣ�
//		case 5:{CHASSIS_MOTOR_SPEED_MAX_OUT=2500;NORMAL_MAX_CHASSIS_SPEED_X=1.5;}break;//1.3ֱ��77.9//87.5
//		case 6:{CHASSIS_MOTOR_SPEED_MAX_OUT=2600;NORMAL_MAX_CHASSIS_SPEED_X=1.5;}break;//1.3ֱ��85.5//����93.1
//		case 7:{CHASSIS_MOTOR_SPEED_MAX_OUT=2700;NORMAL_MAX_CHASSIS_SPEED_X=1.5;}break;//ֱ��90   //����95
//		case 8:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=1.6;}break;//ֱ��93��99//����100
//		case 9:{CHASSIS_MOTOR_SPEED_MAX_OUT=3200;NORMAL_MAX_CHASSIS_SPEED_X=1.6;}break;//ֱ��99��105//����106
//		case 10:{CHASSIS_MOTOR_SPEED_MAX_OUT=4000;NORMAL_MAX_CHASSIS_SPEED_X=1.6;}break;//ֱ��114//����121.6
//		default:break;
//		
////#define CHASSIS_MOTOR_SPEED_MAX_OUT //��3200(1.2)//��2600��1.3��//��2100��1.1��//��2750(1.2)//��2250(1.0)//��2300��1.2��//��2500��1.2��//��3000(1.3)//ʮ4000(1.5)//һ2000//7000//14000  //15000
////#define NORMAL_MAX_CHASSIS_SPEED_X 1.2f//2.0f//1.0f//2.0f//3.0//1.0f//2.0f

//	}
//}
