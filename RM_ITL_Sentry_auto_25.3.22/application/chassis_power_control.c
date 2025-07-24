/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"

#include "stdio.h"
#include "usart.h"

float POWER_LIMIT;        //��������
float WARNING_POWER;      //���湦��
float WARNING_POWER_BUFF; //���滺������

 float volt=0.0f;
 float current=0.0f;
 float chassis_power = 0.0f;   				//���̹���
 float chassis_power_buffer = 0.0f;   //���̻�������
// #define POWER_LIMIT 60.0f
// #define WARNING_POWER 40.0f
// #define WARNING_POWER_BUFF 50.0f
extern ext_game_robot_state_t robot_state;
#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 16000.0f
#define POWER_TOTAL_CURRENT_LIMIT 20000.0f
float k = 0.0f; //ϵ��
float k_speed = 0.0f;
float k_speed_level = 0.0f;
float limit_k = 0.5f;
float powerbuffer_const = 0.0f;
int Chassis_Maxspeed_Level = 1;
int Chassis_Maxspeed_Level_last = 1;
int Chassis_Maxspeed_Warning_Flag1 = 0;
int Chassis_Maxspeed_Warning_Flag2 = 0;
int Chassis_Maxspeed_Warning_Flag3 = 0;

uint8_t robot_level;
uint8_t a;
uint8_t b;
uint8_t c;
uint8_t d;
//#define a;
//#define b;
//#define c;
//#define d;

/**
 * @brief          limit the power, mainly limit motor current
 * @param[in]      chassis_power_control: chassis data
 * @retval         none
 */
/**
 * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
 * @param[in]      chassis_power_control: ��������
 * @retval         none
 */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
//	robot_state.robot_level = 2;
    if (robot_state.robot_level == 1||robot_state.robot_level == 0) // 1��
    {
			//a++;
        k_speed_level = 1.0f;
        POWER_LIMIT = 60.0f;
        WARNING_POWER = 40.0f;
        WARNING_POWER_BUFF = 50.0f;
    }
    if (robot_state.robot_level == 2) // 2��
    {
			b++;
        k_speed_level = 2.0f;
        POWER_LIMIT = 80.0f;
        WARNING_POWER = 50.0f;
        WARNING_POWER_BUFF = 60.0f;
    }
    if (robot_state.robot_level == 3) // 3��
    {
			c++;
        k_speed_level = 1.4f;
        POWER_LIMIT = 100.0f;
        WARNING_POWER = 60.0f;
        WARNING_POWER_BUFF = 80.0f;
    }

    if (rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)
    {
			d++;
        chassis_power_control->vx_max_speed = 2.0;
        chassis_power_control->vx_min_speed = -2.0;
        chassis_power_control->vy_max_speed = 2.0;
        chassis_power_control->vy_min_speed = -2.0;
    }
    if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
    {
        chassis_power_control->vx_max_speed = 1.0;
      chassis_power_control->vx_min_speed = -1.0;
         chassis_power_control->vy_max_speed = 1.0;
         chassis_power_control->vy_min_speed = -1.0;
     }
     else
     {
         if (PowerData[1] > 22 && !Chassis_Maxspeed_Warning_Flag1)
         {
             k = 0.1f;			//0.1
             // k_speed = 1.0f;
             Chassis_Maxspeed_Level = 1;
         }
         else if ((PowerData[1] > 22 && Chassis_Maxspeed_Warning_Flag1) || (PowerData[1] > 19 && PowerData[1] <= 22 && !Chassis_Maxspeed_Warning_Flag2))
         {
             k = 0.2f;		//0.2
             // k_speed = 1.0f;
             Chassis_Maxspeed_Level = 2;
         }
         else if ((PowerData[1] > 19 && PowerData[1] <= 22 && Chassis_Maxspeed_Warning_Flag2) || (PowerData[1] > 17 && PowerData[1] <= 19 && !Chassis_Maxspeed_Warning_Flag3))
         {
             k = 0.5f;			//0.5
             // k_speed = 0.5f;
             Chassis_Maxspeed_Level = 3;
         }
         else if ((PowerData[1] > 17 && PowerData[1] <= 19 && Chassis_Maxspeed_Warning_Flag3) || (PowerData[1] <= 17))
         {
             k = 0.7f;		//0.7
             // k_speed = 0.2f;
         }
    chassis_power_control->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X * k_speed_level;
    chassis_power_control->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X * k_speed_level;
    chassis_power_control->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y * k_speed_level;
    chassis_power_control->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y * k_speed_level;

         chassis_power_control->motor_speed_pid[0].out *= k;
         chassis_power_control->motor_speed_pid[1].out *= k;
         chassis_power_control->motor_speed_pid[2].out *= k;
         chassis_power_control->motor_speed_pid[3].out *= k;
         if (Chassis_Maxspeed_Level == 1 && Chassis_Maxspeed_Level_last == 2)
             Chassis_Maxspeed_Warning_Flag1 = 1;
         if (PowerData[1] > 22.5f)
             Chassis_Maxspeed_Warning_Flag1 = 0;

         if (Chassis_Maxspeed_Level == 2 && Chassis_Maxspeed_Level_last == 3)
             Chassis_Maxspeed_Warning_Flag2 = 1;
         if (PowerData[1] > 20.5f)
             Chassis_Maxspeed_Warning_Flag2 = 0;

         if (Chassis_Maxspeed_Level == 3 && Chassis_Maxspeed_Level_last == 4)
             Chassis_Maxspeed_Warning_Flag3 = 1;
         if (PowerData[1] > 18.0f)
             Chassis_Maxspeed_Warning_Flag3 = 0;

         Chassis_Maxspeed_Level_last = Chassis_Maxspeed_Level;
     }

    float chassis_power = 0.0f;
    float chassis_power_buffer = 0.0f;
    float total_current_limit = 0.0f;
    float total_current = 0.0f;
    uint8_t robot_id = get_robot_id();

    if (robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        get_chassis_power_and_buffer(&current,&volt,&chassis_power, &chassis_power_buffer);
//                  // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
//                  //���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
//                  if (chassis_power_buffer < WARNING_POWER_BUFF)
//                  {
//                      float power_scale;
//                      if (chassis_power_buffer > 5.0f)
//                      {
//                          // scale down WARNING_POWER_BUFF
//                          //��СWARNING_POWER_BUFF
//                          power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
//                      }
//                      else
//                      {
//                          // only left 10% of WARNING_POWER_BUFF
//                          power_scale = 5.0f / WARNING_POWER_BUFF;
//                      }
//                      // scale down
//                      //��С
//                      total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
//                  }
//                  else
//                  {
//                      // power > WARNING_POWER
//                      //���ʴ���WARNING_POWER

//                      if (chassis_power > WARNING_POWER)
//                      {
//                          float power_scale;
//                          // power < 80w
//                          //����С��80w
//                          if (chassis_power < POWER_LIMIT)
//                          {
//                              // scale down
//                              //��С
//                              power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
//                          }
//                          // power > 80w
//                          //���ʴ���80w
//                          else
//                          {
//                              power_scale = 0.0f;
//                          }

//                          total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
//                      }
//                      // power < WARNING_POWER
//                      //����С��WARNING_POWER
//                      else
//                      {
//                          total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
//                      }
//                  }
//              

//              total_current = 0.0f;
//              // calculate the original motor current set
//              //����ԭ����������趨
//                  for (uint8_t i = 0; i < 4; i++)
//                  {
//                      total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
//                  }

//                  if (total_current > total_current_limit)
//                  {
//                      float current_scale = total_current_limit / total_current;
//                      chassis_power_control->motor_speed_pid[0].out *= current_scale;
//                      chassis_power_control->motor_speed_pid[1].out *= current_scale;
//                      chassis_power_control->motor_speed_pid[2].out *= current_scale;
//                      chassis_power_control->motor_speed_pid[3].out *= current_scale;
//                  }
             /**************************************�ϰ湦������****************************************************************************/

        //�жϻ��������Ƿ�λ��60J��250J֮��
        if (chassis_power_buffer > 60.0f && chassis_power_buffer <= 250.0f)
            powerbuffer_const = chassis_power_buffer;
        else
            powerbuffer_const = 60.0f; //���򻺳�����Ϊ60J
        limit_k = 0.25f * ((float)chassis_power_buffer / powerbuffer_const) * ((float)chassis_power_buffer / powerbuffer_const) *
                      ((float)chassis_power_buffer / powerbuffer_const) +
                  0.5f * ((float)chassis_power_buffer / powerbuffer_const) *
                      ((float)chassis_power_buffer / powerbuffer_const) +
                  0.25f * ((float)chassis_power_buffer / powerbuffer_const);
        chassis_power_control->motor_speed_pid[0].out *= limit_k;
        chassis_power_control->motor_speed_pid[1].out *= limit_k;
        chassis_power_control->motor_speed_pid[2].out *= limit_k;
        chassis_power_control->motor_speed_pid[3].out *= limit_k;
									
    }
}
