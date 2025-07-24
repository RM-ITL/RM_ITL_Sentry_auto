/**
 * @file chassis_behaviour.c
 * @author ���廪
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "config.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "detect_task.h"
#include "USART_receive.h"

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }
extern auto_move_t auto_move;
/**
 * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ����С���ݵ���Ϊ״̬���£�����ģʽ��һ����תһ������ָ̨�����˶�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     vy_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
 * @retval         none
 */
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);//,auto_move_t *auto_move

/**
 * @brief          ����ģʽѡ��
 * @param[in]      chassis_move_mode: ��������
 * @retval         none
 */
static int up2 = 0, medium2 = 0, down2 = 0; //�ֱ��ӦС���ݣ�����ģʽ������ģʽ
float angle_set_user = 0.0f;                //һ����ͷ��90�ȣ�Ŀ��set��¼ֵ
float angle_set = 0.0f;                     // 45�ȸ��淽��ֵ
float f = 1.0;                              //����ȫ��С����
int n = 0;                                  //�ж�С���ݱ�־λ����nΪ0�������С���ݣ���nΪ1����ȫ��С����
extern ext_game_robot_state_t robot_state;  //�����ȼ�
int B_flag = 0, B_flag_last = 0;            // B�����������½����жϱ�־λ
int F_flag = 0, F_flag_last = 0;            // F�����������½����жϱ�־λ
int Shift_Q_flag = 0, Shift_Q_flag_last = 0;
int Q_flag = 0, Q_flag_last = 0;
int E_flag = 0, E_flag_last = 0;
int R_flag = 0, R_flag_last = 0;
int V_flag = 0, V_flag_last = 0;


int untop_flag=0;
int unfollow_flag=0;

int up2_last = 0;
float vx_set_use = 0.0;
float vy_set_use = 0.0;
float yaw_angle_last = 0.0;
extern uint16_t draw_init_flag;

// extern int16_t c_switch;
//extern uint8_t blind_walk_flag;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //ң��������ģʽ(�Ǳ�Ҫ�������)
    if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //��
    {
        #ifdef AUTO_DEBUG
            chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        #else
            chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_TOP
        #endif
    }
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_Z||switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //��
    {//����ģʽ
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;//CHASSIS_FOLLOW_GIMBAL;
				auto_move_flag=AUTO_IN_HOME;
    }
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_G||switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //��
    {
			chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
			if(auto_move_flag==AUTO_CHASSIS_TOP)//С����
			{
				chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_TOP;//CHASSIS_NO_MOVE;
			}
			else if(auto_move_flag==AUTO_IN_HOME)
			{
				chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
			}
			else//�ƶ�
			{
				chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_FOLLOW_GIMBAL;3.22
			}
    }
    
    if(up2==1)      chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//CHASSIS_TOP;
    if(medium2==1)  
		{
			chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
			auto_move_flag=AUTO_IN_HOME;
		}
		//CHASSIS_FOLLOW_GIMBAL;
    if(down2==1)    
	{	
			chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
			if(auto_move_flag==AUTO_CHASSIS_TOP)//С����
			{
				chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_TOP;//CHASSIS_NO_MOVE;
			}
			else if(auto_move_flag==AUTO_IN_HOME)
			{
				chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
			}
			else//�ƶ�
			{
				chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_FOLLOW_GIMBAL;3.22
			}
	}
    //����̨��ĳЩģʽ�£����ʼ���� ���̲���
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;//CHASSIS_NO_MOVE;
    }
  
//    if (toe_is_error(DBUS_TOE))
//    {
//        chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;//CHASSIS_TOP��CHASSIS_ZERO_FORCE
//        if(blind_walk_flag==2)
//        {
//            chassis_move_mode->chassis_mode = CHASSIS_TOP;
//        }
		  if (toe_is_error(DBUS_TOE))
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//CHASSIS_TOP��CHASSIS_ZERO_FORCE��CHASSIS_NO_FOLLOW_GIMBAL
//        if(blind_walk_flag==2)
//        {
//            chassis_move_mode->chassis_mode = CHASSIS_TOP;
//        }
    }

    if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_F) )//F��С����
   {untop_flag=1;}
    
    if((rc_ctrl.key & KEY_PRESSED_OFFSET_F)&&untop_flag)     //С����ģʽ   (��) 
    {
        untop_flag=0;
    
        if(up2==1)
        {
            up2=0;
            down2=0;
            medium2=1;
        }
        else
        { 
            up2=1;
            down2=0;
            medium2=0;
        }
    }
    
  /****************************V С���ݿ�����ת************************************************************/
  F_flag_last = F_flag;
  F_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_V);
  if (F_flag - F_flag_last == 1)
  {
    if (up2 == 1 && medium2 == 0 && down2 == 0)
    {
      if (n == 1)
      {
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        up2 = 0;
        medium2 = 1;
        down2 = 0;
        n = 0;
      }
      else
      {
        chassis_move_mode->chassis_mode = CHASSIS_TOP;
        up2 = 1;
        medium2 = 0;
        down2 = 0;
        n = 1;
      }
    }
    else
    {
      chassis_move_mode->chassis_mode = CHASSIS_TOP;
      up2 = 1;
      medium2 = 0;
      down2 = 0;
      n = 1;
      gimbal_control.gimbal_yaw_motor.absolute_angle_set -= PI / 106.5;
    }
  }
  // if (chassis_move_mode->chassis_mode == CHASSIS_TOP)
  /***********************С�����ٶȣ���-��-ͣ***************************************/
  //   if (up2 == 1 && medium2 == 0 && down2 == 0)
  //   {
  //     if (n == 0)
  //     {
  //       chassis_move_mode->chassis_mode = CHASSIS_TOP;
  //       up2 = 1;
  //       medium2 = 0;
  //       down2 = 0;
  //       n = 1;
  //     }
  //     else if (n == 1)
  //     {
  //       chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
  //       up2 = 0;
  //       medium2 = 1;
  //       down2 = 0;
  //       n = 0;
  //       gimbal_control.gimbal_yaw_motor.absolute_angle_set = yaw_angle_last;
  //       // gimbal_control.gimbal_yaw_motor.absolute_angle_set -= PI / 150;
  //     }
  //   }
  //   else
  //   {
  //     chassis_move_mode->chassis_mode = CHASSIS_TOP;
  //     up2 = 1;
  //     medium2 = 0;
  //     down2 = 0;
  //     n = 0;
  //     yaw_angle_last = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
  //     gimbal_control.gimbal_yaw_motor.absolute_angle_set -= PI / 106.5;
  //   }
  // }
  /***************************** С�����Զ��ּ� **************************************************************************/
  if (chassis_move_mode->chassis_mode == CHASSIS_TOP)
  {
    if (robot_state.robot_level == 1 & n == 0)
      f = 0.8;
    if (robot_state.robot_level == 2 & n == 0)
      f = 1.0;
    if (robot_state.robot_level == 3 & n == 0)
      f = 1.5;
    if (robot_state.robot_level == 1 & n == 1)
      f = 1;
    if (robot_state.robot_level == 2 & n == 1)
      f = 1.2;
    if (robot_state.robot_level == 3 & n == 1)
      f = 1.5;
  }
  // if (chassis_move_mode->chassis_mode == CHASSIS_TOP)
  // {
  //   if (robot_state.robot_level == 1)
  //     f = 1;
  //   if (robot_state.robot_level == 2)
  //     f = 1.3;
  //   if (robot_state.robot_level == 3)
  //     f = 1.7;
  // }

  /***************************** SHIFT + Q һ����ͷ ******************************************************************/
   Shift_Q_flag_last = Shift_Q_flag;
   Shift_Q_flag = ((rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT) && rc_ctrl.key & KEY_PRESSED_OFFSET_Q);
   if (Shift_Q_flag - Shift_Q_flag_last == 1)
   {
     gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + PI;
   }
  /********************************* Q ��ת90 ******************************************************/
  Q_flag_last = Q_flag;
  Q_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_Q);
  if (Q_flag - Q_flag_last == 1)
  {
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + PI / 2.0;
  }
  /***************************************** E ��ת90******************************************************/
  E_flag_last = E_flag;
  E_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_E);
  if (E_flag - E_flag_last == 1)
  {
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set - PI / 2.0;
  }
  /*************************************************** R 45�ȸ��濪�� ******************************************************/
  R_flag_last = R_flag;
  R_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_R);
  if (R_flag - R_flag_last == 1)
  {
    if (up2 != 1)
    {
      if (angle_set == PI / 4)//PI / 4),PI / 2
      {
        angle_set = 0;
      }
      else if (up2 != 1)
      {
        angle_set = PI / 4;//PI / 4;PI / 2
      }
    }
  }
  /**************************************������**********************************************************************/
  // V_flag_last = V_flag;
  // V_flag = (rc_ctrl.key & KEY_PRESSED_OFFSET_V);
  // if (R_flag - R_flag_last == 1)
  // {
  //   if (c_switch == 8)
  //   {
  //     c_switch = 7;
  //   }
  //   else
  //   {
  //     c_switch = 8;
  //   }
  // }

  // if (rc_ctrl.key & KEY_PRESSED_OFFSET_V)
  // {
  //   c_switch = 8;
  // }
  // if (rc_ctrl.key == 16416)
  // {
  //   c_switch = 7;
  // }
  /**************************************�ص���**********************************************************************/
  if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL]))
  {
    up2 = 0;
    medium2 = 0;
    down2 = 0;
    B_flag = 0, B_flag_last = 0; // B�����������½����жϱ�־λ
    F_flag = 0, F_flag_last = 0; // F�����������½����жϱ�־λ
    Shift_Q_flag = 0, Shift_Q_flag_last = 0;
    Q_flag = 0, Q_flag_last = 0;
    E_flag = 0, E_flag_last = 0;
    R_flag = 0, R_flag_last = 0;
  }

    if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_G))
         {unfollow_flag=1;}	
        
    if((rc_ctrl.key & KEY_PRESSED_OFFSET_G)&&unfollow_flag)     //����ģʽ(��)   G����̨������
    {
            unfollow_flag=0;
        
            if(down2==1)
            {
                 up2=0;
                 down2=0;
                 medium2=1;
            }
            else
            {
                up2=0;
                down2=1;
                medium2=0;
            
            }
    }

    if(rc_ctrl.key & KEY_PRESSED_OFFSET_C)     
    {
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;//C����̨����
        down2=0;
        up2=0;
        medium2=1;
    }

    if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_F) )//F��С����
    {untop_flag=1;}
     
    if((rc_ctrl.key & KEY_PRESSED_OFFSET_F)&&untop_flag)     //С����ģʽ   (��) 
    {
          untop_flag=0;
        
            if(up2==1)
            {
               up2=0;
               down2=0;
               medium2=1;
            }
        else
        {
            
            up2=1;
            down2=0;
            medium2=0;
            
        }
    }


  if ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_G)
  {
    draw_init_flag = 1;
  }
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_F||up2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//CHASSIS_TOP; //С����
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_Z||medium2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//CHASSIS_FOLLOW_GIMBAL; //����ģʽ
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_G||down2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL; //����ģʽ


}
/**
 * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ������ò�ͬ�Ŀ��ƺ���.
 * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
 * @retval         none
 */
void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_MOVE)
  {
    chassis_no_move_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
  {
    chassis_follow_gimbal_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_TOP)
  {
    chassis_top_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL)
  {
    chassis_no_follow_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_OPEN)
  {
    chassis_open_set_control(chassis_move_rc_to_vector);
  }
  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_zero_force_control(chassis_move_rc_to_vector);
  }
}

/**
 * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f;

  //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
  sin_yaw = arm_sin_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  chassis_move_rc_to_vector->vx_set = vx_set ;//cos_yaw * vx_set ;//- sin_yaw * vy_set;
  chassis_move_rc_to_vector->vy_set = vy_set;//cos_yaw * vy_set;//sin_yaw * vx_set + cos_yaw * vy_set;

  //���ÿ��������̨�Ƕ�
  chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(angle_set)+2.761932f-3.14f;///2;///4*3;
  //chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(PI/4);
  chassis_move_rc_to_vector->chassis_relative_angle = rad_format(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle - GIMBAL_YAW_OFFSET_ECD);

  //������תPID���ٶ�
  chassis_move_rc_to_vector->wz_set = -PID_calc(&chassis_move_rc_to_vector->chassis_angle_pid, chassis_move_rc_to_vector->chassis_relative_angle, chassis_move_rc_to_vector->chassis_relative_angle_set);
  

  //�ٶ��޷�
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief          ����С���ݵ���Ϊ״̬���£�����ģʽ��һ����תһ������ָ̨�����˶�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

  //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
  sin_yaw = arm_sin_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  chassis_move_rc_to_vector->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
  chassis_move_rc_to_vector->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

  //����С����ת��
  if ((vx_set == 0 && vy_set == 0 )|| rc_ctrl.key & KEY_PRESSED_OFFSET_F)
  {
    chassis_move_rc_to_vector->wz_set = f * CHASSIS_TOP_SPEED;
  }
  else
  {
    //ƽ��ʱС�����ٶȱ���
    chassis_move_rc_to_vector->wz_set = 0.7 * f * CHASSIS_TOP_SPEED;//0.7 old data 
  }
  // chassis_move_rc_to_vector->vx_set += 0.01;
  //�ٶ��޷���С����ʱ�ƶ��ٶȱ�����
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->top_min_speed, chassis_move_rc_to_vector->top_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->top_min_speed, chassis_move_rc_to_vector->top_max_speed);
}
/**
 * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  wz_set = CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL];

  chassis_move_rc_to_vector->wz_set = wz_set;
  chassis_move_rc_to_vector->vx_set = float_constrain(vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_move_rc_to_vector->vx_set = rc_ctrl.rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->vy_set = -rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->wz_set = -rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  return;
}

/**
 * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
 *
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     vy_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
 * @retval         none
 */

float vx_set_channel_q;
float vy_set_channel_q;
int auto_move_flag=AUTO_IN_HOME;//��ʼ��
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector)//2025.1.10,auto_move_t *auto_move
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  float vx_set_channel, vy_set_channel;
	if(auto_move_flag==AUTO_MOVE)//
	{
	  vx_set_channel=auto_move.auto_vx;
		vy_set_channel=auto_move.auto_vy;
	}
	else//�޵�������
	{
	  //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
		rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
		vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
		vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
	}



//vx_set_channel
  //���̿���
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_W)
  {
    if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
    {
      if (vx_set_channel < chassis_move_rc_to_vector->vx_max_speed)
      {
        vx_set_use += 0.001f;
        vx_set_channel = vx_set_use;
      }
      else
      {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
      }
    }
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)
    {
      vx_set_channel = chassis_move_rc_to_vector->vx_max_speed * 1.5;
    }
    else
    {
      vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
  }
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_S) //����
  {
    if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
    {
      if (vx_set_channel > chassis_move_rc_to_vector->vx_min_speed)
      {
        vx_set_use -= 0.001f;
        vx_set_channel = vx_set_use;
      }
      else
      {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
      }
    }
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)
    {
      vx_set_channel = chassis_move_rc_to_vector->vx_min_speed * 1.5;
    }
    else
    {
      vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }
  }
  else
  {
    vx_set_use = 0.0f;
  }

  if (rc_ctrl.key & KEY_PRESSED_OFFSET_A) //����
  {
    if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
    {
      if (vy_set_channel < chassis_move_rc_to_vector->vy_max_speed)
      {
        vy_set_use += 0.003f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
      }
    }
    else
    {
      if (vy_set_channel < chassis_move_rc_to_vector->vy_max_speed)
      {
        vy_set_use += 0.01f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
      }
    }
  }
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_D) //����
  {
    if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
    {
      if (vy_set_channel > chassis_move_rc_to_vector->vy_min_speed)
      {
        vy_set_use -= 0.003f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
      }
    }
    else
    {
      if (vy_set_channel > chassis_move_rc_to_vector->vy_min_speed)
      {
        vy_set_use -= 0.01f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
      }
    }
  }
  else
  {
    vy_set_use = 0.0f;
  }

  // // //һ�׵�ͨ�˲�����б����Ϊ�����ٶ����루ƽ����
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
  //�������ڣ�ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

//   *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
//   *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
  vx_set_channel_q=vx_set_channel_q*0.95+0.05*vx_set_channel;
  vy_set_channel_q=vy_set_channel_q*0.95+0.05*vy_set_channel;
  *vx_set = vx_set_channel_q;
  *vy_set = vy_set_channel_q;
}
