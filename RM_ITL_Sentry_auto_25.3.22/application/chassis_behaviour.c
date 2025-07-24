/**
 * @file chassis_behaviour.c
 * @author 何清华
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
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘小陀螺的行为状态机下，底盘模式是一边旋转一边以云台指向方向运动
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);//,auto_move_t *auto_move

/**
 * @brief          底盘模式选择
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
static int up2 = 0, medium2 = 0, down2 = 0; //分别对应小陀螺，单轴模式，跟随模式
float angle_set_user = 0.0f;                //一键掉头，90度，目标set记录值
float angle_set = 0.0f;                     // 45度跟随方向值
float f = 1.0;                              //调整全速小陀螺
int n = 0;                                  //判断小陀螺标志位，若n为0，则半速小陀螺，若n为1，则全速小陀螺
extern ext_game_robot_state_t robot_state;  //步兵等级
int B_flag = 0, B_flag_last = 0;            // B按键上升沿下降沿判断标志位
int F_flag = 0, F_flag_last = 0;            // F按键上升沿下降沿判断标志位
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

    //遥控器设置模式(非必要不建议改)
    if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //上
    {
        #ifdef AUTO_DEBUG
            chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
        #else
            chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_TOP
        #endif
    }
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_Z||switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //中
    {//测试模式
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;//CHASSIS_FOLLOW_GIMBAL;
				auto_move_flag=AUTO_IN_HOME;
    }
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_G||switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //下
    {
			chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
			if(auto_move_flag==AUTO_CHASSIS_TOP)//小陀螺
			{
				chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_TOP;//CHASSIS_NO_MOVE;
			}
			else if(auto_move_flag==AUTO_IN_HOME)
			{
				chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
			}
			else//移动
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
			if(auto_move_flag==AUTO_CHASSIS_TOP)//小陀螺
			{
				chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_TOP;//CHASSIS_NO_MOVE;
			}
			else if(auto_move_flag==AUTO_IN_HOME)
			{
				chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
			}
			else//移动
			{
				chassis_move_mode->chassis_mode = CHASSIS_TOP;//CHASSIS_FOLLOW_GIMBAL;3.22
			}
	}
    //当云台在某些模式下，像初始化， 底盘不动
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;//CHASSIS_NO_MOVE;
    }
  
//    if (toe_is_error(DBUS_TOE))
//    {
//        chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;//CHASSIS_TOP，CHASSIS_ZERO_FORCE
//        if(blind_walk_flag==2)
//        {
//            chassis_move_mode->chassis_mode = CHASSIS_TOP;
//        }
		  if (toe_is_error(DBUS_TOE))
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//CHASSIS_TOP，CHASSIS_ZERO_FORCE，CHASSIS_NO_FOLLOW_GIMBAL
//        if(blind_walk_flag==2)
//        {
//            chassis_move_mode->chassis_mode = CHASSIS_TOP;
//        }
    }

    if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_F) )//F键小陀螺
   {untop_flag=1;}
    
    if((rc_ctrl.key & KEY_PRESSED_OFFSET_F)&&untop_flag)     //小陀螺模式   (上) 
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
    
  /****************************V 小陀螺快速旋转************************************************************/
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
  /***********************小陀螺速度：慢-快-停***************************************/
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
  /***************************** 小陀螺自动分级 **************************************************************************/
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

  /***************************** SHIFT + Q 一键调头 ******************************************************************/
   Shift_Q_flag_last = Shift_Q_flag;
   Shift_Q_flag = ((rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT) && rc_ctrl.key & KEY_PRESSED_OFFSET_Q);
   if (Shift_Q_flag - Shift_Q_flag_last == 1)
   {
     gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + PI;
   }
  /********************************* Q 左转90 ******************************************************/
  Q_flag_last = Q_flag;
  Q_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_Q);
  if (Q_flag - Q_flag_last == 1)
  {
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + PI / 2.0;
  }
  /***************************************** E 右转90******************************************************/
  E_flag_last = E_flag;
  E_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_E);
  if (E_flag - E_flag_last == 1)
  {
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set - PI / 2.0;
  }
  /*************************************************** R 45度跟随开关 ******************************************************/
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
  /**************************************开电容**********************************************************************/
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
  /**************************************关电容**********************************************************************/
  if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL]))
  {
    up2 = 0;
    medium2 = 0;
    down2 = 0;
    B_flag = 0, B_flag_last = 0; // B按键上升沿下降沿判断标志位
    F_flag = 0, F_flag_last = 0; // F按键上升沿下降沿判断标志位
    Shift_Q_flag = 0, Shift_Q_flag_last = 0;
    Q_flag = 0, Q_flag_last = 0;
    E_flag = 0, E_flag_last = 0;
    R_flag = 0, R_flag_last = 0;
  }

    if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_G))
         {unfollow_flag=1;}	
        
    if((rc_ctrl.key & KEY_PRESSED_OFFSET_G)&&unfollow_flag)     //单轴模式(下)   G键云台不跟随
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
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;//C键云台跟随
        down2=0;
        up2=0;
        medium2=1;
    }

    if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_F) )//F键小陀螺
    {untop_flag=1;}
     
    if((rc_ctrl.key & KEY_PRESSED_OFFSET_F)&&untop_flag)     //小陀螺模式   (上) 
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
    chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//CHASSIS_TOP; //小陀螺
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_Z||medium2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;//CHASSIS_FOLLOW_GIMBAL; //跟随模式
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_G||down2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL; //单轴模式


}
/**
 * @brief          设置控制量.根据不同底盘控制模式，会调用不同的控制函数.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
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
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
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
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f;

  //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
  sin_yaw = arm_sin_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  chassis_move_rc_to_vector->vx_set = vx_set ;//cos_yaw * vx_set ;//- sin_yaw * vy_set;
  chassis_move_rc_to_vector->vy_set = vy_set;//cos_yaw * vy_set;//sin_yaw * vx_set + cos_yaw * vy_set;

  //设置控制相对云台角度
  chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(angle_set)+2.761932f-3.14f;///2;///4*3;
  //chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(PI/4);
  chassis_move_rc_to_vector->chassis_relative_angle = rad_format(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle - GIMBAL_YAW_OFFSET_ECD);

  //计算旋转PID角速度
  chassis_move_rc_to_vector->wz_set = -PID_calc(&chassis_move_rc_to_vector->chassis_angle_pid, chassis_move_rc_to_vector->chassis_relative_angle, chassis_move_rc_to_vector->chassis_relative_angle_set);
  

  //速度限幅
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief          底盘小陀螺的行为状态机下，底盘模式是一边旋转一边以云台指向方向运动
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

  //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
  sin_yaw = arm_sin_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(GIMBAL_YAW_OFFSET_ECD - chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  chassis_move_rc_to_vector->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
  chassis_move_rc_to_vector->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

  //设置小陀螺转速
  if ((vx_set == 0 && vy_set == 0 )|| rc_ctrl.key & KEY_PRESSED_OFFSET_F)
  {
    chassis_move_rc_to_vector->wz_set = f * CHASSIS_TOP_SPEED;
  }
  else
  {
    //平移时小陀螺速度变慢
    chassis_move_rc_to_vector->wz_set = 0.7 * f * CHASSIS_TOP_SPEED;//0.7 old data 
  }
  // chassis_move_rc_to_vector->vx_set += 0.01;
  //速度限幅（小陀螺时移动速度变慢）
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->top_min_speed, chassis_move_rc_to_vector->top_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->top_min_speed, chassis_move_rc_to_vector->top_max_speed);
}
/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
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
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
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
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
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
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */

float vx_set_channel_q;
float vy_set_channel_q;
int auto_move_flag=AUTO_IN_HOME;//初始化
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
	else//无导航数据
	{
	  //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
		rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
		vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
		vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
	}



//vx_set_channel
  //键盘控制
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
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_S) //后退
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

  if (rc_ctrl.key & KEY_PRESSED_OFFSET_A) //左移
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
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_D) //右移
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

  // // //一阶低通滤波代替斜坡作为底盘速度输入（平滑）
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
  //在死区内，停止信号，不需要缓慢加速，直接减速到零
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
