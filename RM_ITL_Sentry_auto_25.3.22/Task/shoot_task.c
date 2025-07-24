/**
 * @file shoot_task.c
 * @author 何清华
 * @brief 包含对摩擦轮以及拨盘的控制
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "shoot_task.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "stdio.h"
#include "cmsis_os.h"

#include "bsp_laser.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee.h"
#include "chassis_behaviour.h"
#include "usart.h"
#include <stdio.h>

//#ifdef __GNUC__
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif

// PUTCHAR_PROTOTYPE
//{
//   HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//×èÈû·½Ê½´òÓ¡,´®¿Ú1
//   return ch;
// }

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

float speed[5];
int flag_servo = 0;
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_supply_projectile_booking_t supply_projectile_booking_t;
extern float this_angle_207;

float error_207[2];
int X_flag = 0;
int X_flag_last = 0;                               // x按键上升沿下降沿判断标志位
int press_left_flag = 0, press_left_flag_last = 0; //鼠标左键标志位
int servo_flag = 0;
static uint8_t move_flag = 0;



int heat_flag; 
int flagxl=0;
int flag_2=0;
int flag_3=0;
int medium1=0;
int down1=0;
/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_init(void);

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
static void shoot_control_loop(void);

/**
 * @brief          堵转倒转处理
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void);

static void servo_init();

//射击数据
shoot_control_t shoot_control;
//调整发射速度
float k_shoot_speed = 1.0;

//发送的电机电流
static int16_t left_friction_can_set_current = 0, right_friction_can_set_current = 0, trigger_can_set_current = 0;

extern uint8_t permiss_shoot_yaw ;
extern uint8_t permiss_shoot_pitch ;

void fire_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(SHOOT_TASK_INIT_TIME);

  //发射机构初始化
  shoot_init();
  servo_init();

  //判断摩擦轮电机是否都在线
  while (toe_is_error(LEFT_FRICTION_MOTOR_TOE) || toe_is_error(RIGHT_FRICTION_MOTOR_TOE))
  {
    vTaskDelay(SHOOT_CONTROL_TIME);
  }
//       printf("left=%d", shoot_control.left_fricition_motor.given_current);
//      printf("right=%d\n", shoot_control.right_fricition_motor.given_current);

  extern int i, j, kk, m, nn;
  extern uint16_t this_time_rx_len;

  while (1)
  {
    //射击模式选择
    shoot_set_mode();

    //发射机构数据更新
    shoot_feedback_update();
    //射击控制量计算
    shoot_control_loop();
    speed[0] = shoot_control.left_fricition_motor.speed * 10;
    speed[1] = shoot_control.left_fricition_motor.speed_set * 10;
    speed[2] = -shoot_control.right_fricition_motor.speed * 10;
    speed[3] = shoot_control.right_fricition_motor.speed_set * 10;

    left_friction_can_set_current = shoot_control.left_fricition_motor.given_current;
    right_friction_can_set_current = shoot_control.right_fricition_motor.given_current;
    trigger_can_set_current = shoot_control.trigger_motor.given_current;

    if (!(toe_is_error(LEFT_FRICTION_MOTOR_TOE) || toe_is_error(RIGHT_FRICTION_MOTOR_TOE)))
    {
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_friction(0, 0);
        CAN_cmd_shoot(0);
//            CAN_cmd_friction(left_friction_can_set_current, right_friction_can_set_current);
//            CAN_cmd_shoot(trigger_can_set_current);
      }
      else
      {
        CAN_cmd_friction(left_friction_can_set_current, right_friction_can_set_current);
        CAN_cmd_shoot(trigger_can_set_current);
      }
    }

    vTaskDelay(SHOOT_CONTROL_TIME);
  }
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
  shoot_control.shoot_mode = SHOOT_STOP;
  //电机指针
  shoot_control.trigger_motor.motor_measure = get_trigger_motor_measure_point();
  shoot_control.left_fricition_motor.motor_measure = get_left_friction_motor_measure_point();
  shoot_control.right_fricition_motor.motor_measure = get_right_friction_motor_measure_point();

  //初始化PID
  PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KP, TRIGGER_KI, TRIGGER_KD, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT);

  PID_init(&shoot_control.left_fricition_motor.motor_speed_pid, FRICTION_KP_L, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //左摩擦轮

  PID_init(&shoot_control.right_fricition_motor.motor_speed_pid, FRICTION_KP, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //右摩擦轮
  //更新数据
  shoot_feedback_update();
  //拨弹电机
  shoot_control.trigger_motor.ecd_count = 0;
  shoot_control.trigger_motor.angle = shoot_control.trigger_motor.motor_measure->angle;
  shoot_control.trigger_motor.given_current = 0;
  shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
  shoot_control.trigger_motor.speed = 0.0f;
  shoot_control.trigger_motor.speed_set = 0.0f;
  //左摩擦轮
  shoot_control.left_fricition_motor.accel = 0.0f;
  shoot_control.left_fricition_motor.speed = 0.0f;
  shoot_control.left_fricition_motor.speed_set = 0.0f;
  shoot_control.left_fricition_motor.given_current = 0;
  //右摩擦轮
  shoot_control.right_fricition_motor.accel = 0.0f;
  shoot_control.right_fricition_motor.speed = 0.0f;
  shoot_control.right_fricition_motor.speed_set = 0.0f;
  shoot_control.right_fricition_motor.given_current = 0;
}

/**
 * @brief          射击状态机设置，遥控器上拨1次发射数颗，一直处在上，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
// int medium1 = 0;
// int down1 = 0;
int shoot_count=0;
static void shoot_set_mode(void)
{
    press_left_flag_last = press_left_flag;
    press_left_flag = (shoot_control.press_l);

     /***********获取热量，达到上限停止发***********/
    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);//实际heat_limit为：400，

    //热量限制
     
    if ((!toe_is_error(REFEREE_TOE)) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if (shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET||shoot_control.shoot_mode == SHOOT_READY)
        {
            shoot_control.shoot_mode = SHOOT_READY;
            heat_flag=0;
        }
    }
    else
    {
        heat_flag=1;
    }

    //处于上档，进入发射模式
    if (heat_flag&&switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_BULLET;
    }
    //处于中档，开启摩擦轮
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT||switch_is_mid(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_READY;
        //如果处于中档并按下鼠标左键，进入发射模式
        //if (press_left_flag - press_left_flag_last == 1&&heat_flag)
        if (!((press_left_flag - press_left_flag_last == 1)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)) flagxl=1;   // 
      //if (((shoot_control.press_l)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&flagxl&&heat_flag)			
        if (((press_left_flag - press_left_flag_last == 1)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&flagxl&&heat_flag)
        {
            flagxl=0;
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
        else
        {
            shoot_control.shoot_mode = SHOOT_READY;
            if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //关闭摩擦轮
            {
                 shoot_control.shoot_mode = SHOOT_STOP;
                 medium1=0;
                 down1=1;
            }
        }
				if( permiss_shoot_yaw ==1 && permiss_shoot_pitch== 1)
        {
            shoot_count = 1000;//5000;//500;//500,1000
        }
        
        if( shoot_count >0)
         {
            if(heat_flag)
            {
                 shoot_control.shoot_mode = SHOOT_BULLET;
            }
            shoot_count--;
         }
    }
  //处于下档，关闭摩擦轮
    else if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL||switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
        //shoot_control.shoot_mode =SHOOT_READY;
        move_flag = 0;
        //if(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)    //开启拨盘
        if((rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)||medium1==1)
        {
            shoot_control.shoot_mode = SHOOT_READY;	
            if (!((press_left_flag - press_left_flag_last == 1)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)) flagxl=1;   // 
            if (((press_left_flag - press_left_flag_last == 1)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&flagxl&&heat_flag)    //
            {
                flagxl=0;
                shoot_control.shoot_mode = SHOOT_BULLET;
            //shoot_bullet_control();
            }
            else
            {
                shoot_control.shoot_mode = SHOOT_READY;
                if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //关闭摩擦轮
                {
                     shoot_control.shoot_mode = SHOOT_STOP;
                     medium1=0;
                     down1=1;
                }
            }
        }
        if( permiss_shoot_yaw ==1 && permiss_shoot_pitch== 1)
        {
            shoot_count = 1000;//5000;//500;//500,1000
        }
        
        if( shoot_count >0)
         {
            if(heat_flag)
            {
                 shoot_control.shoot_mode = SHOOT_BULLET;
            }
            shoot_count--;
         }
    }
//    else
//    {
//      shoot_control.shoot_mode = SHOOT_READY;
//    }
    
    //遥控器或者左拨杆置上开启摩擦轮，满足条件射击 云台自瞄模式下的射击
//    if (!toe_is_error(DBUS_TOE)||switch_is_up(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //toe_is_error(DBUS_TOE)||
//    {
//        shoot_control.shoot_mode = SHOOT_READY;
//        
//        if( permiss_shoot_yaw ==1 && permiss_shoot_pitch== 1)
//        {
//            shoot_count = 2000;//500,1000
//        }
//        
//        if( shoot_count >0)
//         {
//            if(heat_flag)
//            {
//                 shoot_control.shoot_mode = SHOOT_BULLET;
//            }
//            shoot_count--;
//         }
//    }
    
    
    

        /*       键盘控制       */

    if(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT||medium1==1)    //开启拨盘
    {
        shoot_control.shoot_mode = SHOOT_READY;
        medium1=1;
        down1=0;
       //如果处于中档并按下鼠标左键，进入发射模式
       if(shoot_control.press_l&&heat_flag)
    //if (medium1=1&&KEY_PRESSED_OFFSET_CTRL)
       {
            shoot_control.shoot_mode = SHOOT_BULLET;
       }
    }
//  if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //关闭摩擦轮//CTRL
//  {
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //关闭摩擦轮//CTRL
    {
         shoot_control.shoot_mode = SHOOT_STOP;
         medium1=0;
         down1=1;
    }
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) {medium1=0;down1=0;} 
            
    
    //鼠标长按一直进入射击状态 保持连发
    if (((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))&&heat_flag)//&&heat_flag
    {
        shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        move_flag = 0;
    }
    /*********************************** 一级发射速度 ******************************************************/
    if (robot_state.robot_level == 10 || robot_state.robot_level == 0)//哨兵默认10级
    {
        k_shoot_speed = 1.153;//1;
    }

    /*********************************** 二级三级发射速度 ******************************************************/
    else if (robot_state.robot_level == 2 || robot_state.robot_level == 3)
    {
        k_shoot_speed = 1.153;
    }


    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    // if (supply_projectile_booking_t.supply_robot_id == robot_state.robot_id || rc_ctrl.key & KEY_PRESSED_OFFSET_X)
    X_flag_last = X_flag;
    X_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_X);
    if (X_flag - X_flag_last == 1)
    {
        if (servo_flag == 0)
        {
          __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 4700); //?180 2000
          servo_flag = 1;
        }
        else
        {
          __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 800); //?0
          servo_flag = 0;
        }
    }
}

/**
 * @brief 射击数据更新
 *@param[in] void
 *@retval void
 */
static void shoot_feedback_update(void)
{
  shoot_control.trigger_motor.angle = shoot_control.trigger_motor.motor_measure->angle;

  shoot_control.trigger_motor.speed = shoot_control.trigger_motor.motor_measure->speed_rpm;
  //左摩擦轮
  shoot_control.left_fricition_motor.speed = shoot_control.left_fricition_motor.motor_measure->speed_rpm;
  //右摩擦轮
  shoot_control.right_fricition_motor.speed = shoot_control.right_fricition_motor.motor_measure->speed_rpm;

  //鼠标按键
  shoot_control.last_press_l = shoot_control.press_l;
  shoot_control.last_press_r = shoot_control.press_r;
  shoot_control.press_l = rc_ctrl.mouse.press_l;
  shoot_control.press_r = rc_ctrl.mouse.press_r;
  //长按计时
  if (shoot_control.press_l)
  {
    if (shoot_control.press_l_time < PRESS_LONG_TIME)
    {
      shoot_control.press_l_time++;
    }
  }
  else
  {
    shoot_control.press_l_time = 0;
  }

  if (shoot_control.press_r)
  {
    if (shoot_control.press_r_time < PRESS_LONG_TIME)
    {
      shoot_control.press_r_time++;
    }
  }
  else
  {
    shoot_control.press_r_time = 0;
  }

  //射击开关时间计时
  if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
  {
    if (shoot_control.rc_s_time < RC_S_LONG_TIME)
    {
      shoot_control.rc_s_time++;
    }
  }
  else
  {
    shoot_control.rc_s_time = 0;
  }
}

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
static void shoot_control_loop(void)
{
  if (shoot_control.shoot_mode == SHOOT_STOP)
  {
    //激光关闭
    shoot_laser_off();
    //设置拨弹轮的速度
    shoot_control.trigger_motor.speed_set = 0.0f;
    shoot_control.left_fricition_motor.speed_set = 0.0f;
    shoot_control.right_fricition_motor.speed_set = 0.0f;

    //设置加速减速电流限制
    shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
    shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY)
  {
    //设置拨弹轮的速度
    shoot_control.trigger_motor.speed_set = 0.0f;
    //摩擦轮缓启动
    if (shoot_control.right_fricition_motor.speed > (FRICTION_SPEED_SET * 0.8f * k_shoot_speed))
    {
      shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
      shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
    }
    //摩擦轮转速设置
    shoot_control.left_fricition_motor.speed_set = -FRICTION_SPEED_SET * k_shoot_speed;
    shoot_control.right_fricition_motor.speed_set = FRICTION_SPEED_SET * k_shoot_speed;
  }
  else if (shoot_control.shoot_mode == SHOOT_BULLET)
  {
    //单发控制
    shoot_bullet_control();
    //设置拨弹轮的拨动速度,并开启堵转反转处理
    shoot_control.trigger_motor.speed_set = TRIGGER_SPEED;
    trigger_motor_turn_back();
  }
  else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
  {
    //设置拨弹轮的拨动速度,并开启堵转反转处理
    shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
    trigger_motor_turn_back();
    move_flag = 0;
  }
  //判断单发标志位
  if (move_flag == 1)
  {
    if (shoot_control.trigger_motor.angle - shoot_control.trigger_motor.set_angle > 0)
    {
      //没到达一直设置旋转速度
      shoot_control.trigger_motor.speed_set = TRIGGER_SPEED;
      //开启反堵转处理
      trigger_motor_turn_back();
    }
    else
    {
      move_flag = 0;
    }
  }

  if (1)//(shoot_control.shoot_mode != SHOOT_STOP)
  {
    shoot_laser_on(); //激光开启
  }

  //计算拨弹轮电机PID
  PID_calc(&shoot_control.trigger_motor.motor_speed_pid, shoot_control.trigger_motor.speed, shoot_control.trigger_motor.speed_set);
  shoot_control.trigger_motor.given_current = (int16_t)(shoot_control.trigger_motor.motor_speed_pid.out);
  // if (shoot_control.shoot_mode <= SHOOT_READY)
  // {
  //   shoot_control.trigger_motor.given_current = 0;
  // }
  //计算摩擦轮电机PID
  PID_calc(&shoot_control.left_fricition_motor.motor_speed_pid, shoot_control.left_fricition_motor.speed, shoot_control.left_fricition_motor.speed_set);
  shoot_control.left_fricition_motor.given_current = (int16_t)(shoot_control.left_fricition_motor.motor_speed_pid.out);

  PID_calc(&shoot_control.right_fricition_motor.motor_speed_pid, shoot_control.right_fricition_motor.speed, shoot_control.right_fricition_motor.speed_set);
  shoot_control.right_fricition_motor.given_current = (int16_t)(shoot_control.right_fricition_motor.motor_speed_pid.out);
}

/**
 * @brief 卡弹检测及反转处理
 *
 */
static void trigger_motor_turn_back(void)
{
  if (shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.trigger_motor.speed_set = shoot_control.trigger_motor.speed_set;
  }
  else
  {
    shoot_control.trigger_motor.speed_set = -shoot_control.trigger_motor.speed_set;
  }

  if (fabs(shoot_control.trigger_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.block_time++;
    shoot_control.reverse_time = 0;
  }
  else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
  {
    shoot_control.reverse_time++;
  }
  else
  {
    shoot_control.block_time = 0;
  }
}

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{
  //每次拨动一定角度
  if (move_flag == 0)
  {
    shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle - 1300.0f;
    move_flag = 1;
  }
}

static void servo_init()
{
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  //  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 700); // 660 1600
	
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,4700);
}
