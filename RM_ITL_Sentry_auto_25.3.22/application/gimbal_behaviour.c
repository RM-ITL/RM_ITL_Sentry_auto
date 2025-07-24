/**
 * @file gimbal_behaviour.c
 * @author ���廪
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "user_lib.h"
#include "USART_receive.h"
#include "config.h"
#include "filter.h"
#include "chassis_behaviour.h"
#include "pid.h"
#include "INS_task.h"
#include "referee.h"

//����̨��У׼, ���÷�����Ƶ�ʺ�ǿ��
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
int16_t rc_x, rc_y;
extern chassis_move_t chassis_move;
extern ext_game_robot_state_t robot_state;
extern auto_move_t auto_move;
extern struct SolveTrajectoryParams st;
/**
 * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
 * @param          �����ң����ֵ
 * @param          ��������������ң����ֵ
 * @param          ����ֵ
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
 * @brief          ��̨��Ϊ״̬������.
 * @param[in]      gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
 *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
 * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨��������ģʽ��ͨ����λ�����͵����ݿ���
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

//��̨��Ϊ״̬��,Ĭ����̨����ģʽ
gimbal_behaviour_e gimbal_behaviour = GIMBAL_AUTO;//GIMBAL_ZERO_FORCE;
/**
 * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
 * @param[out]     gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //��̨��Ϊ״̬������
    gimbal_behavour_set(gimbal_mode_set);

    //������̨��Ϊ״̬�����õ��״̬��
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {                                                                                                                                                                                                                                                                                                                                      
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_AUTO)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    
}

/**
 * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
 * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
 * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
 * @param[in]      gimbal_mode_set:��̨����ָ��
 * @retval         none
 */
int16_t yaw_count=0;//2024.3.31
int16_t add_dir=0;
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint8_t last_behaviour = GIMBAL_ZERO_FORCE;//GIMBAL_ZERO_FORCE;
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO)
    {
        gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);
    }
    
    if (last_behaviour != gimbal_behaviour)
    {
        gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;//gimbal_control_set->gimbal_pitch_motor.absolute_angle_set;
        gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
        if(gimbal_behaviour==GIMBAL_AUTO)
        {
          gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;//gimbal_control_set->gimbal_pitch_motor.absolute_angle_set;//����̨����ģʽ����̨����ģʽ����ʼ��pitch��Ƕ�
					 //gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
            yaw_count=0;
            add_dir=1;
            
        }
			  yaw_error_sum = 0.0f;
        yaw_last_error = 0.0f;
        yaw_d_filter = 0.0f;
        pitch_error_sum = 0.0f;
        pitch_last_error = 0.0f;
        pitch_d_filter = 0.0f;
    }

    last_behaviour = gimbal_behaviour;
}

/**
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS||gimbal_behaviour ==GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          ��̨��Ϊ״̬������.
 * @param[in]      gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
#define AUTO_DEBUG
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    //ң�������ؿ��� ��̨״̬(�Ǳ�Ҫ�������)
    if (switch_is_up(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //��
    {
        #ifdef AUTO_DEBUG
            gimbal_behaviour = GIMBAL_AUTO;
        #else
            gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
        #endif
    }
    else if (switch_is_mid(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //��
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;//GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (switch_is_down(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //��
    {
        gimbal_behaviour = GIMBAL_AUTO;//GIMBAL_ABSOLUTE_ANGLE;//GIMBAL_ZERO_FORCE;
    }
    
    //ң�����رտ�����̨����ģʽ
    if (toe_is_error(DBUS_TOE)) 
    {
        //gimbal_behaviour = GIMBAL_AUTO;//GIMBAL_AUTO;//GIMBAL_ABSOLUTE_ANGLE;//GIMBAL_ZERO_FORCE;
				gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    
    //����Ҽ�������̨����ģʽ
    if (rc_ctrl.mouse.press_r != 0)
    {
        gimbal_behaviour = GIMBAL_AUTO;
    }

}

/**
 * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
 *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
 * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */

float rc_x_q,rc_y_q;
    
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    rc_x = rc_ctrl.mouse.x;
    rc_y = rc_ctrl.mouse.y;
  
    rc_x_q=rc_x_q*0.93+0.07*rc_x;
    rc_y_q=rc_y_q*0.93+0.07*rc_y;
    
    *yaw = yaw_channel * YAW_RC_SEN - rc_x_q * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + rc_y_q * PITCH_MOUSE_SEN;

}

/**
 * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    *yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + rc_ctrl.mouse.y * PITCH_MOUSE_SEN;
}

/**
 * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}


uint8_t find_target=0;
uint8_t target_distance=0;
uint16_t auto_shoot_wait=0;
float pitch_dir=0.00015;//0.0003
uint8_t permiss_shoot_yaw = 0;
uint8_t permiss_shoot_pitch = 0;
//int16_t yaw_count=0;//2024.3.31
//int16_t add_dir=0;

float h = 0;

static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
     //static float auto_yaw;
     //static float auto_pitch;
    float h = 0;
//    //����̨��������
		if(game_begin_flag==1)
		{
			gimbal_data_send();
		}
    

    if(find_target==1)//��Ұ����Ŀ�꣬�Ӿ�����
    {
        auto_shoot_wait=300;//1000
    }
		else if(find_target==0&&auto_move_flag==AUTO_CHASSIS_TOP)//�Ҳ���Ŀ��
    {
//			if(yaw_count==0)
//			{      
//					add_dir = 1;
//			}
//			else if(yaw_count==4500)//2500//4500//6000
//			{
//					add_dir = -1;
//			}
//			 yaw_count=yaw_count+add_dir;
//			*yaw = -0.0005*add_dir;//0.0006//0.0008//0.0006//0.0003f;(3.15)//0.0003��0.0005��0.0008 0.0012f,0.0005f
			*yaw=0;
			*pitch=0;
     }
    
//    if(0) //(toe_is_error(USER_USART_DATA_TOE))
//    {
//        *yaw = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//        *pitch = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//    }
    if(auto_shoot_wait>0)//ѭ��500�Σ�
    {
         permiss_shoot_yaw=0;
         permiss_shoot_pitch=0;
        if(robot_state.robot_id==7)//�ҷ�Ϊ���ڱ���ʶ����ɫ
        {                
            switch(target_distance)
            {
                case 0: h=0.0f; break;//0.0f
                case 1: h=0.0f; break;//0.0f
                case 2: h=0.00075f; break;//0.00075f
                case 3: h=0.0015f; break;//0.0015f
                case 4: h=0.002f; break;//0.002f
                case 5: h=0.0024f; break;//0.0023f
                case 6: h=0.0025f; ////0.00233f//auto_shoot.pitch_add /1000��break;
                case 7: h=0.0033f; break;//0.003f
                case 8: h=0.0039f;break;//0.0035f
                default:break;
            }
        }
        else if(robot_state.robot_id==107)//�ҷ�Ϊ���ڱ���ʶ���ɫ
        {
            switch(target_distance)
            {
                case 0: h=0.0f; break;//0.0f
                case 1: h=0.00011f; break;//0.0f
                case 2: h=0.00081f; break;//0.00075f
                case 3: h=0.00156f; break;//0.0015f
                case 4: h=0.00176f; break;//0.002f
                case 5: h=0.00192f; break;//0.0023f
                case 6: h=0.00244f; ////0.00233f//auto_shoot.pitch_add /1000��break;
                case 7: h=0.00333f; break;//0.003f
                case 8: h=0.00384f;break;//0.0035f
                default:break;
            }
        }
        else//ͬʶ���ɫ��������
        {
            switch(target_distance)
            {
                case 0: h=0.0f; break;//0.0f
                case 1: h=0.0f; break;//0.0f
                case 2: h=0.00075f; break;//0.00075f
                case 3: h=0.0015f; break;//0.0015f
                case 4: h=0.0023f; break;//0.002f
                case 5: h=0.0025f; break;//0.0023f
                case 6: h=0.0028f; ////0.00233f//auto_shoot.pitch_add /1000��break;
                case 7: h=0.0036f; break;//0.003f
                case 8: h=0.004f;break;//0.0035f
                default:break;
            }
        }
        if(find_target==1)
        {
						if( auto_shoot.yaw_add >=2.5-0.25||auto_shoot.yaw_add <=-2.5-0.25)//����ƫ��-0.5
						{
							*yaw = auto_shoot.yaw_add*-0.000087f;//0.0005f;
						}
            else if(auto_shoot.yaw_add==0)
            {
               //*yaw=0.00006;
							 *yaw=0.0;
               *pitch=0; 
            }
            else////����׼Ŀ��
            {
                *yaw = 0.0f;
                permiss_shoot_yaw=1;
            }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						if( auto_shoot.pitch_add >=2||auto_shoot.pitch_add <=-2)
						{
							*pitch = auto_shoot.pitch_add*-0.00005f;
						}
           else if(auto_shoot.pitch_add==0)
            {
               *yaw=0.0;//0.00006;
               *pitch=0; 
            }
            else  //����׼Ŀ��
            {
                *pitch=0.0f;
                permiss_shoot_pitch=1;
            }
        }
        else if(find_target==0&&auto_move_flag==AUTO_CHASSIS_TOP)//�Ҳ���Ŀ����Ը����һ���ٶ�ת
        {
					*yaw=0.0;
					*pitch=0.0;
        }
				else//����·��
				{
					*pitch=0;
				}
        auto_shoot_wait--;
    }
		
		
		
    if(auto_move_flag==AUTO_MOVE)//��·
    {
			if(auto_move.auto_wz>0)
			{
				*yaw=0.000001f;
			}
			else if(auto_move.auto_wz<0)
			{
				*yaw=-0.000001f;
			}
			else
			{
				*yaw=0.0f;
        *pitch=0.0f;
			}
						
    }
						//        *yaw = 0.00f;//0.0003f;//0.0003��0.0005��0.0008 0.0012f,0.0005f
////        *pitch=0.0f;
//        if(INS_data.angle_pitch>0.35)//pitch�������˶�
//        {
//           pitch_dir=-0.0003f;//0.0003f,0.00015f
//        }
//        else if(INS_data.angle_pitch<-0.05)//0.05, -0.14
//        {
//             pitch_dir=0.0003f;
//        }
//         *pitch = pitch_dir;
//        *yaw = 0.0006f;//0.0003f;(3.15)//0.0003��0.0005��0.0008 0.0012f,0.0005f
        
//        if(yaw_count==0)
//        {      
//            add_dir = 1;
//        }
//        else if(yaw_count==4500)//2500//4500//6000
//        {
//            add_dir = -1;
//        }
//         yaw_count=yaw_count+add_dir;
//        *yaw = 0.0005*add_dir;//0.0006//0.0008//0.0006//0.0003f;(3.15)//0.0003��0.0005��0.0008 0.0012f,0.0005f
//        *pitch=0.0f;
//					*yaw=auto_move.auto_wz;

}