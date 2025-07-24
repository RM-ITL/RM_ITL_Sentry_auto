/**
 * @file gimbal_task.c/h
 * @author ���廪,������
 * @brief ��̨���������߳�
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include "main.h"
#include "INS_task.h"
#include "arm_math.h"
#include "detect_task.h"
#include "user_lib.h"
#include "USART_receive.h"
#include "chassis_behaviour.h"
#include "referee.h"
#include "bsp_led.h"

//#include "usart.h"
//#include <stdio.h>

//#ifdef __GNUC__
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif

//PUTCHAR_PROTOTYPE
//{
//  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//������ʽ��ӡ,����1
//  return ch;
//}

//float gimbal_yaw_old=0.0f;
//float gimbal_yaw_speed_set;
//float err;

//float yaw;
//  float  yaw_use;   
//  float yaw_old;   
//  float yaw_f; 

#define debug_mode 1
#define MOUSE_SENSE 2.5f
// PID�������
extern chassis_move_t chassis_move;
extern ext_game_state_t game_state;
uint8_t game_begin_flag=0;
#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }
/**
 * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void gimbal_init(void);

/**
 * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void gimbal_set_mode(void);

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @retval         none
 */
static void gimbal_feedback_update(void);

/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
 * @retval         none
 */
static void gimbal_set_control(void);

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @retval         none
 */
static void gimbal_control_loop(void);

/**
 * @brief ��̨��̬���ݷ��͵���λ��
 *
 */
void gimbal_data_send(void);
void aRGB_led_show(uint32_t aRGB);
/**
 * @brief          ��GIMBAL_MOTOR_GYROģʽ�����ƽǶ��趨,��ֹ�������
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          ��GIMBAL_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
 
// static void gimbal_yaw_speed_control(gimbal_motor_t *gimbal_yaw_moter);
 
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_auto_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief ��̨PID��ʼ������
 *
 * @param pid PID���ݽṹָ��
 * @param kp
 * @param ki
 * @param kd
 * @param maxout ������
 * @param max_iout ���������
 */
static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float maxout, float max_iout);

/**
 * @brief ��̨PID���㺯��
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta ���΢���ֱ�ӴӴ�������ȡ��������
 * @return fp32
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta);

/**
 * @brief          ��̨PID��������pid��out,iout
 * @param[out]     pid_clear:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);

//��̨���������������
gimbal_control_t gimbal_control;

float gimbal_debug[10] = {0};

//���͵ĵ������
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
extern gimbal_behaviour_e gimbal_behaviour;

//uint16_t blind_walk_mode_count=0;
//uint8_t blind_walk_flag=0;

void gimbal_task(void const *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //��̨��ʼ��
    gimbal_init();
	  aRGB_led_show(0xFF0000FF);
		auto_move_flag=AUTO_IN_HOME;

    //�жϵ���Ƿ�����
    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))

    vTaskDelay(GIMBAL_CONTROL_TIME);
    
    //��̨���ݸ���
    //gimbal_feedback_update(); 
    //uint16_t blind_walk_wait_count=100;//�������ȴ�3000��3m��//�����ص�15000��15s��
    //uint16_t blind_walking_count=1;//1800//�������3600��2m���ң�//������1800��1m��
    //uint8_t game_begin_flag=0;

    while (1)
    {
        //1.������̨����ģʽ
        gimbal_set_mode();
        
         if(toe_is_error(DBUS_TOE))
        {
            //blind_walk_mode_count=0;
            //blind_walk_flag=0;
					game_begin_flag=0;
					find_target=0;
					auto_shoot.yaw_add=0.0f;
					auto_shoot.pitch_add=0.0f;
					gimbal_init();
					auto_move_flag=AUTO_IN_HOME;
					gimbal_control.gimbal_pitch_motor.relative_angle_set = 0.15;//gimbal_control.gimbal_pitch_motor.relative_angle;
        }
//        if(game_state.game_progress==3&&game_state.stage_remain_time==0)//game_state.game_progress==3&&
//        {
//            game_begin_flag=1;
//        }
				if(1)
        {
            game_begin_flag=1;
        }

        //2.��̨��ǰ�Ƕȣ����ٶ����ݸ���
        gimbal_feedback_update();
        
        //3.��ȡ��̨������
        gimbal_set_control();

        //4.��̨����PID����
        gimbal_control_loop();

        //5.��̨��̬���ݷ��͵���λ��(����gimbal_behaviour��ʹ��)
        //gimbal_data_send();

        if (chassis_move.chassis_mode == CHASSIS_TOP)
        {
            //��ʼ��yaw���pid
            gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_tuolo, YAW_ABSOLUTE_ANGLE_KP_TUOLUO, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD_TUOLUO, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
            PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
        }
//				else if (gimbal_behaviour == GIMBAL_AUTO)
//        {
//            //��ʼ��yaw���pid
//            gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_zimiao, YAW_ABSOLUTE_ANGLE_KP_zimiao , YAW_ABSOLUTE_ANGLE_KI_zimiao, YAW_ABSOLUTE_ANGLE_KD_zimiao, YAW_ABSOLUTE_ANGLE_MAX_OUT_zimiao, YAW_ABSOLUTE_ANGLE_MAX_IOUT_zimiao); //�����ǽǶȿ���
//            PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP_zimiao, YAW_GYRO_KI_zimiao, YAW_GYRO_KD_zimiao, YAW_GYRO_MAX_OUT_zimiao, YAW_GYRO_MAX_IOUT_zimiao);
//        }
        else
        {
            gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
            PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
        }

        //��������װ�����෴����Ҫ�ڵ���ֵǰ��Ӹ���
        #if YAW_OUT_TURN
            yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
        #else
            yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        #endif
        #if PITCH_OUT_TURN
            pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
        #else
            pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
        #endif
         
         //������̨�ļ�����
        if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
        {
            if (toe_is_error(DBUS_TOE))
            {
              CAN_cmd_gimbal(0, 0);
            //  //��ֹʧ��
              gimbal_control.gimbal_pitch_motor.absolute_angle_set = 0.254;
              gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
                //CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current);	
            }
            else
            {
                CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current);	
            }
        }

        #if debug_mode
            if (toe_is_error(YAW_GIMBAL_MOTOR_TOE))  
            {
                gimbal_debug[7] = 5;
            }
            else if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
            {
                gimbal_debug[7] = -5;
            }
            gimbal_debug[0] = gimbal_control.gimbal_pitch_motor.absolute_angle;
            gimbal_debug[1] = gimbal_control.gimbal_pitch_motor.absolute_angle_set;
            gimbal_debug[2] = gimbal_control.gimbal_yaw_motor.absolute_angle;
            gimbal_debug[3] = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
            gimbal_debug[4] = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->given_current;
            gimbal_debug[5] = auto_shoot.pitch_add;
            gimbal_debug[6] = auto_shoot.yaw_add;
            gimbal_debug[8] = gimbal_control.gimbal_yaw_motor.relative_angle;
        #endif
            
        vTaskDelay(GIMBAL_CONTROL_TIME);
    }
}

/**
 * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void gimbal_init(void)
{
    //�������ָ���ȡ
    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    gimbal_control.gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //��ʼ�����ģʽ
    gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = gimbal_control.gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;//2024.2.28,����
    gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = gimbal_control.gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //��ʼ����̨��ֵ����λ
    gimbal_control.gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ECD;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_ECD;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_ECD;

    gimbal_control.gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_OFFSET_ECD;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_ECD;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_ECD;

    //��ʼ��yaw���pid
    gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
    PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
    //��ʼ��pitch���pid
    gimbal_PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_ABSOLUTE_ANGLE_KP, PITCH_ABSOLUTE_ANGLE_KI, PITCH_ABSOLUTE_ANGLE_KD, PITCH_ABSOLUTE_ANGLE_MAX_OUT, PITCH_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
    PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_GYRO_KP, PITCH_GYRO_KI, PITCH_GYRO_KD, PITCH_GYRO_MAX_OUT, PITCH_GYRO_MAX_IOUT);

    //�������PID���
    gimbal_total_pid_clear(&gimbal_control);
    //���ݸ���
    gimbal_feedback_update();
    //��ʼ������״̬
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
    gimbal_control.gimbal_yaw_motor.relative_angle_set = gimbal_control.gimbal_yaw_motor.relative_angle;
    gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_control.gimbal_yaw_motor.motor_gyro;

    gimbal_control.gimbal_pitch_motor.absolute_angle_set = 0.254;//0.3;//-0.1;//
    gimbal_control.gimbal_pitch_motor.relative_angle_set = gimbal_control.gimbal_pitch_motor.relative_angle;
    gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_control.gimbal_pitch_motor.motor_gyro;

    //��ʼ����������ָ��
    user_send_data.gimbal_pitch_angle = &gimbal_control.gimbal_pitch_motor.absolute_angle;
    user_send_data.gimbal_yaw_gyro = &gimbal_control.gimbal_yaw_motor.motor_gyro;

}

/**
 * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void gimbal_set_mode()
{
    gimbal_behaviour_mode_set(&gimbal_control);
}

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @retval         none
 */
static void gimbal_feedback_update()
{
    //��̨���ݸ���
    //����������Ƕȱ仯����������ǽǶȱ仯�����෴����Ҫ����ԽǶ�ǰ��Ӹ���
    #if PITCH_TURN//pitch��
        gimbal_control.gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_pitch_motor.offset_ecd);
    #else
        gimbal_control.gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_pitch_motor.offset_ecd);
    #endif
        gimbal_control.gimbal_pitch_motor.absolute_angle = INS_data.angle_pitch;
        gimbal_control.gimbal_pitch_motor.motor_gyro = INS_data.wy;

    #if YAW_TURN//yaw��
        gimbal_control.gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offset_ecd);
    #else
        gimbal_control.gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offset_ecd);
    #endif
  //gimbal_control.gimbal_yaw_motor.absolute_angle = INS_data.angle_yaw;
    gimbal_control.gimbal_yaw_motor.absolute_angle = my_yaw;
    gimbal_control.gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wz - arm_sin_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wx;

}

/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > 4096)
    {
        relative_ecd -= 8191;
    }
    else if (relative_ecd < -4096)
    {
        relative_ecd += 8191;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
 * @retval         none
 */
// int x;2

extern auto_shoot_t auto_shoot;
extern auto_move_t auto_move;
uint16_t first_go=0;
static void gimbal_set_control(void)
{
    static float add_yaw_angle = 0.0f;
    static float add_pitch_angle = 0.0f;
    static float auto_yaw_target = 0.0f;
    static float auto_pitch_target = 0.0f;

    // yaw
//    if (gimbal_behaviour == GIMBAL_AUTO)
//    {
////        if(1)
////        {
////            if(first_go<1000)//����������ֹ��reset������̨����ģʽ��ת���ٽ�ֵδ֪
////            { 
////                 auto_yaw_target = gimbal_control.gimbal_yaw_motor.absolute_angle;
////                 auto_pitch_target = gimbal_control.gimbal_pitch_motor.absolute_angle;
////								auto_yaw_target =0;
////								auto_pitch_target = 0;
////								//gimbal_control.gimbal_pitch_motor.absolute_angle_set =-0.03;
////                 first_go+=1;
////            }
////            else
////            {
//                gimbal_behaviour_control_set(&auto_yaw_target, &auto_pitch_target, &gimbal_control);
////            }
//            //auto_yaw_target = gimbal_control.gimbal_yaw_motor.absolute_angle;
//            //auto_pitch_target = gimbal_control.gimbal_pitch_motor.absolute_angle;
//						gimbal_control.gimbal_yaw_motor.absolute_angle_set   = gimbal_control.gimbal_yaw_motor.absolute_angle_set  + auto_yaw_target;
//            //gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle_set+ auto_pitch_target;
//            gimbal_absolute_angle_limit(&gimbal_control.gimbal_pitch_motor, auto_pitch_target); //���ԽǶȿ����޷�
////        }
//    }
//    else
//    {
        auto_yaw_target = gimbal_control.gimbal_yaw_motor.absolute_angle;
        auto_pitch_target = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, &gimbal_control);
        gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + add_yaw_angle;//�ص�����
        //gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle_set + add_pitch_angle;//�ǲ�������һ��pitch�ľ��ԽǶ�:û�٣�gimbal_absolute_angle_limit���м��㸳ֵ
        gimbal_absolute_angle_limit(&gimbal_control.gimbal_pitch_motor, add_pitch_angle); //���ԽǶȿ����޷�

//    }


}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
    static float bias_angle;
    static float angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }

//    //��ʼλ������
//    if (gimbal_motor->absolute_angle_set>0.42||gimbal_motor->absolute_angle_set<-0.5)
//    {
//        gimbal_motor->absolute_angle_set = 0.3;
//    }

    //��ǰ�������Ƕ�
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� �����������е�Ƕ�
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //�����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)////3.21
        {
          //�����һ��������ӽǶȣ�
          add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
          add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //�Ƿ񳬹���� ��Сֵ
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_control_loop()
{
    // yaw
    if (gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_control.gimbal_yaw_motor.given_current = 0;
    }
//		else if(gimbal_behaviour == GIMBAL_AUTO)
//		{
//			gimbal_motor_auto_control(&gimbal_control.gimbal_yaw_motor);
//		}
    else
    {
        gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_yaw_motor);
    }

    // pitch
    if (gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_control.gimbal_pitch_motor.given_current = 0;
    }
//		else if(gimbal_behaviour == GIMBAL_AUTO)
//		{
//			gimbal_motor_auto_control(&gimbal_control.gimbal_pitch_motor);
//		}
    else
    {
        gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_pitch_motor);
    }


}


/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
static void gimbal_motor_auto_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid_zimiao, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief ��̨PID��ʼ������
 *
 * @param pid PID���ݽṹָ��
 * @param kp
 * @param ki
 * @param kd
 * @param maxout ������
 * @param max_iout ���������
 */
static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float maxout, float max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

/**
 * @brief ��̨PID���㺯��
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta ���΢���ֱ�ӴӴ�������ȡ��������
 * @return fp32
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta)
{
    float err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
 * @brief          ��̨PID��������pid��out,iout
 * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
 * @brief          ����yaw �������ָ��
 * @param[in]      none
 * @retval         yaw���ָ��
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          ����pitch �������ָ��
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
  return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief ��̨��̬���ݷ��͵���λ��
 *
 */
void gimbal_data_send(void)
{
    static uint8_t send_time;
    //ÿ5���뷢��һ��  500Hz
    if (send_time == 5)
    {
			
				user_data_pack_handle();
        send_time = 0;
			
    }
    send_time++;
}
