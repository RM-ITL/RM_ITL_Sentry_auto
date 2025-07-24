/**
 * @file chassis_behaviour.h
 * @author ���廪
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

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 0
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 1
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2
//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL LEFT_SWITCH
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f
//ҡ������
#define CHASSIS_RC_DEADLINE 10
#define CHASSIS_OPEN_RC_SCALE 10 //��chassis_open ģ���£�ң�������Ըñ������͵�can��


#define AUTO_CHASSIS_TOP 0
#define AUTO_MOVE 1
#define AUTO_IN_HOME 2

/**
 * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
 * @param[in]      chassis_move_mode: ��������
 * @retval         none
 */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
 * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
 * @param[out]     vx_set, ͨ�����������ƶ�.
 * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
 * @param[out]     wz_set, ͨ��������ת�˶�.
 * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
 * @retval         none
 */
extern void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector);
extern int auto_move_flag;

#endif
