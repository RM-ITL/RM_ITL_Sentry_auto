/**
 * @file USART_receive.c
 * @author ���廪
 * @brief �����жϽ��պ�������������Ƭ���������豸�Ĵ���ͨ������
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "USART_receive.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "referee.h"
#include "chassis_task.h"
#include "usart.h"
#include "chassis_power_control.h"
#include "gimbal_behaviour.h"
#include "chassis_behaviour.h"
#include "string.h"
#include "INS_task.h"
#include "bsp_led.h"

//#include "usart.h"
//#include <stdio.h>

//#ifdef __GNUC__
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif

// PUTCHAR_PROTOTYPE
//{
//   HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//������ʽ��ӡ,����1
//   return ch;
// }
extern chassis_move_t chassis_move;
extern UART_HandleTypeDef huart1;

auto_shoot_t auto_shoot = {0.0f, 0.0f, 0}; //��������
auto_move_t auto_move = {0.0f, 0.0f, 0};//��������
user_send_data_t user_send_data;
int16_t c_switch = 7;
uint8_t target_get=0;

//����ԭʼ���ݣ�Ϊ10���ֽڣ�����18���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];

/**
 * @brief �û����ݽ��
 *
 * @param buf ���ڽ�������ָ��
 * @param auto_shoot �������ݽṹָ��
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot,auto_move_t *auto_move);
void aRGB_led_show(uint32_t aRGB);
void user_usart_init(void)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_NUM); //
 
}

uint16_t this_time_rx_len = 0;

void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) //���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        // static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);
        
        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            //ʧЧDMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //�����趨���ݳ���
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //�趨������1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            //ʹ��DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

            if (this_time_rx_len == USER_FRAME_LENGTH)//
            {
                //��ȡ����
               user_data_solve(usart1_rx_buf[0], &auto_shoot,&auto_move);
            }
            else if(this_time_rx_len !=USER_FRAME_LENGTH)
            {
                
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //ʧЧDMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //�����趨���ݳ���
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //�趨������0
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            //ʹ��DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //��ȡ����
                user_data_solve(usart1_rx_buf[1], &auto_shoot,&auto_move);
            } 
            else if(this_time_rx_len !=USER_FRAME_LENGTH)
            {
               
            }
        }
    }
}


/**
 * @brief �û����ݽ��
 *
 * @param buf ���ڽ�������ָ��
 * @param auto_shoot �������ݽṹָ��
 */
extern uint8_t target_distance;


void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot,auto_move_t *auto_move)
{
    //У��
    if (buf[0] == 0xFF && buf[11]== 0xFE)
    {
				aRGB_led_show(0xFF00FF00);
        auto_shoot->pitch_add =0.001f *((float)(buf[2] << 8 | buf[1]));//(0.0001f * ((float)(buf[2] << 8 | buf[1]))) - USART_PI;
        auto_shoot->yaw_add = 0.001f *((float)(buf[4] << 8 | buf[3]));//(0.0001f * ((float)(buf[4] << 8 | buf[3]))) - USART_PI;
			if((auto_shoot->pitch_add||auto_shoot->yaw_add)!=0)
				{
					 find_target=1;
				}
				else
				{
					find_target=0;
				}
			switch(buf[5])
        {
            case 0:auto_shoot->pitch_add=auto_shoot->pitch_add*-1;auto_shoot->yaw_add=auto_shoot->yaw_add*-1;break;
            case 1:auto_shoot->pitch_add=auto_shoot->pitch_add*-1;break;
            case 2:auto_shoot->yaw_add=auto_shoot->yaw_add*-1;break;
            default:break;
        }
        target_distance=buf[6];
				auto_move->auto_vx=buf[7];//((float)(buf[8] << 8 | buf[7]));
				auto_move->auto_vy=buf[8];//((float)(buf[10] << 8 | buf[9]));
				auto_move->auto_wz=buf[9];//((float)(buf[12] << 8 | buf[11]));
				switch(buf[10])
        {
            case 0:auto_move->auto_vx=auto_move->auto_vx*-1;auto_move->auto_vy=auto_move->auto_vy*-1;auto_move->auto_wz=auto_move->auto_wz*-1;break;//xyzΪ��
            case 1:auto_move->auto_vx=auto_move->auto_vx*-1;auto_move->auto_vy=auto_move->auto_vy*-1;break;//xyΪ��
            case 2:auto_move->auto_vx=auto_move->auto_vx*-1;auto_move->auto_wz=auto_move->auto_wz*-1;break;//xz
						case 3:auto_move->auto_vy=auto_move->auto_vy*-1;auto_move->auto_wz=auto_move->auto_wz*-1;break;//yz
						case 4:auto_move->auto_vx=auto_move->auto_vx*-1;break;//x
						case 5:auto_move->auto_vy=auto_move->auto_vy*-1;break;//y
						case 6:auto_move->auto_wz=auto_move->auto_wz*-1;break;//z
//						case 7:
//						case 8:auto_move_flag=AUTO_CHASSIS_TOP;
            default:break;
        }
				if((auto_move->auto_vx||auto_move->auto_vy||auto_move->auto_wz)!=0)//���ƶ�����
				{
					auto_move_flag=AUTO_MOVE;//����
					//find_target=0;//��·������,ͷ����
				}
//				else if((auto_move->auto_vx&&auto_move->auto_vy&&auto_move->auto_wz&&auto_shoot->pitch_add&&auto_shoot->yaw_add==0)&&buf[10]==8)//&&buf[10]==8
//				{
//					auto_move_flag=AUTO_CHASSIS_TOP;
//				}
				else
				{
					auto_move_flag=AUTO_CHASSIS_TOP;
					//auto_move_flag=AUTO_IN_HOME;
				}
        detect_hook(USER_USART_DATA_TOE);
    }
}



int m = 1;//ģʽ�л���־λ��Ϊ0��ʾ������������Ϊ1��ʾ������̨����,2�������
//bool_t reset_tracker = 0;//���ü���־λ��Ĭ��0
float chassis_power1 = 0.0f;        //���̹���
float chassis_power_buffer1 = 0.0f;   //���̻�������
uint8_t back_home=0;
extern ext_power_heat_data_t power_heat_data_t;
extern void get_chassis_power_and_buffer(float *current,float *volt,float *power, float *buffer);
extern INS_data_t INS_data;
extern ext_game_robot_state_t robot_state;


void float_to_be_bytes(float value, uint8_t* bytes) {
    uint32_t int_val;
    memcpy(&int_val, &value, sizeof(float));
    
    bytes[0] = (int_val >> 24) & 0xFF;
    bytes[1] = (int_val >> 16) & 0xFF;
    bytes[2] = (int_val >> 8) & 0xFF;
    bytes[3] = int_val & 0xFF;
}

void user_data_pack_handle()
{
    if(m==0)
    {
    //���͵�ǰ�����ٶ� ��λm/s
        ANO_DT_send_int16((int16_t)(chassis_move.motor_chassis[0].speed*100), (int16_t)(chassis_move.motor_chassis[1].speed*100),
        (int16_t)(chassis_move.motor_chassis[2].speed*100), (int16_t)(chassis_move.motor_chassis[3].speed*100), 
        power_heat_data_t.chassis_current, power_heat_data_t.chassis_power,power_heat_data_t.chassis_power_buffer, 0 );
//      get_chassis_power_and_buffer(&chassis_power1, &chassis_power_buffer1);
//      ANO_DT_send_int16((int16_t)chassis_power1, (int16_t)chassis_power_buffer1	,0, 0, 0, 0, 0, 0 );
    }
    else
    {
				static uint8_t tx_buf[4];
        if (get_robot_id() <= 7)
        {
            user_send_data.enemy_color = 0;//��ɫ
        }
        else
        {
            user_send_data.enemy_color = 1;//��ɫ
        }
				if(robot_state.remain_HP>=robot_state.max_HP*50/100)
				{
					back_home=0;
				}
				else
				{
					back_home=1;
				}
        tx_buf[0] = 0xFF;
				tx_buf[1] = user_send_data.enemy_color;
				tx_buf[2] = back_home;
				tx_buf[3] = 0xFE;
       usart1_tx_dma_enable(tx_buf, 4);
//  if(a==2)
//  {
//      //get_chassis_power_and_buffer(&chassis_power1, &chassis_power_buffer1);
//      ANO_DT_send_int16((int16_t)chassis_power1, (int16_t)chassis_power_buffer1,0, 0, 0, 0, 0, 0 );
//      }
    }
}
