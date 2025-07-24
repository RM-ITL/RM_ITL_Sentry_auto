#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//void can_filter_init(void)
//{
//    CAN_FilterTypeDef can_filter_st;

//    // ���� CAN1 ������
//    can_filter_st.FilterActivation = ENABLE;
//    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;  // ʹ�� IDList ģʽ����ȫƥ�� ID
//    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;  // 32 λ������
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;  // ���� FIFO0

//    // Ϊ CAN1 ���ö����������ƥ������ ID: 0x201, 0x202, 0x203, 0x204, 0x207, 0x209, 0x20A, 0x211
//    uint16_t can1_ids[] = {0x201, 0x202, 0x203, 0x204, 0x207, 0x209, 0x20A, 0x211};
//    for (int i = 0; i < 8; i++) {
//        can_filter_st.FilterIdHigh = can1_ids[i] << 5;   // ���ù������� ID �� 16 λ��ID ���� 5 λ��
//        can_filter_st.FilterIdLow = 0x0000;               // ID �� 16 λΪ 0
//        can_filter_st.FilterMaskIdHigh = 0x0000;          // ��ʹ�����룬��ȫƥ�� ID
//        can_filter_st.FilterMaskIdLow = 0x0000;           // ��ʹ�����룬��ȫƥ�� ID
//        can_filter_st.FilterBank = i;                      // ʹ�ù����� 0-7
//        HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);      // ���ù�����
//    }

//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

//    // ���� CAN2 ������
//    can_filter_st.SlaveStartFilterBank = 14;  // �ӵ� 14 �Ź�������ʼ����
//    can_filter_st.FilterBank = 14;            // ʹ�ù����� 14
//    uint16_t can2_ids[] = {0x201, 0x202};     // ֻ���� 0x201 �� 0x202

//    for (int i = 0; i < 2; i++) {
//        can_filter_st.FilterIdHigh = can2_ids[i] << 5;   // ���ù������� ID �� 16 λ��ID ���� 5 λ��
//        can_filter_st.FilterIdLow = 0x0000;               // ID �� 16 λΪ 0
//        can_filter_st.FilterMaskIdHigh = 0x0000;          // ��ʹ�����룬��ȫƥ�� ID
//        can_filter_st.FilterMaskIdLow = 0x0000;           // ��ʹ�����룬��ȫƥ�� ID
//        HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);      // ���ù�����
//    }

//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//}

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}
