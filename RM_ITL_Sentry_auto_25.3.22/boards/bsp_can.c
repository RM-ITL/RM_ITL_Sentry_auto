#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//void can_filter_init(void)
//{
//    CAN_FilterTypeDef can_filter_st;

//    // 配置 CAN1 过滤器
//    can_filter_st.FilterActivation = ENABLE;
//    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;  // 使用 IDList 模式，完全匹配 ID
//    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;  // 32 位过滤器
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;  // 配置 FIFO0

//    // 为 CAN1 配置多个过滤器，匹配以下 ID: 0x201, 0x202, 0x203, 0x204, 0x207, 0x209, 0x20A, 0x211
//    uint16_t can1_ids[] = {0x201, 0x202, 0x203, 0x204, 0x207, 0x209, 0x20A, 0x211};
//    for (int i = 0; i < 8; i++) {
//        can_filter_st.FilterIdHigh = can1_ids[i] << 5;   // 设置过滤器的 ID 高 16 位（ID 左移 5 位）
//        can_filter_st.FilterIdLow = 0x0000;               // ID 低 16 位为 0
//        can_filter_st.FilterMaskIdHigh = 0x0000;          // 不使用掩码，完全匹配 ID
//        can_filter_st.FilterMaskIdLow = 0x0000;           // 不使用掩码，完全匹配 ID
//        can_filter_st.FilterBank = i;                      // 使用过滤器 0-7
//        HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);      // 配置过滤器
//    }

//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

//    // 配置 CAN2 过滤器
//    can_filter_st.SlaveStartFilterBank = 14;  // 从第 14 号过滤器开始配置
//    can_filter_st.FilterBank = 14;            // 使用过滤器 14
//    uint16_t can2_ids[] = {0x201, 0x202};     // 只接收 0x201 和 0x202

//    for (int i = 0; i < 2; i++) {
//        can_filter_st.FilterIdHigh = can2_ids[i] << 5;   // 设置过滤器的 ID 高 16 位（ID 左移 5 位）
//        can_filter_st.FilterIdLow = 0x0000;               // ID 低 16 位为 0
//        can_filter_st.FilterMaskIdHigh = 0x0000;          // 不使用掩码，完全匹配 ID
//        can_filter_st.FilterMaskIdLow = 0x0000;           // 不使用掩码，完全匹配 ID
//        HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);      // 配置过滤器
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
