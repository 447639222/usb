/******************************************************************************
 /// @brief
 /// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
 /// @license MIT License
 /// Permission is hereby granted, free of charge, to any person obtaining a copy
 /// of this software and associated documentation files (the "Software"), to deal
 /// in the Software without restriction,including without limitation the rights
 /// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
 /// copies of the Software, and to permit persons to whom the Software is furnished
 /// to do so,subject to the following conditions:
 ///
 /// The above copyright notice and this permission notice shall be included in
 /// all copies or substantial portions of the Software.
 ///
 /// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 /// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 /// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 /// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 /// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 /// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 /// THE SOFTWARE.
 *******************************************************************************/

#include "can.h"
#include "bsp_can.h"
#include "oled.h"
#include "stdio.h"

char message[50];
moto_measure_t moto_chassis[4] =
{ 0 }; //4 chassis moto

void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, uint8_t aData[]);

/*******************************************************************************************
 * @Func	my_can_filter_init
 * @Brief   CAN滤波器配置
 * @Param	CAN_HandleTypeDef* hcan
 * @Retval	None
 * @Date    2019/11/27
 *******************************************************************************************/
void BSP_CAN_Filter_Config(CAN_HandleTypeDef *_hcan)
{
    CAN_FilterTypeDef sCanFilterConf;

    sCanFilterConf.FilterBank = 0;
    sCanFilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
    sCanFilterConf.FilterScale = CAN_FILTERSCALE_32BIT;
    sCanFilterConf.FilterIdHigh = 0x0000;
    sCanFilterConf.FilterIdLow = 0x0000;
    sCanFilterConf.FilterMaskIdHigh = 0x0000;
    sCanFilterConf.FilterMaskIdLow = 0x0000;
    sCanFilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
    sCanFilterConf.SlaveStartFilterBank = 14; //can1(0-13)和can2(14-27)分别得到一半的filter
    sCanFilterConf.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(_hcan, &sCanFilterConf) != HAL_OK)
    {
        //err_deadloop();
        //show error!
        Error_Handler();
    }
}

void BSP_CAN1_Init(void)
{
    BSP_CAN_Filter_Config(&hcan1);

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
            != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}

uint32_t FlashTimer;
/*******************************************************************************************
 * @Func	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief   HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
 * @Param
 * @Retval	None
 * @Date    2019/11/27
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    if (HAL_GetTick() - FlashTimer > 500)
    {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        FlashTimer = HAL_GetTick();
    }

    if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        Error_Handler();
    }

    switch (RxHeader.StdId)
    {
    case CAN_2006Moto1_ID:
    case CAN_2006Moto2_ID:
    case CAN_2006Moto3_ID:
    case CAN_2006Moto4_ID:
    {
        static u8 i;
        i = RxHeader.StdId - CAN_2006Moto1_ID;

        get_moto_measure(&moto_chassis[i], RxData);

//        if (i == 1)
//        {
//            oled_refresh_gram();
//            oled_showstring1(0,  2, "Parameters:");
//            sprintf(message, "%d", moto_chassis[i].last_angle);
//            oled_showstring1(1,  2, message);
//            sprintf(message, "%d", moto_chassis[i].angle);
//            oled_showstring1(2,  2, message);
//            sprintf(message, "%d", moto_chassis[i].speed_rpm);
//            oled_showstring1(3,  2, message);
//            sprintf(message, "%.3f", moto_chassis[i].real_current);
//            oled_showstring1(4,  2, message);
//            oled_refresh_gram();
//        }
    }
        break;
    }
}

/*******************************************************************************************
 * @Func	 void get_moto_measure(moto_measure_t *ptr, uint8_t aData[])
 * @Brief    接收P2006电机通过CAN发过来的信息
 * @Param
 * @Retval	 None
 * @Date     2019/11/27
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, uint8_t aData[])
{

    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t) (aData[0] << 8 | aData[1]);
    ptr->speed_rpm = (int16_t) (aData[2] << 8 | aData[3]);
    ptr->real_current = (aData[4] << 8 | aData[5]) * 5.f / 16384.f;

    ptr->hall = aData[6];

    if (ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, uint8_t aData[])
{
    ptr->angle = (uint16_t) (aData[0] << 8 | aData[1]);
    ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
 *@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
 */
void get_total_angle(moto_measure_t *p)
{

    int res1, res2, delta;
    if (p->angle < p->last_angle)
    {			//可能的情况
        res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    }
    else
    {	//angle > last
        res1 = p->angle - 8192 - p->last_angle;	//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if (ABS(res1) < ABS(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
}

void set_moto_current(CAN_HandleTypeDef *_hcan, s16 iq1, s16 iq2, s16 iq3,
        s16 iq4)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;

    txHeader.StdId = 0x200;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 0x08;

    txData[0] = (iq1 >> 8);
    txData[1] = iq1;
    txData[2] = (iq2 >> 8);
    txData[3] = iq2;
    txData[4] = (iq3 >> 8);
    txData[5] = iq3;
    txData[6] = (iq4 >> 8);
    txData[7] = iq4;

    /* Start the Transmission process */
    if (HAL_CAN_AddTxMessage(_hcan, &txHeader, txData, &txMailbox) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
}
