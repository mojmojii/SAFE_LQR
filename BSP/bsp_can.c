#include "can.h"
#include "bsp_can.h"
#include <string.h>

uint8_t chassis_can_send_data[8]={0};
CAN_TxHeaderTypeDef  chassis_tx_message;
motor_measure_t motor_chassis[9];
volatile float can_a1_position = 0.0f;
volatile float can_a1_speed = 0.0f;


extern CAN_HandleTypeDef hcan1;

//int16_t ecd[4];

int16_t bbb=0;
 int test=0;
int16_t bbbg=0;
//uint8_t ppd=0;
//int16_t cff=0;

//float idangle[4]={0};

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//0x200
{
	
	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data,&send_mail_box);
}
void CAN_cmd_chassis1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//0x1FF
{

	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x1FF;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data,&send_mail_box);
}
void CAN_cmd_chassis_can2(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//0x200
{
	
	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data,&send_mail_box);
}
void CAN_cmd_chassis1_can2(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//0x1FF
{

	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x1FF;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data,&send_mail_box);
}
void CAN_cmd_chassis1_xx(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//0x1FF
{

	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x2FF;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data,&send_mail_box);
}
void can_filter_init(void)//can12�Ĺ�������ʼ������
{

    CAN_FilterTypeDef can_filter_st = {0};
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*
*ecd-ת�ӻ�е�Ƕ�
*speed_rpm-ת��ת��
*given_current-ʵ��ת�ص���
*temperate-�¶�
*/
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
//		void jiaoduzl(uint8_t i)
//{
// switch(i)
// {
//	 case 4:
//	 {ecd[0]=(int16_t)(motor_chassis[4].ecd)-5479;
//      idangle[0]=ecd[0]*360/(8191);
//	    break;
//	 } 
//	 case 5:
//	 {
//		  ecd[1]=653-motor_chassis[5].ecd+cff;
//     idangle[1]=ecd[1]*360/(8191);
//		 if(ppd==0)
//		 {
//			 if(ecd[1]<-1000)
//		   {
//			  cff=8191;
//			 }
//		 ppd=1;
//		 }
//	    break;		
//	 }		 
//   case 6:
//	 {ecd[2]=3721-(int16_t)(motor_chassis[6].ecd);
//	   idangle[2]=ecd[2]*360/(8191);
//	   break;
//	 }
//	 case 7:
//	 { ecd[3]=(int16_t)(motor_chassis[7].ecd)-6158;
//	   idangle[3]=ecd[3]*360/(8191);
//	   break;
//	 }
//   default:
//        {
//            break;
//        }
//	 
// }  
//					   
//					   


//}
//				
//	void quanshu(motor_measure_t *t,uint8_t p)
//{
//	if((p==1)||(p==2))
//	{
//  if(t->last_ecd-t->ecd>5000)
//	{
//	   bbb[p]=bbb[p]-1;
//	
//	}
//  if(t->ecd - t->last_ecd>5000)
//	{
//	  bbb[p]=bbb[p]+1;
//	}
//}
//		else
//	{
//  if(t->last_ecd-t->ecd>5000)
//	{
//	   bbb[p]=bbb[p]+1;
//	
//	}
//  if(t->ecd - t->last_ecd>5000)
//	{
//	  bbb[p]=bbb[p]-1;
//	}
//}
//	
//}	
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	  test=test+1;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
 
	      switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case 0x205:
        case 0x206:
        case 0x207:
		case 0x208:
	    case 0x209: {
            static uint8_t i = 0;

            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);

            break;
        }
        case 0x01: {
            ProcessFeedbackFrame(rx_data, &feedbackFrame[rx_header.StdId - 1U]);
            break;
        }
	    case 0x02: {
            float position;
            float speed;

            if (rx_header.DLC == 8U)
            {
                memcpy(&position, &rx_data[0], sizeof(position));
                memcpy(&speed, &rx_data[4], sizeof(speed));
                can_a1_position = position;
                can_a1_speed = speed;
            }

             break;
        }
        default:
        {
            break;
        }
   
	 }
    
 

 }





		FeedbackFrame feedbackFrame[2];
void DM_MIT_send(CAN_HandleTypeDef *hcan, uint16_t id,float p_des,float v_des,float Kp,float Kd,float t_ff)
{
        const CAN_TxHeaderTypeDef tx_header = {
            .StdId = id,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE,
        };
uint32_t tx_mailbox;


 uint8_t tx_message[8] = {0};
int16_t p_int = (int16_t)((p_des+12.5f)* 32767.0f / 12.5f);
tx_message[0] = (uint8_t)((p_int >> 8) & 0xFF);
tx_message[1] = (uint8_t)(p_int & 0xFF);
int16_t v_raw = (int16_t)((v_des+50.0f) * (2047.0f / 50.0f));
tx_message[2] = (uint8_t)((v_raw >> 4) & 0xFF);
 int16_t kp_raw=(Kp)*4095/500.0f;
tx_message[3] = ((v_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F);
tx_message[4] = (uint8_t)(kp_raw & 0xFF);
int16_t t_raw = (int16_t)((t_ff+5.0f) * (2047.0f / 5.0f));

int16_t kd_raw=(Kd)*4095/5.0f;
tx_message[5] = (uint8_t)((kd_raw >> 4) & 0xFF);
tx_message[6] = ((kd_raw & 0x0F) << 4) | ((t_raw >> 8) & 0x0F);
tx_message[7] = (uint8_t)(t_raw & 0xFF);
// memcpy((void *)p,(const void *)tx_message,8);
HAL_CAN_AddTxMessage(hcan, &tx_header, tx_message, &tx_mailbox);
}
void DM_MIT_send_qidon(CAN_HandleTypeDef *hcan, uint16_t id)
{
        const CAN_TxHeaderTypeDef tx_header = {
            .StdId = id,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE,
        };
uint32_t tx_mailbox;


 uint8_t tx_message[8] = {0};
 tx_message[0]=0xFF;
 tx_message[1]=0xFF;
 tx_message[2]=0xFF;
 tx_message[3]=0xFF;
  tx_message[4]=0xFF;
  tx_message[5]=0xFF;
  tx_message[6]=0xFF;
  tx_message[7]=0xFC;


HAL_CAN_AddTxMessage(hcan, &tx_header, tx_message, &tx_mailbox);
}



void ProcessFeedbackFrame(uint8_t *data, FeedbackFrame *frame) {

	uint16_t p;
	int16_t v;
	int16_t t;
    frame->MST_ID = data[0]>> 4;
    frame->ID_ERR = data[0];
     p = (data[1] << 8) | data[2];
     v= ((data[3] & 0xFF) << 4) | (data[4]>>4) ;
     t = ((data[4] & 0x0F) << 8) | data[5];
    frame->T_MOS = data[6];
    frame->T_Rotor= data[7];
	   frame->POS= ((p-32767)/32767.0f)*12.5f;
	   frame->VEL= ((v-2047)/2047.0f)*50.0f;
	   frame->T= ((t-2047)/2047.0f)*5.0f;

}




