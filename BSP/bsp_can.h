#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"
#include "struct_typedef.h"
#include "pid.h"

//extern float idangle[4];
extern int16_t bbb;
extern int16_t bbbg;
extern uint16_t rc_channels[3];
extern uint16_t zimiao[2];
extern float idspeed[4];
extern volatile float can_a1_position;
extern volatile float can_a1_speed;
//extern int16_t ecd[4];
//extern int16_t bbb[4];
extern void CAN_cmd_chassis(int16_t motor1, 
	                          int16_t motor2, 
                            int16_t motor3, 
                            int16_t motor4);

extern void CAN_cmd_chassis1(int16_t motor1, 
	                          int16_t motor2, 
                            int16_t motor3, 
                            int16_t motor4);
extern void CAN_cmd_chassis_can2(CAN_HandleTypeDef *hcan,
                            int16_t motor1, 
	                          int16_t motor2, 
                            int16_t motor3, 
                            int16_t motor4);

extern void CAN_cmd_chassis1_can2(CAN_HandleTypeDef *hcan,
                            int16_t motor1, 
	                          int16_t motor2, 
                            int16_t motor3, 
                            int16_t motor4);
extern void CAN_cmd_chassis1_xx(int16_t motor1, 
	                          int16_t motor2, 
                            int16_t motor3, 
                            int16_t motor4);







void can_filter_init(void);

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

typedef struct
{
    int16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

float firstOrderFilter(float in_data,float ab);
float firstOrderFilter_1(float in_data,float ab);
float firstOrderFilter_2(float in_data,float ab);
float firstOrderFilter_3(float in_data,float ab);
float firstOrderFilter_4(float in_data,float ab);
float firstOrderFilter_5(float in_data,float ab);
//typedef struct
//{
////  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
////  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
////  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
//  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
////  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
////  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
//  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
//   pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
////  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid
////	pid_type_def revolve_pid;                    //С������תpid

//  //first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
//  //first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

//  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
//  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
//  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
//  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
//  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
//  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
//  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
//  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�
//  fp32 chassis_yaw_set;             

//  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
//  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
//  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
//  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
//  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
//  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
//  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�

//		fp32 chassis_angle_set;//С���ݵڶ��ַ���ʱ������ĽǶ�
//		fp32 revolve_angle;//���ڼ���С������תʱǰ��ʱ��ά�������Ǻ���

//} chassis_move_t;


extern motor_measure_t motor_chassis[9];
typedef struct {
	uint8_t MST_ID;
	uint8_t ID_ERR;
	float POS;
	float VEL;
	float T;
	uint8_t T_MOS;
	uint8_t T_Rotor;
} FeedbackFrame;

extern	FeedbackFrame feedbackFrame[2];
extern void DM_MIT_send(CAN_HandleTypeDef *hcan, uint16_t id,float p_des,float v_des,float Kp,float Kd,float t_ff);
extern void DM_MIT_send_qidon(CAN_HandleTypeDef *hcan, uint16_t id);
extern void ProcessFeedbackFrame(uint8_t *data, FeedbackFrame *frame);








#endif

