#include "main.h"
#include "top_ctrl.h"
#include "math.h"
#include "pid.h"
#include "can.h"
#include "can_recv.h"
#include "chassis.h"
#include "usart.h"
#include "bsp_usart.h"
#include "ball_track.h"
#include "struct_typedef.h"
#include "user_lib.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#define DEBUG 0     //调试开关

fp32 filter_coefficient[1];

extern DM4340_motor_data_t DM4340_Date[3];    // 引用DM4340回传数据结构体
extern motor_control_t motor_control;         // 引用底盘电机控制结构体
extern DBUSDecoding_Type DBUS_ReceiveData;    // 底盘发来的dbus的数据
extern ball_track_target_t ball_track_target; // 球追踪目标点
extern angle_t ANGLE;                         // 底盘电机角度

double T[3];     // T是引入万能公式的一个变量。令tan（THETA/2） = T,这样就可以将式子中相关与THETA的复杂运算给简化成用T表示。
double THETA[3]; // THETA变量表示三个电机的角度，可由万能公式的T解出

float angle[3];

Top_Pos delta_position;
Ball_Pos RX_ball_pos;
first_order_filter_type_t filter_angle;

double A[3] = {0};
double B[3] = {0};
double C[3] = {0};

double K[3] = {0};
double U[3] = {0};
double V[3] = {0};

// 球拍控制主循环
void top_contorl_Task(void const *argument)
{
///////////////初始化函数///////////////
    DM_Motor_Init();
    motor_init(&motor_control);
    ball_track_pid_init();
///////////////初始化函数///////////////


    while (1)
    {
        osDelay(1);
///////////////调试函数///////////////
        if(DEBUG)
        {uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n",
                        DM4340_Date[0].real_angle,
                        DM4340_Date[1].real_angle,
                        DM4340_Date[2].real_angle);
        }
///////////////调试函数///////////////

///////////////计算函数///////////////


///////////////计算函数///////////////

///////////////控制函数///////////////
        for (int i = 1; i < 3; i++)
        {
        MD_motor_SendCurrent(&hcan2, i, DM4340_Date[i].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
        osDelay(1);
        }

///////////////控制函数///////////////

    }
}

// 达妙电机初始化
void DM_Motor_Init(void)
{

    for (int i = 0; i < 3; i++)
    {
        start_motor(&hcan2, 0x01);
        osDelay(300);
        start_motor(&hcan2, 0x02);
        osDelay(300);
        start_motor(&hcan2, 0x03);
        osDelay(300);
    }
    DM4340_Date[0].target_angle = (-(float_constrain(96, 86, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(157, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(149, 139, 169)) / 180 * PI);

        MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
        osDelay(2);
        MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
        osDelay(2);
        MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
        osDelay(2);
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

float RUD_DirAngle_c(float Angle)
{
    while (Angle > 18000 || Angle < 0)
    {
        if (Angle < 0)
        {
            Angle += 360;
        }
        if (Angle > 360)
        {
            Angle -= 360;
        }
    }
    return (float)Angle;
}

// 球拍姿态解算
void delta_arm_solution(void)
{

    A[0] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) - 2 * delta_position.x * (R1 - R2)) / (2 * L1);
    B[0] = -(R1 - R2 - delta_position.x);
    C[0] = delta_position.z;

    A[1] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (delta_position.x - sqrt(3) * delta_position.y) * (R1 - R2)) / L1;
    B[1] = -2 * (R1 - R2) - (delta_position.x - sqrt(3) * delta_position.y);
    C[1] = 2 * delta_position.z;

    A[2] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (delta_position.x - sqrt(3) * delta_position.y) * (R1 - R2)) / L1;
    B[2] = -2 * (R1 - R2) - (delta_position.x + sqrt(3) * delta_position.y);
    C[2] = 2 * delta_position.z;

    K[0] = A[0] + B[0];
    U[0] = 2 * C[0];
    V[0] = A[0] - B[0];

    K[1] = A[1] + B[1];
    U[1] = 2 * C[1];
    V[1] = A[1] - B[1];

    K[2] = A[2] + B[2];
    U[2] = 2 * C[2];
    V[2] = A[2] - B[2];

    T[0] = (-U[0] - sqrt(U[0] * U[0] - 4 * K[0] * V[0])) / (2 * K[0]);
    T[1] = (-U[1] - sqrt(U[1] * U[1] - 4 * K[1] * V[1])) / (2 * K[1]);
    T[2] = (-U[2] - sqrt(U[2] * U[2] - 4 * K[2] * V[2])) / (2 * K[2]);

    //	THETA[0] = (180 * (2 * atan(T[0]))) / PI;
    //	THETA[1] = (180 * (2 * atan(T[1]))) / PI;
    //	THETA[2] = (180 * (2 * atan(T[2]))) / PI;

    THETA[0] = (180 * (2 * atan(T[1]))) / PI;
    THETA[1] = (180 * (2 * atan(T[2]))) / PI;
    THETA[2] = (180 * (2 * atan(T[0]))) / PI;

}

//// pos的PID初始化
// static void POS_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
//{
//     if (pid == NULL)
//     {
//         return;
//     }
//     pid->kp = kp;
//     pid->ki = ki;
//     pid->kd = kd;
//
//     pid->err = 0.0f;
//     pid->get = 0.0f;
//
//     pid->max_iout = max_iout;
//     pid->max_out = maxout;
// }
//
//// 6020的PID计算
// static fp32 POS_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
//{
//     fp32 err;
//     if (pid == NULL)
//     {
//         return 0.0f;
//     }
//     pid->get = get;
//     pid->set = set;
//
//     err = set - get;
//     pid->err = rad_format(err);
//     pid->Pout = pid->kp * pid->err;
//     pid->Iout += pid->ki * pid->err;
//     pid->Dout = pid->kd * error_delta;
//     abs_limit(&pid->Iout, pid->max_iout);
//     pid->out = pid->Pout + pid->Iout + pid->Dout;
//     abs_limit(&pid->out, pid->max_out);
//     return pid->out;
// }

// 延时函数
void delay(int count)
{
    int i;
    for (i = 1; i <= count; i++)
        ;
}
