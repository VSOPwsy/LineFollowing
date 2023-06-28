#include "line_following.h"
#include "tim.h"

struct PID pid_control;

void Line_Following_Init()
{
    pid_control.Kp_angle = 3;
    pid_control.Ki_angle = 1;
    pid_control.Kd_angle = 1;
    pid_control.Kp_distance = 3;
    pid_control.Ki_distance = 1;
    pid_control.Kd_distance = 1;

    pid_control.desired_angle = 0;
    pid_control.desired_distance = 0;
    pid_control.previous_angle_error = 0;
    pid_control.previous_distance_error = 0;
    pid_control.integral_angle = 0;
    pid_control.integral_distance = 0;
}

void update_pid(struct PID *pid, float current_angle, float current_distance, void (*motor_function)(int, int))
{
    float angle_error = pid->desired_angle - current_angle;
    pid->integral_angle += angle_error;
    float angle_derivative = angle_error - pid->previous_angle_error;

    float distance_error = pid->desired_distance - current_distance;
    pid->integral_distance += distance_error;
    float distance_derivative = distance_error - pid->previous_distance_error;

    int output_angle = pid->Kp_angle*angle_error + pid->Ki_angle*pid->integral_angle + pid->Kd_angle*angle_derivative;
    int output_distance = pid->Kp_distance*distance_error + pid->Ki_distance*pid->integral_distance + pid->Kd_distance*distance_derivative;
    motor_function(output_angle, output_distance);

    pid->previous_angle_error = angle_error;
    pid->previous_distance_error = distance_error;
}

void Motor_Rotation(int angle, int distance)
{
  Motor_Left_Front(40000 + angle + distance);
  Motor_Left_Rear(40000 + angle - distance);
  Motor_Right_Front(40000 - angle - distance);
  Motor_Right_Rear(40000 - angle + distance);
}

void Motor_Left_Front(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -0.5 * speed + 32768);
}
void Motor_Right_Front(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0.5 * speed + 32768);
}
void Motor_Left_Rear(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -0.5 * speed + 32768);
}
void Motor_Right_Rear(int speed)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -0.5 * speed + 32768);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0.5 * speed + 32768);
}
