#include "driver.h"
#include "ANO_TC.h"

struct PID{
    float balance_velocity_Kp;
    float balance_velocity_Ki;
    float balance_velocity_Kd;

    float balance_angle_Kp;
    float balance_angle_Ki;
    float balance_angle_Kd;

    float balance_palstance_Kp;
    float balance_palstance_Ki;
    float balance_palstance_Kd;

    float motor_Kp;
    float motor_Ki;
    float motor_Kd;
};

struct PID pid_data ={0.73,1.1,0.25,
            13.5,2.5,0.78, 
            2.0,0.16,0.57, 
            15,0.09,50};
short Motor_Bias, Motor_Last_Bias=0, Motor_Integrator;


void motor_control(int16_t pwm)
{
    int16_t _temp = 0;
    if(pwm > 0){
        _temp = pwm;
        motor_forward;
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,_temp);
    }else {
        _temp = -pwm;
        motor_reverse;
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,_temp);
    }
}

int16_t get_encoder(void)
{
    int16_t _cnt = 0;
    _cnt = __HAL_TIM_GetCounter(&htim1);
    if(encoder_dir  == 0){
        _cnt = _cnt;
    }else {
        _cnt = -_cnt;
    }
    __HAL_TIM_SetCounter(&htim1,0);
    return _cnt;
}


int16_t motor_pi(int16_t encoder, int16_t target)
{
    int16_t pwm;
    Motor_Bias = target - encoder;
    Motor_Integrator += Motor_Bias;
    if(Motor_Integrator > 5000){
        Motor_Integrator = 5000;
    }else if(Motor_Integrator < -5000){
        Motor_Integrator = -5000;
    }
    pwm = pid_data.motor_Kp*(Motor_Bias) + pid_data.motor_Ki*Motor_Integrator + pid_data.motor_Kd*(Motor_Bias - Motor_Last_Bias);
    if(pwm > 800){
        pwm = 800;
    }else if(pwm <-800){
        pwm = -800;
    }
    Motor_Last_Bias = Motor_Bias ;
    return pwm;
}

float velocity_control(int16_t encoder, int16_t target)
{
    static int16_t bias,integrator,last_bias;
    float out = 0;
    bias = target - encoder ;
    integrator += bias ;
    if(integrator > 1000){
        integrator = 1000;
    }else if(integrator < -1000){
        integrator = -1000;
    }
    out = pid_data.balance_velocity_Kp*bias + pid_data.balance_velocity_Ki*integrator + pid_data.balance_velocity_Kd*(bias - last_bias);
    last_bias = bias;
    return out;
}

int16_t angle_control(float angle, float target_angle)
{
    static float bias,integrator,last_bias;
    int16_t out = 0;
    bias = target_angle -angle;
    integrator += bias;
    if(integrator > 500.0){
        integrator = 500.0;
    }else if(integrator < -500.0){
        integrator = -500.0;
    }
    out = pid_data.balance_angle_Kp*bias + pid_data.balance_angle_Ki*integrator + pid_data.balance_angle_Kd*(bias - last_bias);
    last_bias = bias;
    return out;
}

int16_t palstance_control(int16_t palstance, int16_t target_palstance)
{
    static int16_t bias,integrator,last_bias;
    int16_t out = 0;
    bias = target_palstance - palstance;
    integrator += bias;
    if(integrator > 1000){
        integrator = 1000;
    }else if(integrator < -1000){
        integrator = -1000;
    }
    out = pid_data.balance_palstance_Kp*bias + pid_data.balance_palstance_Ki*integrator + pid_data.balance_palstance_Kd*(bias - last_bias);
    last_bias = bias;
    if(out > 500){
        out = 500;
    }else if(out <-500){
        out = -500;
    }
    return out;
}
