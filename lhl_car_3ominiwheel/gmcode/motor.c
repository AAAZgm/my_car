#include "motor.h"

//m1 pwmÓÃµÄtimer1 ch3ch4(pe14,13)£¬¶ÁÈ¡ÓÃtim5¶¨(pa0,pa1)£¬Ç°£¬±àºÅ1
//m2 pwmÓÃµÄtim1 ch1ch2(pe)£¬¶ÁÈ¡ÓÃtim2(pb3 pa15)£¬×ó  ±àºÅ4
//m3 pwmÓÃµÄtimer9 ch1ch2£¨pe5,6)£¬¶ÁÈ¡ÓÃtim4¶¨(pb6,pb7)£¬ÓÒ£¬±àºÅ2

PID F_AddPID;  // ±àºÅ1 = Ç° (wheel1/M1)
PID R_AddPID;  // ±àºÅ2 = ÓÒ (wheel3/M3)
PID L_AddPID;  // ±àºÅ4 = ×ó (wheel2/M2)

void F_PID_Init(void)
{
    F_AddPID.PrevError=0;
    F_AddPID.target_val = 0;
    F_AddPID.output_val = 0.0;
    F_AddPID.Error = 0.0;
    F_AddPID.LastError = 0.0;
    F_AddPID.integral = 0.0;
    F_AddPID.Kp = 0.5;
    F_AddPID.Ki = 0.1;
    F_AddPID.Kd = 0; 
}

void R_PID_Init(void)
{
    R_AddPID.PrevError=0;
    R_AddPID.target_val = 0;
    R_AddPID.output_val = 0.0;
    R_AddPID.Error = 0.0;
    R_AddPID.LastError = 0.0;
    R_AddPID.integral = 0.0;
    R_AddPID.Kp = 0.5;
    R_AddPID.Ki = 0.1;
    R_AddPID.Kd = 0; 
}

void L_PID_Init(void)
{
    L_AddPID.PrevError=0;
    L_AddPID.target_val = 0;
    L_AddPID.output_val = 0.0;
    L_AddPID.Error = 0.0;
    L_AddPID.LastError = 0.0;
    L_AddPID.integral = 0.0;
    L_AddPID.Kp = 0.5;
    L_AddPID.Ki = 0.1;
    L_AddPID.Kd = 0; 
}

void motor_init(void)
{
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Init(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  

    HAL_TIM_Base_Start(&htim9);
    HAL_TIM_PWM_Init(&htim9);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);  

    __HAL_TIM_SET_COUNTER(&htim2,0);
    __HAL_TIM_SET_COUNTER(&htim4,0);
    __HAL_TIM_SET_COUNTER(&htim5,0);

    F_PID_Init();  // Ç°
    R_PID_Init();  // ÓÒ
    L_PID_Init();  // ×ó
}

void Set_PID_TargetSpeed(float target, uint8_t which)
{
    if (target < Min_Pid_Value) target = Min_Pid_Value;
    if (target > Max_Pid_Value) target = Max_Pid_Value;

    if (which == 1) {
        F_AddPID.target_val = target;  // ±àºÅ1 = Ç°
    } else if (which == 2) {
        R_AddPID.target_val = target;  // ±àºÅ2 = ÓÒ
    } else if (which == 4) {
        L_AddPID.target_val = target;  // ±àºÅ4 = ×ó
    }
}

void set_v(int16_t speed, uint8_t which)
{
    speed = (speed > Max_PWM_Value) ? Max_PWM_Value : (speed < Min_PWM_Value) ? Min_PWM_Value : speed;
    if (speed > 0) {
        switch (which) {
        case 1:  // Ç°
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
            break;
        case 2:  // ÓÒ
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);
            break;
        case 4:  // ×ó
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
            break;
        }
    } else if (speed < 0) {
        switch (which) {
        case 1:  // Ç°
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -speed);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            break;
        case 2:  // ÓÒ
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -speed);
            break;
        case 4:  // ×ó
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
            break;
        }
    } else {
        switch (which) {
        case 1:  // Ç°
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            break;
        case 2:  // ÓÒ
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            break;
        case 4:  // ×ó
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            break;
        }
    }
}

float addPID_realize(PID *pid, float actual_val)
{
    pid->Error = pid->target_val - actual_val;
    pid->output_val += pid->Kp * (pid->Error - pid->LastError) +
                       pid->Ki * pid->Error +
                       pid->Kd * (pid->Error - 2 * pid->LastError + pid->PrevError);
    pid->PrevError = pid->LastError;
    pid->LastError = pid->Error;
    if (pid->output_val > Max_PWM_Value) {
        pid->output_val = Max_PWM_Value;
    } else if (pid->output_val < Min_PWM_Value) {
        pid->output_val = Min_PWM_Value;
    }
    return pid->output_val;
}

static int16_t F_Encoder_Speed = 0, R_Encoder_Speed = 0, L_Encoder_Speed = 0;
static int16_t F_Output_Val = 0, R_Output_Val = 0, L_Output_Val = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim8)) {
			  update_odom_3wheel(50);
        send_msg(unhandle_msg);
    }
		else if (htim == (&htim6)) { 
			  R_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);   // ±àºÅ2 = ÓÒ
        F_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);   // ±àºÅ1 = Ç°
        L_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);   // ±àºÅ4 = ×ó
				__HAL_TIM_SET_COUNTER(&htim4, 0);
        __HAL_TIM_SET_COUNTER(&htim5, 0);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
			  R_Output_Val = (int16_t)addPID_realize(&R_AddPID, R_Encoder_Speed);
        F_Output_Val = (int16_t)addPID_realize(&F_AddPID, F_Encoder_Speed);
        L_Output_Val = (int16_t)addPID_realize(&L_AddPID, L_Encoder_Speed);

        set_v(F_Output_Val, 1);  // ±àºÅ1 = Ç°
        set_v(R_Output_Val, 2);  // ±àºÅ2 = ÓÒ
        set_v(L_Output_Val, 4);  // ±àºÅ4 = ×ó
}}

float get_angular(int16_t cycle)
{
    return (float)((cycle * 360.0f) / PULSE_A_circle);
}

float get_persentspeed(uint8_t which)
{
    if (which == 1)        // Ç°
        return (float)F_Encoder_Speed * (PI * RADIUS / PULSE_A_circle / 1000.0f) / (1.0f / RATE);
    else if (which == 2)   // ÓÒ
        return (float)R_Encoder_Speed * (PI * RADIUS / PULSE_A_circle / 1000.0f) / (1.0f / RATE);
    else if (which == 4)   // ×ó
        return (float)L_Encoder_Speed * (PI * RADIUS / PULSE_A_circle / 1000.0f) / (1.0f / RATE);
    return 0;
}

float get_targetpulse(float targetv)
{
    return (float)targetv * (1.0f / RATE) / (PI * RADIUS / PULSE_A_circle / 1000.0f);
}
