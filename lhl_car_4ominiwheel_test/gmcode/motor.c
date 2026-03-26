#include "motor.h"
 
//m1(LF) pwm用的TIM1 CH3CH4(PE14,13)，读取用TIM5(PA0,PA1)，左前，which=1
//m2(RF) pwm用的TIM9 CH1CH2(PE5,6)，读取用TIM4(PB6,PB7)，右前，which=2
//m3(LR) pwm用的TIM10 CH1(PB8)+TIM11 CH1(PB9)，读取用TIM3(PB4,PB5)，左后，which=3
//m4(RR) pwm用的TIM1 CH1CH2，读取用TIM2(PB3,PA15)，右后，which=4
//要对m1(LF)的编码器改极性，驱动和m2(RF)反
//不懂就看原理图
PID LF_AddPID;  // which=1, 左前(LF)
PID RF_AddPID;  // which=2, 右前(RF)
PID LR_AddPID;  // which=3, 左后(LR)
PID RR_AddPID;  // which=4, 右后(RR)

void LF_PID_Init(void)
{		LF_AddPID.PrevError=0;
    LF_AddPID.target_val = 0;
    LF_AddPID.output_val = 0.0;
    LF_AddPID.Error = 0.0;
    LF_AddPID.LastError = 0.0;
    LF_AddPID.integral = 0.0;
    LF_AddPID.Kp = 0.5;
    LF_AddPID.Ki = 0.15;
    LF_AddPID.Kd = 0; 

}
void RF_PID_Init(void)
{		RF_AddPID.PrevError=0;
    RF_AddPID.target_val = 0;
    RF_AddPID.output_val = 0.0;
    RF_AddPID.Error = 0.0;
    RF_AddPID.LastError = 0.0;
    RF_AddPID.integral = 0.0;//增量式不用
    RF_AddPID.Kp = 0.5;
    RF_AddPID.Ki = 0.15;
    RF_AddPID.Kd = 0; 

}

void LR_PID_Init(void)
{		LR_AddPID.PrevError=0;
    LR_AddPID.target_val = 0;
    LR_AddPID.output_val = 0.0;
    LR_AddPID.Error = 0.0;
    LR_AddPID.LastError = 0.0;
    LR_AddPID.integral = 0.0;//增量式不用
    LR_AddPID.Kp = 0.5;
    LR_AddPID.Ki = 0.15;
    LR_AddPID.Kd = 0; 

}

void RR_PID_Init(void)
{		RR_AddPID.PrevError=0;
    RR_AddPID.target_val = 0;
    RR_AddPID.output_val = 0.0;
    RR_AddPID.Error = 0.0;
    RR_AddPID.LastError = 0.0;
    RR_AddPID.integral = 0.0;//增量式不用
    RR_AddPID.Kp = 0.5;
    RR_AddPID.Ki = 0.15;
    RR_AddPID.Kd = 0; 

}
 
 void motor_init(void){
	 
HAL_TIM_Base_Start(&htim4);
HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

HAL_TIM_Base_Start(&htim5);
HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);

HAL_TIM_Base_Start(&htim3);
HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

HAL_TIM_Base_Start(&htim2);
HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	 
	 
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim9);
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_Base_Start(&htim11);
	//__HAL_TIM_ENABLE(&htim1);
	//__HAL_TIM_MOE_ENABLE(&htim1);
	 
	 
		HAL_TIM_PWM_Init(&htim1);
	 __HAL_TIM_MOE_ENABLE(&htim1);//1，8需要
	 	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  
   
	  HAL_TIM_PWM_Init(&htim9);
	 // __HAL_TIM_MOE_ENABLE(&htim9);
		HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);  
	
		HAL_TIM_PWM_Init(&htim10);
	 // __HAL_TIM_MOE_ENABLE(&htim9);
		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
		
		HAL_TIM_PWM_Init(&htim11);
	 // __HAL_TIM_MOE_ENABLE(&htim9);
		HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
		
		
 __HAL_TIM_SET_COUNTER(&htim2,0);
 __HAL_TIM_SET_COUNTER(&htim3,0);
 __HAL_TIM_SET_COUNTER(&htim4,0);
 __HAL_TIM_SET_COUNTER(&htim5,0);
 
	LF_PID_Init();  // 左前
	RF_PID_Init();  // 右前
	LR_PID_Init();  // 左后
	RR_PID_Init();  // 右后
//Set_PID_TargetSpeed(0,1);//实际上你不能给百分比，给的圈数，因为kpki无单位的
//Set_PID_TargetSpeed(0,2);	
 }
 
 //正常使用 target：目标转速比例（0~1000对应0~320rpm，负数为反转
 void Set_PID_TargetSpeed( float target,uint8_t which) 
{
    // 限制目标值范围（根据编码器实际量程，如0~100脉冲/50ms）
    if (target < Min_Pid_Value) target = Min_Pid_Value;
    if (target > Max_Pid_Value) target = Max_Pid_Value;

    // 根据电机编号设置目标值
    if (which == 1) {
        LF_AddPID.target_val = target;  // 左前
    } else if (which == 2) {
        RF_AddPID.target_val = target;  // 右前
    }
		else if (which == 3) {
        LR_AddPID.target_val = target;  // 左后
    }
				else if (which == 4) {
        RR_AddPID.target_val = target;  // 右后
    }
}
 
 //紧急操作：用set_v直接干预（如停止、急加速），此时会暂时覆盖 PID 输出。它是-1000-1000，和pwm独立,这个确实百分比
//pwm两个通道一个控制正一个控制反
 void set_v(int16_t speed,uint8_t which)//which=1=左
{
speed = (speed >Max_PWM_Value ) ? Max_PWM_Value : (speed < Min_PWM_Value) ? Min_PWM_Value : speed;
	if(speed>0)
{ 
	switch (which) {
    case 1:
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
			 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,speed);
		 
        break;  // 跳出switch结构（不执行后续case）
    case 2:
         __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,speed);
			   __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
		  
        break;  // 最后一个break可省略，但建议加上
		case 3:
         __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,speed);
			   __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,0);
		  
        break;  // 最后一个break可省略，但建议加上
		case 4:
         __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);
			   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
		  
        break;  // 最后一个break可省略，但建议加上
}
}
else if(speed<0)
{
	switch (which) {
    case 1:
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,-speed);
			 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		
        break;  // 跳出switch结构（不执行后续case）
    case 2:
         __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,-speed);
			   __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,0);
		
        break;  // 最后一个break可省略，但建议加上
		 case 3:
         __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,0);
			   __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,-speed);
		
        break;  // 最后一个break可省略，但建议加上
		 case 4:
         __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,-speed);
			   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		
        break;  // 最后一个break可省略，但建议加上
}
}
else
{
	switch (which) {
    case 1:
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
			 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
        break;  // 跳出switch结构（不执行后续case）
    case 2:
         __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
			   __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,0);
        break;  // 最后一个break可省略，但建议加上
		case 3:
         __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,0);
			   __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,0);
        break;  // 最后一个break可省略，但建议加上
		case 4:
         __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
			   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
        break;  // 最后一个break可省略，但建议加上
}
}
}
 
 
 

 
/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
  *	@note 	无
  * @retval 通过PID计算后的输出
  */
float addPID_realize(PID *pid, float actual_val)
{
	/*计算目标值与实际值的误差*/
	pid->Error = pid->target_val - actual_val;
	/*PID算法实现，照搬公式*/
	pid->output_val += pid->Kp * (pid->Error - pid-> LastError) +
	                  pid->Ki * pid->Error +
	                  pid->Kd *(pid->Error -2*pid->LastError+pid->PrevError);
	/*误差传递*/
	pid-> PrevError = pid->LastError;
	pid-> LastError = pid->Error;
	/*返回当前实际值*/
	    if(pid->output_val>Max_PWM_Value)
{
        pid->output_val=Max_PWM_Value;
        
}
		else if(pid->output_val < Min_PWM_Value) 
{
    pid->output_val = Min_PWM_Value;
}
			
	  return pid->output_val;
}


//int16_t actual1,actual2;
//int16_t targ1,targ2;
//float out1,out2;


//设置目标就行了，pid会自己调，然后给pwm
//int16_t l_total_cycle = 0; 
//int16_t r_total_cycle = 0; 
//float r_count_min;
//float l_count_min;

static int16_t LF_Encoder_Speed=0,RF_Encoder_Speed=0,LR_Encoder_Speed=0,RR_Encoder_Speed=0;
static int16_t LF_Output_Val=0,RF_Output_Val=0,LR_Output_Val=0,RR_Output_Val=0; 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

 //可以加个滤波但没必要
  //  2号定时器中断
  if(htim == (&htim2))
  {

    //  设置测量频率为20Hz(50ms)
  //  uint8_t rate = 20;
		
    //  获取编码器信号数，慢速778
     RF_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);   // which=2, 右前
		 LF_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);   // which=1, 左前
		 LR_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);   // which=3, 左后
		 RR_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);   // which=4, 右后
	  //  编码器数据清零
    __HAL_TIM_SET_COUNTER(&htim4, 0);//这里已经帮你处理正负了
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
    //0.9=(R_Encoder_Speed)*60*20/4/11/30
    // r_count_min = (float)(R_Encoder_Speed)*0.90909f;//2*2：AB向上下沿采样；11：转动一圈11个脉冲；1：30的减速比（不同电机改对应参数）,这里没用
	  // l_count_min = (float)(L_Encoder_Speed)*0.90909f;//4.0f/11.0f/30.0f;//2*2：AB向上下沿采样；11：转动一圈11个脉冲；1：30的减速比（不同电机改对应参数）
    // r_count_min=((float)(R_Encoder_Speed)*2000.0f/1556.0f);//电机的全力的百分之多少
    // l_count_min=((float)(L_Encoder_Speed)*2000.0f/1556.0f);
		
		//调试部分,(每分钟多少圈）
//		l_total_cycle+=L_Encoder_Speed;//*360.0f/(4.0f*11.0f*30.0f);
//		r_total_cycle+=R_Encoder_Speed;//*360.0f/(4.0f*11.0f*30.0f);
		//if((uint8_t)L_AddPID.target_val!=0||(uint8_t)L_AddPID.target_val!=0){
		RF_Output_Val=(int16_t)addPID_realize(&RF_AddPID,RF_Encoder_Speed);
		LF_Output_Val=(int16_t)addPID_realize(&LF_AddPID,LF_Encoder_Speed);
		LR_Output_Val=(int16_t)addPID_realize(&LR_AddPID,LR_Encoder_Speed);
		RR_Output_Val=(int16_t)addPID_realize(&RR_AddPID,RR_Encoder_Speed);
		update_odom(1000/(int16_t)RATE);
//    actual1=L_Encoder_Speed;
//    targ1=L_AddPID.target_val;
//    out1=L_Output_Val;
//    actual2=R_Encoder_Speed;
//    targ2=R_AddPID.target_val;
//    out2=R_Output_Val;
//serial_printf("left%+5d,%+5d,%+05.2f\r\n",actual1,targ1,out1);
//serial_printf("right%+5d,%+5d,%+05.2f\r\n",actual2,targ2,out2);
//serial_printf("R_Encoder_Speed%+5d,r_total_cycle%d,r_total_angle%05.2f,r_v%05.2f\r\n",
//R_Encoder_Speed,r_total_cycle,get_angular(2),get_percentspeed(2));
//serial_printf("L_Encoder_Speed%+5d,l_total_cycle%d,l_total_angle%05.2f,l_v%05.2f\r\n",
//L_Encoder_Speed,l_total_cycle,get_angular(1),get_percentspeed(1));
//    serial_printf("x%05.2f,y%05.2f,wheel_dis%f,angle%+05.2f\r\n",
//     odom_def.x_,odom_def.y_,diff_car.wheel_distance_,odom_def.angle_);


	  set_v(LF_Output_Val,1);  // which=1, 左前
	  set_v(RF_Output_Val,2);  // which=2, 右前
		set_v(LR_Output_Val,3);  // which=3, 左后
		set_v(RR_Output_Val,4);  // which=4, 右后
		send_msg(unhandle_msg);
  }

//serial_printf("%d",while_Command_GetCommand(now_command));

}
float get_angular(int16_t cycle)//读出累计角度
{
		return (float)((cycle*360.0f)/PULSE_A_circle);
}

float get_persentspeed(uint8_t which)//mm/ms==m/s
{
if (which==1)
		return (float)LF_Encoder_Speed*(PI*RADIUS/PULSE_A_circle/1000.0f)/(1.0f/RATE);
else if (which==2)
		return (float)RF_Encoder_Speed*(PI*RADIUS/PULSE_A_circle/1000.0f)/(1.0f/RATE);
else if (which==3)
		return (float)LR_Encoder_Speed*(PI*RADIUS/PULSE_A_circle/1000.0f)/(1.0f/RATE);
else if (which==4)
		return (float)RR_Encoder_Speed*(PI*RADIUS/PULSE_A_circle/1000.0f)/(1.0f/RATE);
 return 0;
}
float get_targetpulse(float targetv)//mm/ms==m/s
{
		return (float)targetv*(1.0f/RATE)/(PI*RADIUS/PULSE_A_circle/1000.0f);//t=1/f=1/20 mm/s

}
//一个脉冲是3.1415926*RADIUS/PULSE_A mm