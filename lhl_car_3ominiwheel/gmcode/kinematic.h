#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__

//#define WHEEL_DIS 0.25f //m
#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"

typedef struct
{
    float robot_radius_;   // 轮子到中心距离
   // float wheel_radius_;   // 轮子半径
    float wheel1_v;        // 前
    float wheel2_v;        // 左后
    float wheel3_v;        // 右后
	//这两乘了1000
//	float angular_v; //弧度/s
//	float cmd_vel; //m/s
}WHEEL_3;

//这些是小车实际的
typedef struct
{
    float  vy_speed_;
    float  vx_speed_;
//	float liner_speed_;//m/s,用于·计算
    float angle_speed_;//m/s,用于·计算
    float x_;
    float y_;//m
    float angle_;//弧度
}ODOM;

extern WHEEL_3 omini3_car;
extern ODOM odom3_def;

void forward_kinematic_3wheel(float wheel1, float wheel2, float wheel3,
                               float *all_vx, float *all_vy, float *all_wz);
void inverse_kinematic_3wheel(float all_vx, float all_vy, float all_wz,
                               float *wheel1, float *wheel2, float *wheel3);
void update_odom_3wheel(uint16_t dt_ms);
void clear_odom_3wheel(ODOM *odom);
static void TF_angel_PI(float *angle); //被 static 修饰的函数（也叫 “静态函数”），只能在其定义所在的 .c 文件（编译单元）内被访问和调用
#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
