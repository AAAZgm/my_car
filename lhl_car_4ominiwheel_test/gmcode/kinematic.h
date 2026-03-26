#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__

#define WHEEL_DISTANCE  0.2f   // 轮距（左右轮中心距）m
#define WHEEL_BASE      0.2f   // 轴距（前后轮中心距）m
// a = WHEEL_BASE/2 (半轴距), b = WHEEL_DISTANCE/2 (半轮距)
// 麦轮运动学关键参数: a + b
#define LW_SUM  ((WHEEL_DISTANCE + WHEEL_BASE) / 2.0f)

#ifdef __cplusplus
extern "C" {
#endif

#include "..\Core\Inc\main.h"

/*  四轮全向轮布局:
 *      m1(LF)    m2(RF)    ← 前方
 *
 *      m3(LR)    m4(RR)
 *
 *  前进 = X轴增加 (vx > 0)
 *  左平 = Y轴增加 (vy > 0)
 *  正旋转 = 逆时针 (wz > 0)
 */
typedef struct
{
	float wheel_base_;       // 轴距（前后轮中心距）m
	float wheel_distance_;   // 轮距（左右轮中心距）m
	float wheel1_v;          // m1 = 左前(LF), which=1
	float wheel2_v;          // m2 = 右前(RF), which=2
	float wheel3_v;          // m3 = 左后(LR), which=3
	float wheel4_v;          // m4 = 右后(RR), which=4
}WHEEL;

//里程计
typedef struct
{
	float  vy_speed_;
	float  vx_speed_;
	float angle_speed_;     // rad/s
	float x_;
	float y_;               // m
	float angle_;           // rad
}ODOM;

extern WHEEL omini_car;
extern ODOM odom_def;

void forward_kinematic(float wheel1, float wheel2, float wheel3, float wheel4,
                       float *all_vx, float *all_vy, float *all_wz);
void inverse_kinematic(float all_vx, float all_vy, float all_wz,
                       float *wheel1, float *wheel2, float *wheel3, float *wheel4);
void clear_odom(ODOM *odom);
void update_odom(uint16_t dt_ms);
static void TF_angel_PI(float *angle);
#ifdef __cplusplus
}
#endif

#endif
