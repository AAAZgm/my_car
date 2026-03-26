#include "kinematic.h"
#include <math.h>
#define SQRT3_2  0.866025f  // √3/2
#define SQRT3    1.732051f  // √3
#define PI       3.1415926f

WHEEL_3 omini3_car = {
    .robot_radius_ = 0.15f,   // 轮子到中心距离
};
ODOM odom3_def;
/*       wheel1 (前/M1/编号1)
          ↑
         /|\
        / | \
       /  |  \
 wheel2 ←--+--→ wheel3
(左/M2/编号4)  (右/M3/编号2)
M3代表正X轴
M1代表正Y轴
*/
/*
 * 轮子方向（辊子朝向）：
 * wheel1: 0°   (水平向右)
 * wheel2: 120° (左后方向)
 * wheel3: 240° (右后方向)
 */
void forward_kinematic_3wheel(float wheel1, float wheel2, float wheel3,
                               float *all_vx, float *all_vy, float *all_wz) {
    const float R = omini3_car.robot_radius_;
    
    // 正确公式
    *all_vx = (2.0f * wheel1 - wheel2 - wheel3) / 3.0f;
    *all_vy = (wheel2 - wheel3) / SQRT3;  // SQRT3 = 1.732051
    *all_wz = (wheel1 + wheel2 + wheel3) / (3.0f * R);
}


void inverse_kinematic_3wheel(float all_vx, float all_vy, float all_wz,
                               float *wheel1, float *wheel2, float *wheel3) {
    const float R = omini3_car.robot_radius_;
    
    // 逆运动学公式
    *wheel1 = all_vx + R * all_wz;
    *wheel2 = -0.5f * all_vx + SQRT3_2 * all_vy + R * all_wz;
    *wheel3 = -0.5f * all_vx - SQRT3_2 * all_vy + R * all_wz;
}

void update_odom_3wheel(uint16_t dt_ms) {
    float dt_s = (float)dt_ms / 1000.0f;
    
    // 1. 计算车体速度
    forward_kinematic_3wheel(
        get_persentspeed(1), get_persentspeed(4), get_persentspeed(2),
        &odom3_def.vx_speed_, &odom3_def.vy_speed_, &odom3_def.angle_speed_
    );
    
    // 2. 更新偏航角
    odom3_def.angle_ += odom3_def.angle_speed_ * dt_s;
    
    // 角度归一化 [-π, π]

    TF_angel_PI(&odom3_def.angle_);
    // 3. 车体坐标系 → 世界坐标系
    float vx_world = odom3_def.vx_speed_ * cosf(odom3_def.angle_) 
                   - odom3_def.vy_speed_ * sinf(odom3_def.angle_);
    float vy_world = odom3_def.vx_speed_ * sinf(odom3_def.angle_) 
                   + odom3_def.vy_speed_ * cosf(odom3_def.angle_);
    
    // 4. 更新位置
    odom3_def.x_ += vx_world * dt_s;
    odom3_def.y_ += vy_world * dt_s;
}


/**
 * 清空里程计数据
 * @param odom: 里程计结构体指针
 */
void clear_odom_3wheel(ODOM *odom) {
    odom->x_ = 0.0f;
    odom->y_ = 0.0f;
    odom->angle_ = 0.0f;
    odom->vx_speed_ = 0.0f;
    odom->vy_speed_ = 0.0f;
    odom->angle_speed_ = 0.0f;
   // odom->liner_speed_ = 0.0f;
}
static void TF_angel_PI(float *angle) {
    while(*angle > PI) {
        *angle -= 2*PI;
    }
    while(*angle < -PI) {
        *angle += 2*PI;
    }
}
