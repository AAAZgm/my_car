#include "kinematic.h"
#include <math.h>

WHEEL omini_car = {.wheel_base_ = 0.2f, .wheel_distance_ = 0.2f};
ODOM odom_def;

/*
 * 麦轮逆运动学 (vx, vy, wz → 4轮速度):
 *
 *      m1(LF)    m2(RF)
 *
 *      m3(LR)    m4(RR)
 *
 *   v_LF =  vx - vy - (a+b)*wz
 *   v_RF =  vx + vy + (a+b)*wz
 *   v_LR =  vx + vy - (a+b)*wz
 *   v_RR =  vx - vy + (a+b)*wz
 *
 * 正负号取决于电机安装方向和编码器极性，
 * 如果实测发现方向反了，对应项取反即可。
 */
void inverse_kinematic(float all_vx, float all_vy, float all_wz,
                       float *wheel1, float *wheel2, float *wheel3, float *wheel4) {
    const float lw = LW_SUM;

    *wheel1 =  all_vx - all_vy - lw * all_wz;  // m1 = LF (左前)
    *wheel2 =  all_vx + all_vy + lw * all_wz;  // m2 = RF (右前)
    *wheel3 =  all_vx + all_vy - lw * all_wz;  // m3 = LR (左后)
    *wheel4 =  all_vx - all_vy + lw * all_wz;  // m4 = RR (右后)
}

/*
 * 麦轮正运动学 (4轮速度 → vx, vy, wz):
 *   vx = (v1 + v2 + v3 + v4) / 4
 *   vy = (-v1 + v2 + v3 - v4) / 4
 *   wz = (-v1 + v2 - v3 + v4) / (4*(a+b))
 *
 * wheel1=LF, wheel2=RF, wheel3=LR, wheel4=RR
 */
void forward_kinematic(float wheel1, float wheel2, float wheel3, float wheel4,
                       float *all_vx, float *all_vy, float *all_wz) {
    const float lw = LW_SUM;

    *all_vx = (wheel1 + wheel2 + wheel3 + wheel4) / 4.0f;
    *all_vy = (-wheel1 + wheel2 + wheel3 - wheel4) / 4.0f;
    *all_wz = (-wheel1 + wheel2 - wheel3 + wheel4) / (4.0f * lw);
}

void update_odom(uint16_t dt_ms) {
    float dt_s = (float)dt_ms / 1000.0f;

    // 1. 正运动学: 编码器速度 → 车体速度
    forward_kinematic(get_persentspeed(1), get_persentspeed(2),
                      get_persentspeed(3), get_persentspeed(4),
                      &odom_def.vx_speed_, &odom_def.vy_speed_, &odom_def.angle_speed_);

    // 2. 更新偏航角
    odom_def.angle_ += odom_def.angle_speed_ * dt_s;
    TF_angel_PI(&odom_def.angle_);

    // 3. 车体坐标系 → 世界坐标系
    float vx_world = odom_def.vx_speed_ * cos(odom_def.angle_) - odom_def.vy_speed_ * sin(odom_def.angle_);
    float vy_world = odom_def.vx_speed_ * sin(odom_def.angle_) + odom_def.vy_speed_ * cos(odom_def.angle_);

    // 4. 更新位置
    odom_def.x_ += vx_world * dt_s;
    odom_def.y_ += vy_world * dt_s;
}

void clear_odom(ODOM *odom) {
    odom->x_ = 0.0f;
    odom->y_ = 0.0f;
    odom->angle_ = 0.0f;
    odom->vx_speed_ = 0.0f;
    odom->vy_speed_ = 0.0f;
    odom->angle_speed_ = 0.0f;
}

static void TF_angel_PI(float *angle) {
    while(*angle > PI) {
        *angle -= 2*PI;
    }
    while(*angle < -PI) {
        *angle += 2*PI;
    }
}
