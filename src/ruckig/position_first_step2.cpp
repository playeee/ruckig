#include <ruckig/block.hpp>
#include <ruckig/position.hpp>
#include <ruckig/profile.hpp>
#include <ruckig/roots.hpp>


namespace ruckig {

/**
 * @brief 一阶位置接口 Step 2 构造函数
 *
 * @param tf 同步后的目标轨迹时长
 * @param p0 初始位置
 * @param pf 目标位置
 * @param vMax 速度上限
 * @param vMin 速度下限
 */
PositionFirstOrderStep2::PositionFirstOrderStep2(double tf, double p0, double pf, double vMax, double vMin): tf(tf), _vMax(vMax), _vMin(vMin) {
    pd = pf - p0;
}

/**
 * @brief 在给定同步时间 tf 下计算一阶轨迹
 *
 * 计算公式：速度 v = Δp / tf
 * 然后检查 v 是否在 [vMin, vMax] 范围内。
 *
 * 这是最简单的 Step 2 情形——匀速运动。
 */
bool PositionFirstOrderStep2::get_profile(Profile& profile) {
    const double vf = pd / tf;

    // 设置一阶轮廓：仅匀速段 t[3] = tf
    profile.t[0] = 0;
    profile.t[1] = 0;
    profile.t[2] = 0;
    profile.t[3] = tf;
    profile.t[4] = 0;
    profile.t[5] = 0;
    profile.t[6] = 0;

    // 验证速度是否在约束范围内
    return profile.check_for_first_order_with_timing<Profile::ControlSigns::UDDU, Profile::ReachedLimits::NONE>(tf, vf, _vMax, _vMin);
}

} // namespace ruckig
