#include <ruckig/block.hpp>
#include <ruckig/position.hpp>


namespace ruckig {

/**
 * @brief 一阶位置接口 Step 1 构造函数
 *
 * 一阶接口仅考虑速度约束（vMax/vMin），
 * 加速度和加加速度被视为无穷大（无限制）。
 * 轨迹简化为一段匀速运动。
 *
 * @param p0 初始位置
 * @param pf 目标位置
 * @param vMax 速度上限
 * @param vMin 速度下限（应为负值）
 */
PositionFirstOrderStep1::PositionFirstOrderStep1(double p0, double pf, double vMax, double vMin): _vMax(vMax), _vMin(vMin) {
    pd = pf - p0;
}

/**
 * @brief 计算一阶极值时间轨迹
 *
 * 极值时间就是按最大速度运动所需的时间：
 *   t = |Δp| / |vMax|
 *
 * 如果 pd > 0（正向运动），使用 vMax；
 * 如果 pd < 0（反向运动），使用 vMin（负值）。
 */
bool PositionFirstOrderStep1::get_profile(const Profile& input, Block& block) {
    auto& p = block.p_min;
    p.set_boundary(input);

    // 根据运动方向选择速度方向
    const double vf = (pd > 0) ? _vMax : _vMin;

    // 一阶轮廓仅需阶段 3（匀速段），其他阶段时长为 0
    p.t[0] = 0;
    p.t[1] = 0;
    p.t[2] = 0;
    p.t[3] = pd / vf;  // t = Δp / v
    p.t[4] = 0;
    p.t[5] = 0;
    p.t[6] = 0;

    // 验证轮廓：检查匀速段的速度 vf 是否在 [vMin, vMax] 范围内
    if (p.check_for_first_order<Profile::ControlSigns::UDDU, Profile::ReachedLimits::VEL>(vf)) {
        block.t_min = p.t_sum.back() + p.brake.duration + p.accel.duration;
        return true;
    }
    return false;
}

} // namespace ruckig
