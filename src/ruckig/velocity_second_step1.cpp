#include <ruckig/block.hpp>
#include <ruckig/velocity.hpp>


namespace ruckig {

/**
 * @brief 二阶速度接口 Step 1 构造函数
 *
 * 速度接口仅控制速度和加速度。
 * 二阶速度接口只考虑加速度约束（aMax/aMin）。
 * 轨迹简化为一段匀变速运动。
 */
VelocitySecondOrderStep1::VelocitySecondOrderStep1(double v0, double vf, double aMax, double aMin): _aMax(aMax), _aMin(aMin) {
    vd = vf - v0;
}

/**
 * @brief 计算二阶速度接口的极值时间轨迹
 *
 * 极值时间由最大加速度决定：
 *   t = |Δv| / |aMax|
 *
 * 公式：vf = v0 + a*t → t = (vf - v0) / a
 * 如果 vd > 0（加速），使用 aMax；否则使用 aMin（减速）。
 */
bool VelocitySecondOrderStep1::get_profile(const Profile& input, Block& block) {
    auto& p = block.p_min;
    p.set_boundary(input);

    // 根据速度变化方向选择加速度方向
    const double af = (vd > 0) ? _aMax : _aMin;

    // 二阶速度轮廓：仅 t[1] 段（匀变速段），其他为 0
    p.t[0] = 0;
    p.t[1] = vd / af;  // t = Δv / a
    p.t[2] = 0;
    p.t[3] = 0;
    p.t[4] = 0;
    p.t[5] = 0;
    p.t[6] = 0;

    // 验证加速度是否在约束范围内
    if (p.check_for_second_order_velocity<Profile::ControlSigns::UDDU, Profile::ReachedLimits::ACC0>(af)) {
        block.t_min = p.t_sum.back() + p.brake.duration + p.accel.duration;
        return true;
    }
    return false;
}

} // namespace ruckig
