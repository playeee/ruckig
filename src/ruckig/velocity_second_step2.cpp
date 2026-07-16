#include <ruckig/block.hpp>
#include <ruckig/velocity.hpp>
#include <ruckig/profile.hpp>
#include <ruckig/roots.hpp>


namespace ruckig {

/**
 * @brief 二阶速度接口 Step 2 构造函数
 *
 * 已知同步时间 tf，计算需要的匀加速度。
 *
 * @param tf 同步后的轨迹时长
 * @param v0 初始速度
 * @param vf 目标速度
 * @param aMax, aMin 加速度约束
 */
VelocitySecondOrderStep2::VelocitySecondOrderStep2(double tf, double v0, double vf, double aMax, double aMin): tf(tf), _aMax(aMax), _aMin(aMin) {
    vd = vf - v0;
}

/**
 * @brief 计算给定同步时间下的二阶速度轨迹
 *
 * 加速度 a = Δv / tf，检查 a 是否在 [aMin, aMax] 范围内。
 * 轮廓为一段匀变速运动。
 */
bool VelocitySecondOrderStep2::get_profile(Profile& profile) {
    const double af = vd / tf;

    // 设置轮廓：仅 t[1] 段（匀变速段）
    profile.t[0] = 0;
    profile.t[1] = tf;
    profile.t[2] = 0;
    profile.t[3] = 0;
    profile.t[4] = 0;
    profile.t[5] = 0;
    profile.t[6] = 0;

    // 验证加速度是否在约束范围内
    if (profile.check_for_second_order_velocity_with_timing<Profile::ControlSigns::UDDU, Profile::ReachedLimits::NONE>(tf, af, _aMax, _aMin)) {
        profile.pf = profile.p.back();
        return true;
    }

    return false;
}

} // namespace ruckig
