#include <ruckig/block.hpp>
#include <ruckig/velocity.hpp>
#include <ruckig/profile.hpp>
#include <ruckig/roots.hpp>


namespace ruckig {

/**
 * @brief 三阶速度接口 Step 2 构造函数
 *
 * 已知同步时间 tf，为速度控制接口重新计算各阶段时间。
 * 需要满足给定的加加速度约束和加速度约束。
 *
 * @param tf 同步后的轨迹时长
 * @param v0, a0 初始速度和加速度
 * @param vf, af 目标速度和加速度
 * @param aMax, aMin 加速度约束
 * @param jMax 加加速度约束
 */
VelocityThirdOrderStep2::VelocityThirdOrderStep2(double tf, double v0, double a0, double vf, double af, double aMax, double aMin, double jMax): a0(a0), tf(tf), af(af), _aMax(aMax), _aMin(aMin), _jMax(jMax) {
    vd = vf - v0;
    ad = af - a0;
}

/**
 * @brief ACC0 类型的时间同步求解
 *
 * 已知同步时间 tf，求解三段式轮廓（匀加速→匀速→匀减速）。
 *
 * 三种可能的解：
 *   1. UD 模式（标准）：先加后减
 *   2. UU 模式（仅加速）
 *   3. 2 段模式（无匀速段）
 */
bool VelocityThirdOrderStep2::time_acc0(Profile& profile, double aMax, double aMin, double jMax) {
    // UD Solution 1/2（先匀加速再匀减速的标准模式）
    {
        // 通过求解二阶方程确定各阶段时间
        const double h1 = std::sqrt((-ad*ad + 2*jMax*((a0 + af)*tf - 2*vd))/(jMax*jMax) + tf*tf);

        profile.t[0] = ad/(2*jMax) + (tf - h1)/2;
        profile.t[1] = h1;
        profile.t[2] = tf - (profile.t[0] + h1);
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::ACC0>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    // UU Solution（仅加速模式）
    {
        const double h1 = (-ad + jMax*tf);

        profile.t[0] = -ad*ad/(2*jMax*h1) + (vd - a0*tf)/h1;
        profile.t[1] = -ad/jMax + tf;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = tf - (profile.t[0] + profile.t[1]);

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::ACC0>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    // UU Solution - 2 段（无第一阶段，直接从零开始）
    {
        profile.t[0] = 0;
        profile.t[1] = -ad/jMax + tf;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = ad/jMax;

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::ACC0>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    return false;
}

/**
 * @brief NONE 类型的时间同步求解
 *
 * 未达到加速度极限的轮廓。
 * 两段式：先加后减（或先减后加）。
 *
 * 通过求解加加速度 jf 的方程来确定轮廓。
 */
bool VelocityThirdOrderStep2::time_none(Profile& profile, double aMax, double aMin, double jMax) {
    // 零初始和零终止的特殊情况
    if (std::abs(a0) < DBL_EPSILON && std::abs(af) < DBL_EPSILON && std::abs(vd) < DBL_EPSILON) {
        profile.t[0] = 0;
        profile.t[1] = tf;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::NONE>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    // UD Solution 1/2
    {
        const double h1 = 2*(af*tf - vd);

        profile.t[0] = h1/ad;
        profile.t[1] = tf - profile.t[0];
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        const double jf = ad*ad/h1;

        // 检查加加速度是否在约束范围内
        if (std::abs(jf) < std::abs(jMax) + 1e-12
            && profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::NONE>(tf, jf, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    return false;
}

/**
 * @brief 入口函数：依次尝试所有可能的轮廓
 *
 * 根据速度变化方向优先尝试对应的加速度方向，
 * 依次尝试 UD、UU 和 2 段模式的组合。
 */
bool VelocityThirdOrderStep2::get_profile(Profile& profile) {
    if (vd > 0) {
        return check_all(profile, _aMax, _aMin, _jMax) || check_all(profile, _aMin, _aMax, -_jMax);
    }

    return check_all(profile, _aMin, _aMax, -_jMax) || check_all(profile, _aMax, _aMin, _jMax);
}

} // namespace ruckig
