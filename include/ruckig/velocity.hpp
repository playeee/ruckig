#pragma once

#include <array>
#include <optional>


namespace ruckig {

/**
 * @defgroup velocity_interfaces 速度控制接口的轨迹计算步骤
 *
 * 速度控制接口与位置接口的区别在于：
 * 位置接口控制完整的运动学状态（位置、速度、加速度），
 * 速度接口仅控制速度和加速度，不直接控制位置。
 *
 * 因此，速度接口的轨迹计算中不包含位置差 pd 的约束，
 * 只关注速度变化 vd = vf - v0 和加速度变化 ad = af - a0。
 *
 * 适用于：停止轨迹、视觉伺服等不需要精确位置控制的场景。
 */


// ==================== 二阶速度接口：Step 1 ====================

/**
 * @brief 二阶速度接口 Step 1：计算极值时间轨迹
 *
 * 仅考虑加速度约束（AMax/AMin），不考虑加加速度。
 * 轨迹简化为两段式：匀加速到目标速度。
 */
class VelocitySecondOrderStep1 {
    const double _aMax, _aMin;
    double vd; // 速度差 vf - v0

public:
    explicit VelocitySecondOrderStep1(double v0, double vf, double aMax, double aMin);

    bool get_profile(const Profile& input, Block& block);
};


// ==================== 二阶速度接口：Step 2 ====================

/**
 * @brief 二阶速度接口 Step 2：给定同步时间计算轨迹
 *
 * 已知同步时间 tf，计算加速度 a = Δv / tf。
 */
class VelocitySecondOrderStep2 {
    const double tf, _aMax, _aMin;
    double vd;

public:
    explicit VelocitySecondOrderStep2(double tf, double v0, double vf, double aMax, double aMin);

    bool get_profile(Profile& profile);
};


// ==================== 三阶速度接口：Step 1 ====================

/**
 * @brief 三阶速度接口 Step 1：计算极值时间轨迹
 *
 * 考虑完整的加加速度约束，但只针对速度和加速度控制。
 * 类似于三阶位置接口，但省略了位置相关的约束。
 *
 * 有效轮廓类型：ACC0（达到加速度上限）和 NONE（未达到限制）
 */
class VelocityThirdOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double a0, af;
    const double _aMax, _aMin, _jMax;

    // 预处理表达式
    double vd;

    // 最多 3 个有效轮廓
    using ProfileIter = std::array<Profile, 3>::iterator;
    std::array<Profile, 3> valid_profiles;

    void time_acc0(ProfileIter& profile, double aMax, double aMin, double jMax, bool return_after_found) const;
    void time_none(ProfileIter& profile, double aMax, double aMin, double jMax, bool return_after_found) const;

    // 仅用于零限制特殊情况
    bool time_all_single_step(Profile* profile, double aMax, double aMin, double jMax) const;

    inline void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit VelocityThirdOrderStep1(double v0, double a0, double vf, double af, double aMax, double aMin, double jMax);

    bool get_profile(const Profile& input, Block& block);
};


// ==================== 三阶速度接口：Step 2 ====================

/**
 * @brief 三阶速度接口 Step 2：给定同步时间计算轨迹
 *
 * 类似三阶位置接口 Step 2，但仅处理速度/加速度约束。
 * 尝试 UD 模式（先加速再减速）或 UU 模式（仅加速）。
 */
class VelocityThirdOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double a0, tf, af;
    const double _aMax, _aMin, _jMax;

    // 预处理表达式
    double vd, ad;

    bool time_acc0(Profile& profile, double aMax, double aMin, double jMax);
    bool time_none(Profile& profile, double aMax, double aMin, double jMax);

    inline bool check_all(Profile& profile, double aMax, double aMin, double jMax) {
        return time_acc0(profile, aMax, aMin, jMax) || time_none(profile, aMax, aMin, jMax);
    }

public:
    explicit VelocityThirdOrderStep2(double tf, double v0, double a0, double vf, double af, double aMax, double aMin, double jMax);

    bool get_profile(Profile& profile);
};

} // namespace ruckig
