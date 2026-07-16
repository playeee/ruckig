#pragma once

#include <array>
#include <optional>


namespace ruckig {

/**
 * @defgroup position_interfaces 位置控制接口的轨迹计算步骤
 *
 * Ruckig 算法的核心分步计算过程。对于位置控制接口，轨迹计算分为
 * Step 1（极值时间计算）和 Step 2（时间同步）两步。
 *
 * 支持三个"阶次"（order）：
 *   - FirstOrder（一阶）：仅速度约束，加加速度和加速度为无穷大
 *   - SecondOrder（二阶）：速度和加速度约束，加加速度为无穷大
 *   - ThirdOrder（三阶）：速度、加速度和加加速度约束（Type V）
 *
 * 每个阶次有独立的 Step1 和 Step2 类。
 */


// ==================== 一阶位置接口：Step 1 ====================

/**
 * @brief 一阶位置接口 Step 1：计算极值时间轨迹
 *
 * 一阶接口仅考虑速度约束（VMax/VMin），忽略加速度和加加速度限制。
 * 轨迹简化为一段恒定速度的运动。
 *
 * 计算公式：t = Δp / v，其中 v = vMax 或 vMin 取决于运动方向。
 */
class PositionFirstOrderStep1 {
    const double _vMax, _vMin;
    double pd; // 位置差 pf - p0（预处理）

public:
    explicit PositionFirstOrderStep1(double p0, double pf, double vMax, double vMin);

    bool get_profile(const Profile& input, Block& block);
};


// ==================== 一阶位置接口：Step 2 ====================

/**
 * @brief 一阶位置接口 Step 2：给定同步时间计算轨迹
 *
 * 已知同步时间 tf，计算在速度约束下从起始到目标的轨迹。
 * 速度 v = Δp / tf，检查 v 是否在 [vMin, vMax] 范围内。
 */
class PositionFirstOrderStep2 {
    const double tf, _vMax, _vMin;
    double pd; // 预处理的位置差

public:
    explicit PositionFirstOrderStep2(double tf, double p0, double pf, double vMax, double vMin);

    bool get_profile(Profile& profile);
};


// ==================== 二阶位置接口：Step 1 ====================

/**
 * @brief 二阶位置接口 Step 1：计算极值时间轨迹
 *
 * 二阶接口考虑速度约束（VMax/VMin）和加速度约束（AMax/AMin），
 * 但不考虑加加速度约束（即加加速度为无穷大）。
 *
 * 轨迹简化为三段式：
 *   阶段 0-2: 匀加速（t[0]），匀速（t[1]），匀减速（t[2]）
 *   或 无匀速段的两段式。
 *
 * 有效轮廓类型：ACC0（达到加速度上限）和 NONE（未达到限制）
 */
class PositionSecondOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, vf;
    const double _vMax, _vMin, _aMax, _aMin;

    // 预处理表达式
    double pd;

    // 最多 3 个有效轮廓 + 1 个备用（数值误差处理）
    using ProfileIter = std::array<Profile, 3>::iterator;
    std::array<Profile, 3> valid_profiles;

    void time_acc0(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, bool return_after_found) const;
    void time_none(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, bool return_after_found) const;

    // 仅用于零限制特殊情况的单段轨迹
    bool time_all_single_step(Profile* profile, double vMax, double vMin, double aMax, double aMin) const;

    inline void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit PositionSecondOrderStep1(double p0, double v0, double pf, double vf, double vMax, double vMin, double aMax, double aMin);

    bool get_profile(const Profile& input, Block& block);
};


// ==================== 二阶位置接口：Step 2 ====================

/**
 * @brief 二阶位置接口 Step 2：给定同步时间计算轨迹
 *
 * 已知同步时间 tf，在速度/加速度约束下重新计算各阶段时间。
 *
 * 求解策略：先尝试 UD 模式（先匀加速再匀减速），
 * 再尝试 UU 模式（仅匀加速），最后尝试 2 段模式。
 */
class PositionSecondOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, tf, vf;
    const double _vMax, _vMin, _aMax, _aMin;

    // 预处理表达式
    double pd, vd;

    bool time_acc0(Profile& profile, double vMax, double vMin, double aMax, double aMin);
    bool time_none(Profile& profile, double vMax, double vMin, double aMax, double aMin);

    inline bool check_all(Profile& profile, double vMax, double vMin, double aMax, double aMin) {
        return time_acc0(profile, vMax, vMin, aMax, aMin) || time_none(profile, vMax, vMin, aMax, aMin);
    }

public:
    explicit PositionSecondOrderStep2(double tf, double p0, double v0, double pf, double vf, double vMax, double vMin, double aMax, double aMin);

    bool get_profile(Profile& profile);
};


// ==================== 三阶位置接口：Step 1 ====================

/**
 * @brief 三阶位置接口 Step 1：计算极值时间轨迹
 *
 * 这是 Ruckig 的核心算法步骤。考虑完整的加加速度约束（Type V），
 * 计算单个自由度上的时间最优轨迹。
 *
 * 时间最优轨迹一定是 7 段式轮廓中的一种，由达到的约束类型决定：
 *   - ACC0_ACC1_VEL: 达到加速度上下限和速度上限（7 段全满）
 *   - ACC0_VEL:     达到加速度上限和速度上限
 *   - ACC1_VEL:     达到加速度下限和速度上限
 *   - VEL:          仅达到速度上限
 *   - ACC0_ACC1:    达到加速度上下限（无匀速段）
 *   - ACC0:         仅达到加速度上限
 *   - ACC1:         仅达到加速度下限
 *   - NONE:         未达到任何极限
 *
 * 每种类型可能有 1-2 个数学解（轮廓），对应 UDDU 和 UDUD 两种符号模式。
 * 通过 Block 类管理这些可行轮廓及其阻挡区间。
 */
class PositionThirdOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, a0;
    const double vf, af;
    const double _vMax, _vMin, _aMax, _aMin, _jMax;

    // 预处理表达式（在构造函数中预计算以提高性能）
    double pd;           // pf - p0，位置差
    double v0_v0, vf_vf; // 速度平方
    double a0_a0, a0_p3, a0_p4; // 加速度幂
    double af_af, af_p3, af_p4;
    double jMax_jMax;    // 加加速度平方

    // 最多 5 个有效轮廓 + 1 个备用（数值误差）
    using ProfileIter = std::array<Profile, 6>::iterator;
    std::array<Profile, 6> valid_profiles;

    // 各类轨迹的计算函数
    void time_all_vel(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax, bool return_after_found) const;
    void time_acc0_acc1(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax, bool return_after_found) const;
    void time_all_none_acc0_acc1(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax, bool return_after_found) const;

    // 仅用于数值误差情况的两段式回退方法
    void time_acc1_vel_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;
    void time_acc0_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;
    void time_vel_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;
    void time_none_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;

    // 仅用于零限制特殊情况
    bool time_all_single_step(Profile* profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;

    inline void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit PositionThirdOrderStep1(double p0, double v0, double a0, double pf, double vf, double af, double vMax, double vMin, double aMax, double aMin, double jMax);

    bool get_profile(const Profile& input, Block& block);
};


// ==================== 三阶位置接口：Step 2 ====================

/**
 * @brief 三阶位置接口 Step 2：时间同步
 *
 * Step 1 计算出各自由度的极值时间后，多自由度同步需要一个统一的
 * 同步时间 tf（由 TargetCalculator::synchronize() 确定）。
 * Step 2 的任务是：已知同步时间 tf，重新计算各阶段的时长分配。
 *
 * 对于每种约束类型，需要求解解析方程来得到各阶段时间。
 * 由于有多种可能的轮廓类型（ACC0_ACC1_VEL, ACC0_VEL, ACC1_VEL,
 * VEL, ACC0_ACC1, ACC0, ACC1, NONE），且每种类型有 UDDU/UDUD
 * 两种符号模式，Step 2 需要依次尝试所有可能的组合，找到第一个
 * 满足所有约束条件的有效轮廓。
 *
 * 求解过程涉及一元四次（quartic）方程的求根，使用 roots.hpp 中的
 * solve_quart_monic() 函数。
 */
class PositionThirdOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, a0;
    const double tf, vf, af;
    const double _vMax, _vMin, _aMax, _aMin, _jMax;

    // 大量预处理表达式（避免在迭代求解中重复计算）
    double pd;
    double tf_tf, tf_p3, tf_p4;  // tf 的幂
    double vd, vd_vd;
    double ad, ad_ad;
    double v0_v0, vf_vf;
    double a0_a0, a0_p3, a0_p4, a0_p5, a0_p6;
    double af_af, af_p3, af_p4, af_p5, af_p6;
    double jMax_jMax;
    double g1, g2; // 辅助表达式

    // 各类时间同步轨迹的计算函数
    bool time_acc0_acc1_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc1_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc0_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc0_acc1(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc1(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc0(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_none(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_none_smooth(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);

public:
    explicit PositionThirdOrderStep2(double tf, double p0, double v0, double a0, double pf, double vf, double af, double vMax, double vMin, double aMax, double aMin, double jMax);

    bool get_profile(Profile& profile);
};

} // namespace ruckig
