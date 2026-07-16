#include <ruckig/block.hpp>
#include <ruckig/position.hpp>


namespace ruckig {

/**
 * @brief 二阶位置接口 Step 1 构造函数
 *
 * 二阶接口考虑速度约束和加速度约束，但不考虑加加速度约束。
 * 轨迹简化为梯形速度轮廓（匀加速→匀速→匀减速）或三角形速度轮廓。
 */
PositionSecondOrderStep1::PositionSecondOrderStep1(double p0, double v0, double pf, double vf, double vMax, double vMin, double aMax, double aMin): v0(v0), vf(vf), _vMax(vMax), _vMin(vMin), _aMax(aMax), _aMin(aMin) {
    pd = pf - p0;
}

/**
 * @brief 计算 ACC0 类型轮廓（达到加速度上限）
 *
 * 三段式：匀加速到 vMax → 匀速保持 vMax → 匀减速到 vf
 * 对应梯形速度轮廓，加速度在加速段达到 aMax，减速段达到 aMin。
 *
 * 各段时间计算公式：
 *   t[0] = (vMax - v0) / aMax      （加速时间）
 *   t[1] = 中间匀速段时间            （通过位置差方程求解）
 *   t[2] = (vf - vMax) / aMin       （减速时间，aMin 为负值）
 */
void PositionSecondOrderStep1::time_acc0(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, bool) const {
    profile->t[0] = (-v0 + vMax)/aMax;
    profile->t[1] = (aMin*v0*v0 - aMax*vf*vf)/(2*aMax*aMin*vMax) + vMax*(aMax - aMin)/(2*aMax*aMin) + pd/vMax;
    profile->t[2] = (vf - vMax)/aMin;
    profile->t[3] = 0;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::ACC0>(aMax, aMin, vMax, vMin)) {
        add_profile(profile);
    }
}

/**
 * @brief 计算 NONE 类型轮廓（未达到加速度极限）
 *
 * 两段式：匀加速 → 匀减速（无匀速段）
 * 对应三角形速度轮廓，速度从 v0 变化到 vf，中间经过一个峰值。
 * 加速度既不达到 aMax 也不达到 aMin。
 *
 * 通过求解二次方程确定峰值速度。
 */
void PositionSecondOrderStep1::time_none(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, bool return_after_found) const {
    // 求解峰值速度的平方
    double h1 = (aMax*vf*vf - aMin*v0*v0 - 2*aMax*aMin*pd)/(aMax - aMin);
    if (h1 >= 0.0) {
        h1 = std::sqrt(h1);

        // 解 1：先加速再减速
        {
            profile->t[0] = -(v0 + h1)/aMax;  // 加速段（负值表示加速度方向与速度相反）
            profile->t[1] = 0;
            profile->t[2] = (vf + h1)/aMin;   // 减速段
            profile->t[3] = 0;
            profile->t[4] = 0;
            profile->t[5] = 0;
            profile->t[6] = 0;

            if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(aMax, aMin, vMax, vMin)) {
                add_profile(profile);
                if (return_after_found) return;
            }
        }

        // 解 2：先减速再加速（少见情况）
        {
            profile->t[0] = (-v0 + h1)/aMax;
            profile->t[1] = 0;
            profile->t[2] = (vf - h1)/aMin;
            profile->t[3] = 0;
            profile->t[4] = 0;
            profile->t[5] = 0;
            profile->t[6] = 0;

            if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(aMax, aMin, vMax, vMin)) {
                add_profile(profile);
            }
        }
    }
}

/**
 * @brief 零限制特殊情况：单段轨迹
 *
 * 当 vMax = vMin = 0 时，只有起始和结束速度均为零才有意义。
 * 如果 pd = 0，直接原地不动；否则无法运动（速度为零不能移动）。
 */
bool PositionSecondOrderStep1::time_all_single_step(Profile* profile, double vMax, double vMin, double, double) const {
    if (std::abs(vf - v0) > DBL_EPSILON) {
        return false;
    }

    profile->t[0] = 0;
    profile->t[1] = 0;
    profile->t[2] = 0;
    profile->t[3] = 0;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    if (std::abs(v0) > DBL_EPSILON) {
        profile->t[3] = pd / v0;
        if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(0.0, 0.0, vMax, vMin)) {
            return true;
        }

    } else if (std::abs(pd) < DBL_EPSILON) {
        if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(0.0, 0.0, vMax, vMin)) {
            return true;
        }
    }

    return false;
}

/**
 * @brief 入口函数：计算所有可能的极值轨迹轮廓
 *
 * 策略：
 * 1. 处理零限制特殊情况
 * 2. 对于 vf=0 的情况，先尝试一种方向，找到即返回
 * 3. 对于一般情况，尝试两种方向（正向/反向）的两种轮廓（ACC0/NONE）
 *
 * 通过 Block::calculate_block 管理有效轮廓。
 */
bool PositionSecondOrderStep1::get_profile(const Profile& input, Block& block) {
    // 零限制特殊情况
    if (_vMax == 0.0 && _vMin == 0.0) {
        auto& p = block.p_min;
        p.set_boundary(input);

        if (time_all_single_step(&p, _vMax, _vMin, _aMax, _aMin)) {
            block.t_min = p.t_sum.back() + p.brake.duration + p.accel.duration;
            if (std::abs(v0) > DBL_EPSILON) {
                block.a = Block::Interval(block.t_min, std::numeric_limits<double>::infinity());
            }
            return true;
        }
        return false;
    }

    const ProfileIter start = valid_profiles.begin();
    ProfileIter profile = start;
    profile->set_boundary(input);

    if (std::abs(vf) < DBL_EPSILON) {
        // vf=0 时不存在阻挡区间，找到就返回
        const double vMax = (pd >= 0) ? _vMax : _vMin;
        const double vMin = (pd >= 0) ? _vMin : _vMax;
        const double aMax = (pd >= 0) ? _aMax : _aMin;
        const double aMin = (pd >= 0) ? _aMin : _aMax;

        time_none(profile, vMax, vMin, aMax, aMin, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, vMax, vMin, aMax, aMin, true);
        if (profile > start) { goto return_block; }

        time_none(profile, vMin, vMax, aMin, aMax, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, vMin, vMax, aMin, aMax, true);

    } else {
        // 一般情况：尝试两种方向的两种轮廓
        time_none(profile, _vMax, _vMin, _aMax, _aMin, false);
        time_none(profile, _vMin, _vMax, _aMin, _aMax, false);
        time_acc0(profile, _vMax, _vMin, _aMax, _aMin, false);
        time_acc0(profile, _vMin, _vMax, _aMin, _aMax, false);
    }

return_block:
    return Block::calculate_block(block, valid_profiles, std::distance(start, profile));
}

} // namespace ruckig
