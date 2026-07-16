#include <ruckig/block.hpp>
#include <ruckig/velocity.hpp>


namespace ruckig {

/**
 * @brief 三阶速度接口 Step 1 构造函数
 *
 * 三阶速度接口考虑加加速度约束和加速度约束。
 * 与位置接口的区别在于不包含位置约束，仅关注速度变化 vd = vf - v0。
 *
 * 轨迹可以是三段式（ACC0：加速→匀速→减速）或两段式（NONE：加速→减速）。
 */
VelocityThirdOrderStep1::VelocityThirdOrderStep1(double v0, double a0, double vf, double af, double aMax, double aMin, double jMax): a0(a0), af(af), _aMax(aMax), _aMin(aMin), _jMax(jMax) {
    vd = vf - v0;
}

/**
 * @brief 计算 ACC0 类型轮廓（达到加速度上限）
 *
 * 三段式加加速度轮廓：
 *   阶段 0: +jMax 增加加速度到 aMax
 *   阶段 1: 保持恒定加速度 aMax（匀变速度段）
 *   阶段 2: -jMax 降低加速度到目标值
 *
 * 各阶段时间由运动学公式推导得出。
 */
void VelocityThirdOrderStep1::time_acc0(ProfileIter& profile, double aMax, double aMin, double jMax, bool) const {
    profile->t[0] = (-a0 + aMax)/jMax;           // 加速到 aMax 的时间
    profile->t[1] = (a0*a0 + af*af)/(2*aMax*jMax) - aMax/jMax + vd/aMax;  // 匀加速度段时间
    profile->t[2] = (-af + aMax)/jMax;           // 减速到目标加速度的时间
    profile->t[3] = 0;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    if (profile->check_for_velocity<ControlSigns::UDDU, ReachedLimits::ACC0>(jMax, aMax, aMin)) {
        add_profile(profile);
    }
}

/**
 * @brief 计算 NONE 类型轮廓（未达到加速度极限）
 *
 * 两段式：先以 +jMax 改变加速度，然后以 -jMax 回到目标加速度。
 * 峰值加速度通过求解二次方程确定。
 *
 * 对应三角形加加速度轮廓。
 */
void VelocityThirdOrderStep1::time_none(ProfileIter& profile, double aMax, double aMin, double jMax, bool return_after_found) const {
    double h1 = (a0*a0 + af*af)/2 + jMax*vd;
    if (h1 >= 0.0) {
        h1 = std::sqrt(h1);

        // 解 1：先加后减
        {
            profile->t[0] = -(a0 + h1)/jMax;
            profile->t[1] = 0;
            profile->t[2] = -(af + h1)/jMax;
            profile->t[3] = 0;
            profile->t[4] = 0;
            profile->t[5] = 0;
            profile->t[6] = 0;

            if (profile->check_for_velocity<ControlSigns::UDDU, ReachedLimits::NONE>(jMax, aMax, aMin)) {
                add_profile(profile);
                if (return_after_found) return;
            }
        }

        // 解 2：先减后加（少见）
        {
            profile->t[0] = (-a0 + h1)/jMax;
            profile->t[1] = 0;
            profile->t[2] = (-af + h1)/jMax;
            profile->t[3] = 0;
            profile->t[4] = 0;
            profile->t[5] = 0;
            profile->t[6] = 0;

            if (profile->check_for_velocity<ControlSigns::UDDU, ReachedLimits::NONE>(jMax, aMax, aMin)) {
                add_profile(profile);
            }
        }
    }
}

/**
 * @brief 零限制特殊情况：单段轨迹
 *
 * 当 jMax = 0 时，仅当起始和结束加速度相同才有意义。
 * 轮廓为一段匀变速运动。
 */
bool VelocityThirdOrderStep1::time_all_single_step(Profile* profile, double aMax, double aMin, double) const {
    if (std::abs(af - a0) > DBL_EPSILON) {
        return false;
    }

    profile->t[0] = 0;
    profile->t[1] = 0;
    profile->t[2] = 0;
    profile->t[3] = 0;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    if (std::abs(a0) > DBL_EPSILON) {
        profile->t[3] = vd / a0;
        if (profile->check_for_velocity<ControlSigns::UDDU, ReachedLimits::NONE>(0.0, aMax, aMin)) {
            return true;
        }

    } else if (std::abs(vd) < DBL_EPSILON) {
        if (profile->check_for_velocity<ControlSigns::UDDU, ReachedLimits::NONE>(0.0, aMax, aMin)) {
            return true;
        }
    }

    return false;
}

/**
 * @brief 入口函数：计算所有可能的极值轨迹轮廓
 *
 * 与位置接口类似，尝试两种方向（正向/反向）的两种轮廓（ACC0/NONE）。
 */
bool VelocityThirdOrderStep1::get_profile(const Profile& input, Block& block) {
    if (_jMax == 0.0) {
        auto& p = block.p_min;
        p.set_boundary(input);

        if (time_all_single_step(&p, _aMax, _aMin, _jMax)) {
            block.t_min = p.t_sum.back() + p.brake.duration + p.accel.duration;
            if (std::abs(a0) > DBL_EPSILON) {
                block.a = Block::Interval(block.t_min, std::numeric_limits<double>::infinity());
            }
            return true;
        }
        return false;
    }

    const ProfileIter start = valid_profiles.begin();
    ProfileIter profile = start;
    profile->set_boundary(input);

    if (std::abs(af) < DBL_EPSILON) {
        const double aMax = (vd >= 0) ? _aMax : _aMin;
        const double aMin = (vd >= 0) ? _aMin : _aMax;
        const double jMax = (vd >= 0) ? _jMax : -_jMax;

        time_none(profile, aMax, aMin, jMax, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, aMax, aMin, jMax, true);
        if (profile > start) { goto return_block; }

        time_none(profile, aMin, aMax, -jMax, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, aMin, aMax, -jMax, true);

    } else {
        time_none(profile, _aMax, _aMin, _jMax, false);
        time_none(profile, _aMin, _aMax, -_jMax, false);
        time_acc0(profile, _aMax, _aMin, _jMax, false);
        time_acc0(profile, _aMin, _aMax, -_jMax, false);
    }

return_block:
    return Block::calculate_block(block, valid_profiles, std::distance(start, profile));
}

} // namespace ruckig
