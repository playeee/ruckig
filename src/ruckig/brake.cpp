#include <ruckig/brake.hpp>


namespace ruckig {

/**
 * @brief 辅助函数：给定初始速度和加速度，在恒定加加速度下计算 t 时刻的速度
 *
 * v(t) = v0 + a0*t + j*t²/2
 */
inline double v_at_t(double v0, double a0, double j, double t) {
    return v0 + t * (a0 + j * t / 2);
}

/**
 * @brief 辅助函数：计算加速度过零时的速度
 *
 * 当加速度从 a0 以加加速度 j 变化到零时，经过时间 t = -a0/j，
 * 此时的速度为 v = v0 + a0²/(2j)
 */
inline double v_at_a_zero(double v0, double a0, double j) {
    return v0 + (a0 * a0)/(2 * j);
}

// ==================== 加速度制动 ====================

/**
 * @brief 加速度制动：当前加速度超出最大/最小加速度限制时使用
 *
 * 策略：
 * 1. 以最大加加速度 jMax 降低（或升高）加速度到极限值 aMax/aMin
 * 2. 如果此时速度仍然超限，继续以恒定极限加速度减速/加速
 *
 * 判断逻辑：
 *   - 如果加速度过零时的速度仍超过 vMax → 需要速度制动（更激进）
 *   - 如果到达 aMax 时的速度低于 vMin（反向越限）→ 需要两段制动
 *   - 否则一段制动即可
 */
void BrakeProfile::acceleration_brake(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax) {
    j[0] = -jMax;

    // 到达加速度上限所需时间
    const double t_to_a_max = (a0 - aMax) / jMax;
    // 加速度降到零所需时间
    const double t_to_a_zero = a0 / jMax;

    // 到达 aMax 时的速度
    const double v_at_a_max = v_at_t(v0, a0, -jMax, t_to_a_max);
    // 加速度过零时的速度
    const double v_at_a_zero = v_at_t(v0, a0, -jMax, t_to_a_zero);

    if ((v_at_a_zero > vMax && jMax > 0) || (v_at_a_zero < vMax && jMax < 0)) {
        // 加速度过零时速度仍超限 → 需要更激进的制动策略
        velocity_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

    } else if ((v_at_a_max < vMin && jMax > 0) || (v_at_a_max > vMin && jMax < 0)) {
        // 到达 aMax 时速度已经反向超限 → 两段制动
        const double t_to_v_min = -(v_at_a_max - vMin)/aMax;
        const double t_to_v_max = -aMax/(2*jMax) - (v_at_a_max - vMax)/aMax;

        t[0] = t_to_a_max + eps;
        t[1] = std::max(std::min(t_to_v_min, t_to_v_max - eps), 0.0);

    } else {
        // 简单情况：仅需一段制动将加速度降到 aMax
        t[0] = t_to_a_max + eps;
    }
}

// ==================== 速度制动 ====================

/**
 * @brief 速度制动：当前速度超出限制时的制动策略
 *
 * 当加速度过零时速度仍超过极限，说明仅降低加速度还不够，
 * 需要将加速度反向增大（朝相反方向加速）来更快地减速。
 *
 * 策略：
 * 1. 先以最大加加速度将加速度降到最小值 aMin（反向最大加速度）
 * 2. 然后保持 aMin，以恒定反向加速度减速到速度极限
 *
 * 这相当于同时使用加速度和速度约束的协同制动。
 */
void BrakeProfile::velocity_brake(double v0, double a0, double vMax, double vMin, double, double aMin, double jMax) {
    j[0] = -jMax;

    // 到达加速度最小值的时间
    const double t_to_a_min = (a0 - aMin)/jMax;
    // 到达速度最大值的时间（通过求解二次方程）
    const double t_to_v_max = a0/jMax + std::sqrt(a0*a0 + 2 * jMax * (v0 - vMax)) / std::abs(jMax);
    // 到达速度最小值的时间
    const double t_to_v_min = a0/jMax + std::sqrt(a0*a0 / 2 + jMax * (v0 - vMin)) / std::abs(jMax);

    const double t_min_to_v_max = std::min(t_to_v_max, t_to_v_min);

    if (t_to_a_min < t_min_to_v_max) {
        // 先到达 aMin（先改变加速度），再以恒定 aMin 减速
        const double v_at_a_min = v_at_t(v0, a0, -jMax, t_to_a_min);
        const double t_to_v_max_with_constant = -(v_at_a_min - vMax)/aMin;
        const double t_to_v_min_with_constant = aMin/(2*jMax) - (v_at_a_min - vMin)/aMin;

        t[0] = std::max(t_to_a_min - eps, 0.0);
        t[1] = std::max(std::min(t_to_v_max_with_constant, t_to_v_min_with_constant), 0.0);

    } else {
        // 直接到达速度极限（加速度不需要到极限值）
        t[0] = std::max(t_min_to_v_max - eps, 0.0);
    }
}


// ==================== 接口函数 ====================

/**
 * @brief 计算位置控制接口的制动轨迹（三阶，考虑加加速度约束）
 *
 * 判断逻辑链：
 *   1. a0 > aMax → 加速度制动（正向越限）
 *   2. a0 < aMin → 加速度制动（反向越限，符号取反）
 *   3. v0 > vMax 且减速到零时仍超速 → 速度制动
 *   4. v0 < vMin 且加速到零时仍超速 → 速度制动（反向）
 *   5. 否则无需制动
 */
void BrakeProfile::get_position_brake_trajectory(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax) {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;

    if (jMax == 0.0 || aMax == 0.0 || aMin == 0.0) {
        return; // 零限制情况下忽略制动
    }

    if (a0 > aMax) {
        acceleration_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

    } else if (a0 < aMin) {
        acceleration_brake(v0, a0, vMin, vMax, aMin, aMax, -jMax);

    } else if ((v0 > vMax && v_at_a_zero(v0, a0, -jMax) > vMin) || (a0 > 0 && v_at_a_zero(v0, a0, jMax) > vMax)) {
        velocity_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

    } else if ((v0 < vMin && v_at_a_zero(v0, a0, jMax) < vMax) || (a0 < 0 && v_at_a_zero(v0, a0, -jMax) < vMin)) {
        velocity_brake(v0, a0, vMin, vMax, aMin, aMax, -jMax);
    }
}

/**
 * @brief 计算位置控制接口的制动轨迹（二阶，仅加速度约束）
 *
 * 当 max_jerk 无穷大时使用。
 * 仅检查速度是否越限，若是则使用恒定反向加速度减速。
 */
void BrakeProfile::get_second_order_position_brake_trajectory(double v0, double vMax, double vMin, double aMax, double aMin) {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;
    a[0] = 0.0;
    a[1] = 0.0;

    if (aMax == 0.0 || aMin == 0.0) {
        return;
    }

    if (v0 > vMax) {
        // 使用反向最大加速度减速
        a[0] = aMin;
        t[0] = (vMax - v0)/aMin + eps;

    } else if (v0 < vMin) {
        a[0] = aMax;
        t[0] = (vMin - v0)/aMax + eps;
    }
}

/**
 * @brief 计算速度控制接口的制动轨迹（三阶）
 *
 * 仅检查加速度是否越限，若是则通过加加速度改变加速度到极限值。
 */
void BrakeProfile::get_velocity_brake_trajectory(double a0, double aMax, double aMin, double jMax) {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;

    if (jMax == 0.0) {
        return;
    }

    if (a0 > aMax) {
        // 以 -jMax 减小加速度到 aMax
        j[0] = -jMax;
        t[0] = (a0 - aMax)/jMax + eps;

    } else if (a0 < aMin) {
        j[0] = jMax;
        t[0] = -(a0 - aMin)/jMax + eps;
    }
}

/**
 * @brief 计算速度控制接口的制动轨迹（二阶）
 *
 * 无加加速度约束时，速度接口的制动始终为空操作。
 */
void BrakeProfile::get_second_order_velocity_brake_trajectory() {
    t[0] = 0.0;
    t[1] = 0.0;
    j[0] = 0.0;
    j[1] = 0.0;
}

} // namespace ruckig
