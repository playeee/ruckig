#pragma once

#include <array>
#include <functional>
#include <tuple>
#include <vector>

#include <ruckig/error.hpp>
#include <ruckig/profile.hpp>


namespace ruckig {

template<size_t, template<class, size_t> class> class TargetCalculator;
template<size_t, template<class, size_t> class> class WaypointsCalculator;


/**
 * @brief Ruckig 算法生成的完整轨迹
 *
 * Trajectory 类存储了所有自由度上完整的运动轨迹信息，包括：
 *   - 每个自由度上的 Profile（7 段式恒加加速度轮廓）
 *   - 轨迹的总持续时间
 *   - 多段轨迹（当有中间路径点时）的累计时间
 *
 * 主要方法：
 *   - at_time(time, ...)：查询任意时刻的完整运动学状态
 *   - get_duration()：获取轨迹总时长
 *   - get_profiles()：获取底层轮廓数据
 *
 * @tparam DOFs 自由度数量，0 表示动态
 * @tparam CustomVector 自定义向量类型
 */
template<size_t DOFs, template<class, size_t> class CustomVector = StandardVector>
class Trajectory {
#if defined WITH_CLOUD_CLIENT
    // 社区版（云端）：使用 std::vector 支持多个中间路径点
    template<class T> using Container = std::vector<T>;
#else
    // 无云端版：仅支持单个轨迹段
    template<class T> using Container = std::array<T, 1>;
#endif

    template<class T> using Vector = CustomVector<T, DOFs>;

    // 声明友元类，允许它们访问内部数据
    friend class TargetCalculator<DOFs, CustomVector>;
    friend class WaypointsCalculator<DOFs, CustomVector>;

    // 所有自由度/段的轮廓数据
    Container<Vector<Profile>> profiles;

    // 总轨迹时长
    double duration {0.0};

    // 各段结束时的累计时间（用于多段轨迹）
    Container<double> cumulative_times;

    // 每个自由度的最小独立运动时间
    Vector<double> independent_min_durations;

    // 继续计算计数（用于增量计算）
    size_t continue_calculation_counter {0};

#if defined WITH_CLOUD_CLIENT
    // 调整轨迹段数量（支持多个中间路径点）
    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    void resize(size_t max_number_of_waypoints) {
        profiles.resize(max_number_of_waypoints + 1);
        cumulative_times.resize(max_number_of_waypoints + 1);
    }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    void resize(size_t max_number_of_waypoints) {
        resize<1>(max_number_of_waypoints);
        for (auto& p: profiles) {
            p.resize(degrees_of_freedom);
        }
    }
#endif

    /**
     * @brief 内部函数：获取某个时间点的积分基准状态
     *
     * 根据给定的时间，找到该时间所属的轨迹段（section）和阶段（segment），
     * 然后通过回调函数提供该阶段的起始状态（位置、速度、加速度、加加速度）
     * 以及已经过的时间偏移量，供调用者进行积分计算。
     *
     * @param time 查询时间
     * @param new_section 输出所属的轨迹段索引
     * @param set_integrate 回调函数，接收 (dof, t_diff, p, v, a, j)
     */
    template<typename Func>
    void state_to_integrate_from(double time, size_t& new_section, Func&& set_integrate) const {
        if (time >= duration) {
            // 超出轨迹时长：保持恒定加速度延伸
            new_section = profiles.size();
            const auto& profiles_dof = profiles.back();
            for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
                const double t_pre = (profiles.size() > 1) ? cumulative_times[cumulative_times.size() - 2] : profiles_dof[dof].brake.duration;
                const double t_diff = time - (t_pre + profiles_dof[dof].t_sum.back());
                set_integrate(dof, t_diff, profiles_dof[dof].p.back(), profiles_dof[dof].v.back(), profiles_dof[dof].a.back(), 0.0);
            }
            return;
        }

        // 二分查找时间点所属的轨迹段
        const auto new_section_ptr = std::upper_bound(cumulative_times.begin(), cumulative_times.end(), time);
        new_section = std::distance(cumulative_times.begin(), new_section_ptr);
        double t_diff = time;
        if (new_section > 0) {
            t_diff -= cumulative_times[new_section - 1];
        }

        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            const Profile& p = profiles[new_section][dof];
            double t_diff_dof = t_diff;

            // 处理制动段（轨迹开始前的减速部分）
            if (new_section == 0 && p.brake.duration > 0) {
                if (t_diff_dof < p.brake.duration) {
                    const size_t index = (t_diff_dof < p.brake.t[0]) ? 0 : 1;
                    if (index > 0) {
                        t_diff_dof -= p.brake.t[index - 1];
                    }
                    set_integrate(dof, t_diff_dof, p.brake.p[index], p.brake.v[index], p.brake.a[index], p.brake.j[index]);
                    continue;
                } else {
                    t_diff_dof -= p.brake.duration;
                }
            }

            // 处理超过轨迹时长的情况：保持恒定加速度
            if (t_diff_dof >= p.t_sum.back()) {
                set_integrate(dof, t_diff_dof - p.t_sum.back(), p.p.back(), p.v.back(), p.a.back(), 0.0);
                continue;
            }

            // 二分查找时间点所属的阶段
            const auto index_dof_ptr = std::upper_bound(p.t_sum.begin(), p.t_sum.end(), t_diff_dof);
            const size_t index_dof = std::distance(p.t_sum.begin(), index_dof_ptr);

            if (index_dof > 0) {
                t_diff_dof -= p.t_sum[index_dof - 1];
            }

            set_integrate(dof, t_diff_dof, p.p[index_dof], p.v[index_dof], p.a[index_dof], p.j[index_dof]);
        }
    }

public:
    size_t degrees_of_freedom;

    // ============ 构造函数 ============
    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    Trajectory(): degrees_of_freedom(DOFs) {
#if defined WITH_CLOUD_CLIENT
        resize(0);
#endif
    }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    Trajectory(size_t dofs): degrees_of_freedom(dofs) {
#if defined WITH_CLOUD_CLIENT
        resize(0);
#endif
        profiles[0].resize(dofs);
        independent_min_durations.resize(dofs);
    }

#if defined WITH_CLOUD_CLIENT
    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    Trajectory(size_t max_number_of_waypoints): degrees_of_freedom(DOFs) {
        resize(max_number_of_waypoints);
    }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    Trajectory(size_t dofs, size_t max_number_of_waypoints): degrees_of_freedom(dofs) {
        resize(max_number_of_waypoints);
        independent_min_durations.resize(dofs);
    }
#endif

    // ============ 状态查询方法 ============

    /**
     * @brief 获取轨迹在给定时间的完整运动学状态
     *
     * @param time 查询时间点
     * @param new_position 输出位置
     * @param new_velocity 输出速度
     * @param new_acceleration 输出加速度
     * @param new_jerk 输出加加速度
     * @param new_section 输出所属轨迹段
     */
    void at_time(double time, Vector<double>& new_position, Vector<double>& new_velocity, Vector<double>& new_acceleration, Vector<double>& new_jerk, size_t& new_section) const {
        if constexpr (DOFs == 0) {
            if (degrees_of_freedom != new_position.size() || degrees_of_freedom != new_velocity.size() || degrees_of_freedom != new_acceleration.size() || degrees_of_freedom != new_jerk.size()) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
        }

        state_to_integrate_from(time, new_section, [&](size_t dof, double t, double p, double v, double a, double j) {
            std::tie(new_position[dof], new_velocity[dof], new_acceleration[dof]) = integrate(t, p, v, a, j);
            new_jerk[dof] = j;
        });
    }

    /**
     * @brief 获取轨迹在给定时间的运动学状态（不含加加速度和段索引）
     */
    void at_time(double time, Vector<double>& new_position, Vector<double>& new_velocity, Vector<double>& new_acceleration) const {
        // ... 实现体同前
        if constexpr (DOFs == 0) {
            if (degrees_of_freedom != new_position.size() || degrees_of_freedom != new_velocity.size() || degrees_of_freedom != new_acceleration.size()) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
        }

        size_t new_section;
        state_to_integrate_from(time, new_section, [&](size_t dof, double t, double p, double v, double a, double j) {
            std::tie(new_position[dof], new_velocity[dof], new_acceleration[dof]) = integrate(t, p, v, a, j);
        });
    }

    // 获取位置（不含速度/加速度）
    void at_time(double time, Vector<double>& new_position) const;
    // 单自由度版本的 at_time 重载
    template<size_t D = DOFs, typename std::enable_if<(D == 1), int>::type = 0>
    void at_time(double time, double& new_position, double& new_velocity, double& new_acceleration, double& new_jerk, size_t& new_section) const;
    template<size_t D = DOFs, typename std::enable_if<(D == 1), int>::type = 0>
    void at_time(double time, double& new_position, double& new_velocity, double& new_acceleration) const;
    template<size_t D = DOFs, typename std::enable_if<(D == 1), int>::type = 0>
    void at_time(double time, double& new_position) const;

    // ============ 访问器 ============

    //! 获取底层轨迹轮廓数据
    Container<Vector<Profile>> get_profiles() const { return profiles; }

    //! 获取同步后的轨迹总时长
    double get_duration() const { return duration; }

    //! 获取各中间路径点处的累计时间
    Container<double> get_intermediate_durations() const { return cumulative_times; }

    //! 获取每个自由度的最小独立运动时间
    Vector<double> get_independent_min_durations() const { return independent_min_durations; }

    /**
     * @brief 获取所有自由度上的位置极值
     *
     * 遍历每个轨迹段中的每个自由度，计算每个自由度上的
     * 位置最小值和最大值。
     */
    void get_position_extrema(CustomVector<Bound, DOFs>& position_extrema) const {
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            position_extrema[dof] = profiles[0][dof].get_position_extrema();
        }

        for (size_t i = 1; i < profiles.size(); ++i) {
            for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
                auto section_position_extrema = profiles[i][dof].get_position_extrema();
                if (section_position_extrema.max > position_extrema[dof].max) {
                    position_extrema[dof].max = section_position_extrema.max;
                    position_extrema[dof].t_max = section_position_extrema.t_max;
                }
                if (section_position_extrema.min < position_extrema[dof].min) {
                    position_extrema[dof].min = section_position_extrema.min;
                    position_extrema[dof].t_min = section_position_extrema.t_min;
                }
            }
        }
    }

    /**
     * @brief 获取轨迹第一次到达指定位置的时间
     *
     * 通过求解三次方程找到给定位置在轨迹中出现的时间点。
     *
     * @param dof 目标自由度
     * @param position 目标位置
     * @param time_after 搜索起始时间
     * @return 时间点（如果找到），否则 std::nullopt
     */
    std::optional<double> get_first_time_at_position(size_t dof, double position, double time_after=0.0) const {
        if (dof >= degrees_of_freedom) {
            return std::nullopt;
        }

        double time;
        for (size_t i = 0; i < profiles.size(); ++i) {
            if (profiles[i][dof].get_first_state_at_position(position, time, time_after)) {
                const double section_time = (i > 0) ? cumulative_times[i-1] : 0.0;
                return section_time + time;
            }
        }
        return std::nullopt;
    }
};

} // namespace ruckig
