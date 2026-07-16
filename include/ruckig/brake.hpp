#pragma once

#include <array>
#include <cmath>
#include <iostream>

#include <ruckig/utils.hpp>


namespace ruckig {

/**
 * @brief 计算制动（刹车）轨迹，使当前状态降低到运动学约束范围内
 *
 * 当初始状态的速度或加速度超出限制时，需要在主轨迹之前（或之后）
 * 添加一段制动轨迹，将状态"拉回"到约束范围内。
 *
 * 制动轨迹是一个两段式轮廓：
 *   - 第一段：以恒定加加速度 jMax 改变加速度到极限值
 *   - 第二段：以恒定加加速度 0（即恒定加速度）将速度降到极限值
 *
 * 对应论文中的"预轨迹"（pre-trajectory）概念。
 */
class BrakeProfile {
    static constexpr double eps {2.2e-14};

    /**
     * @brief 加速度制动：当前加速度超出限制时的制动策略
     *
     * 先以最大加加速度将加速度降到极限值（aMax 或 aMin），
     * 然后再以恒定加速度将速度降到极限值。
     */
    void acceleration_brake(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax);

    /**
     * @brief 速度制动：当前速度超出限制时的制动策略
     *
     * 先以最大加加速度将加速度反方向增加到极限值，利用反向加速度减速。
     * 适用于当前速度已经越限但加速度还在范围内的场景。
     */
    void velocity_brake(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax);

public:
    //! 制动轨迹的总时长
    double duration {0.0};

    /**
     * @brief 两段式制动轮廓的详细信息
     *
     * t[0], t[1]: 两段时间长度
     * j[0], j[1]: 两段的加加速度
     * a[0], a[1]: 两段开始时的加速度
     * v[0], v[1]: 两段开始时的速度
     * p[0], p[1]: 两段开始时的位置
     */
    std::array<double, 2> t, j, a, v, p;

    /**
     * @brief 计算三阶位置接口的制动轨迹（考虑加加速度约束）
     *
     * 对于 Type V 轨迹生成（考虑加加速度），制动轨迹需要平滑地
     * 将状态降低到约束范围内。
     */
    void get_position_brake_trajectory(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax);

    /**
     * @brief 计算二阶位置接口的制动轨迹（仅考虑加速度约束）
     *
     * 当 max_jerk 为无穷大（即不限制加加速度）时使用。
     * 制动仅通过加速度的变化来实现。
     */
    void get_second_order_position_brake_trajectory(double v0, double vMax, double vMin, double aMax, double aMin);

    /**
     * @brief 计算三阶速度接口的制动轨迹
     *
     * 速度控制接口中，只控制速度和加速度，位置不受直接控制。
     * 制动主要将加速度约束到范围内。
     */
    void get_velocity_brake_trajectory(double a0, double aMax, double aMin, double jMax);

    /**
     * @brief 计算二阶速度接口的制动轨迹
     *
     * 当不限制加加速度时，速度接口的制动简化为空操作。
     */
    void get_second_order_velocity_brake_trajectory();

    /**
     * @brief 完成三阶制动：沿运动学状态积分，计算制动的最终效果
     *
     * 根据计算出的两段时间，使用 integrate() 函数逐步积分，
     * 得到制动结束后的位置 ps、速度 vs、加速度 as。
     *
     * @param ps 输入当前位置，输出制动结束后的位置
     * @param vs 输入当前速度，输出制动结束后的速度
     * @param as 输入当前加速度，输出制动结束后的加速度
     */
    void finalize(double& ps, double& vs, double& as) {
        if (t[0] <= 0.0 && t[1] <= 0.0) {
            duration = 0.0;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        a[0] = as;
        std::tie(ps, vs, as) = integrate(t[0], ps, vs, as, j[0]);

        if (t[1] > 0.0) {
            duration += t[1];
            p[1] = ps;
            v[1] = vs;
            a[1] = as;
            std::tie(ps, vs, as) = integrate(t[1], ps, vs, as, j[1]);
        }
    }

    /**
     * @brief 完成二阶制动：沿运动学状态积分（无加加速度）
     *
     * 与 finalize() 类似，但仅用于二阶情况（加加速度为 0）。
     * 制动段使用恒定加速度。
     */
    void finalize_second_order(double& ps, double& vs, double& as) {
        if (t[0] <= 0.0) {
            duration = 0.0;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        std::tie(ps, vs, as) = integrate(t[0], ps, vs, a[0], 0.0);
    }
};

} // namespace ruckig
