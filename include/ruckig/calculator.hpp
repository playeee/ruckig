#pragma once

#include <ruckig/calculator_target.hpp>
#ifdef WITH_CLOUD_CLIENT
#include <ruckig/calculator_cloud.hpp>
#endif
#include <ruckig/input_parameter.hpp>
#include <ruckig/trajectory.hpp>


namespace ruckig {

/**
 * @brief 主计算器的内部接口及超参数
 *
 * Calculator 是一个调度类，根据输入是否包含中间路径点，
 * 选择合适的计算器：
 *   - TargetCalculator: 状态到状态的轨迹计算（无中间路径点）
 *   - WaypointsCalculator: 带中间路径点的轨迹计算（社区版通过云端 API）
 *
 * 对于社区版（WITH_CLOUD_CLIENT 启用），带路径点的计算会通过 HTTP
 * 请求发送到云端服务器执行。
 */
template<size_t DOFs, template<class, size_t> class CustomVector = StandardVector>
class Calculator {
    /**
     * @brief 判断是否使用路径点轨迹计算
     *
     * 当输入包含中间路径点且控制接口为位置控制时，
     * 需要使用 WaypointsCalculator。
     */
    inline bool use_waypoints_trajectory(const InputParameter<DOFs, CustomVector>& input) {
        return !input.intermediate_positions.empty() && input.control_interface == ControlInterface::Position;
    }

public:
    // 状态到状态的计算器
    TargetCalculator<DOFs, CustomVector> target_calculator;

#if defined WITH_CLOUD_CLIENT
    // 带中间路径点的计算器（社区版通过云端 API）
    WaypointsCalculator<DOFs, CustomVector> waypoints_calculator;
#endif

    // ============ 构造函数 ============
    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    explicit Calculator() { }

#if defined WITH_CLOUD_CLIENT
    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    explicit Calculator(size_t max_waypoints):
        waypoints_calculator(WaypointsCalculator<DOFs, CustomVector>(max_waypoints))
        { }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit Calculator(size_t dofs):
        target_calculator(TargetCalculator<DOFs, CustomVector>(dofs)),
        waypoints_calculator(WaypointsCalculator<DOFs, CustomVector>(dofs))
        { }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit Calculator(size_t dofs, size_t max_waypoints):
        target_calculator(TargetCalculator<DOFs, CustomVector>(dofs)),
        waypoints_calculator(WaypointsCalculator<DOFs, CustomVector>(dofs, max_waypoints))
        { }
#else
    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit Calculator(size_t dofs): target_calculator(TargetCalculator<DOFs, CustomVector>(dofs)) { }
#endif

    /**
     * @brief 计算时间最优轨迹
     *
     * 根据输入参数，自动选择使用 TargetCalculator 或 WaypointsCalculator。
     *
     * @tparam throw_error 是否在出错时抛出异常
     * @param input 输入参数
     * @param trajectory 输出的轨迹
     * @param delta_time 控制周期
     * @param was_interrupted 输出计算是否被中断
     * @return 计算结果状态
     */
    template<bool throw_error>
    Result calculate(const InputParameter<DOFs, CustomVector>& input, Trajectory<DOFs, CustomVector>& trajectory, double delta_time, bool& was_interrupted) {
        Result result;
#if defined WITH_CLOUD_CLIENT
        if (use_waypoints_trajectory(input)) {
            result = waypoints_calculator.template calculate<throw_error>(input, trajectory, delta_time, was_interrupted);
        } else {
            result = target_calculator.template calculate<throw_error>(input, trajectory, delta_time, was_interrupted);
        }
#else
        result = target_calculator.template calculate<throw_error>(input, trajectory, delta_time, was_interrupted);
#endif
        return result;
    }

    /**
     * @brief 继续轨迹计算（增量计算，仅 Ruckig Pro）
     *
     * 当上次计算被中断时，可以继续计算以 refine 轨迹。
     */
    template<bool throw_error>
    Result continue_calculation(const InputParameter<DOFs, CustomVector>& input, Trajectory<DOFs, CustomVector>& trajectory, double delta_time, bool& was_interrupted) {
        Result result;
#if defined WITH_CLOUD_CLIENT
        if (use_waypoints_trajectory(input)) {
            result = waypoints_calculator.template continue_calculation<throw_error>(input, trajectory, delta_time, was_interrupted);
        } else {
            result = target_calculator.template continue_calculation<throw_error>(input, trajectory, delta_time, was_interrupted);
        }
#else
        result = target_calculator.template continue_calculation<throw_error>(input, trajectory, delta_time, was_interrupted);
#endif
        return result;
    }
};

} // namespace ruckig
