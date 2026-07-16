#pragma once


namespace ruckig {

/**
 * @brief Ruckig update() 函数的返回结果类型枚举
 *
 * 该枚举表示轨迹计算的状态。可以通过与整数值比较来判断结果状态：
 *   - 正值或零：正常状态（Working 或 Finished）
 *   - 负值：错误状态
 */
enum Result {
    /**
     * @brief 轨迹正在正常计算中，尚未到达目标状态
     *
     * 在在线模式中，每次调用 update() 后如果返回 Working，
     * 表示轨迹仍在执行中，需要继续调用 update() 推进时间。
     */
    Working = 0,

    /**
     * @brief 轨迹已到达目标状态
     *
     * 此时轨迹已经完成，输出状态已到达目标位置/速度/加速度。
     */
    Finished = 1,

    /**
     * @brief 未分类的错误
     */
    Error = -1,

    /**
     * @brief 输入参数无效
     *
     * 输入参数中存在非法值（如 NaN、负数限制等），
     * 或者输入状态超出了约束范围。
     */
    ErrorInvalidInput = -100,

    /**
     * @brief 轨迹持续时间超出数值限制
     *
     * 计算出的轨迹时长超过约 7.6e3 秒的数值稳定上限。
     * 建议对输入进行缩放或使用 Ruckig Pro 的数值缩放功能。
     */
    ErrorTrajectoryDuration = -101,

    /**
     * @brief 轨迹超出了给定的位置约束（仅 Ruckig Pro）
     */
    ErrorPositionalLimits = -102,

    /**
     * @brief 零限制冲突导致轨迹无效
     *
     * 当某个自由度的 max_acceleration、min_acceleration 或 max_jerk
     * 为零时，可能与其他自由度的同步需求产生冲突。
     */
    ErrorZeroLimits = -104,

    /**
     * @brief 第一步（极值时间计算）出错
     *
     * 在计算单个自由度的最优时间时发生错误。
     * 通常与输入参数的有效性有关。
     */
    ErrorExecutionTimeCalculation = -110,

    /**
     * @brief 第二步（时间同步）出错
     *
     * 在多自由度时间同步过程中，找不到有效的同步时间点。
     * 可能原因是各个自由度的可行时间区间没有交集。
     */
    ErrorSynchronizationCalculation = -111,
};

} // namespace ruckig
