#pragma once

#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>


namespace ruckig {

/**
 * @brief 表示动态（运行时设置）自由度数的常量
 *
 * 当自由度数量在编译期未知时使用此常量作为模板参数。
 * 此时内部向量类型会从 std::array 切换为 std::vector。
 */
constexpr static size_t DynamicDOFs {0};

/**
 * @brief 基于 C++ 标准库的向量数据类型
 *
 * 根据 DOFs 是否在编译期已知（>= 1），自动选择：
 *   - std::array<T, DOFs>（静态大小，栈分配，性能更优）
 *   - std::vector<T>（动态大小，堆分配，灵活）
 *
 * @tparam T 元素类型（通常是 double 或 bool）
 * @tparam DOFs 自由度数量，0 表示动态
 */
template<class T, size_t DOFs>
using StandardVector = typename std::conditional<DOFs >= 1, std::array<T, DOFs>, std::vector<T>>::type;

/**
 * @brief 固定大小的标准向量类型
 *
 * 与 StandardVector 类似但允许指定与 DOFs 不同的大小 SIZE。
 * 用于需要固定大小但不同于自由度数的场景（如同步时间候选列表）。
 */
template<class T, size_t DOFs, size_t SIZE>
using StandardSizeVector = typename std::conditional<DOFs >= 1, std::array<T, SIZE>, std::vector<T>>::type;


// 如果包含 Eigen 且版本符合要求，提供 Eigen 向量类型别名
#ifdef EIGEN_VERSION_AT_LEAST
#if EIGEN_VERSION_AT_LEAST(3,3,7)
    template<class T, size_t DOFs> using EigenVector = typename std::conditional<DOFs >= 1, Eigen::Matrix<T, DOFs, 1>, Eigen::Matrix<T, Eigen::Dynamic, 1>>::type;
#endif
#endif


/**
 * @brief 将数组/向量的元素用逗号连接成字符串
 *
 * @param array 要格式化的数组或向量
 * @param high_precision 是否使用高精度（16位小数）输出
 * @return 格式化后的字符串
 */
template<class Vector>
inline std::string join(const Vector& array, bool high_precision = false) {
    std::ostringstream ss;
    for (size_t i = 0; i < array.size(); ++i) {
        if (i) ss << ", ";
        if (high_precision) ss << std::setprecision(16);
        ss << array[i];
    }
    return ss.str();
}

/**
 * @brief 在恒定加加速度（jerk）下积分运动学状态
 *
 * 给定初始位置 p0、速度 v0、加速度 a0 和恒定加加速度 j，
 * 经过时间 t 后的运动学状态由以下公式计算：
 *   p(t) = p0 + v0*t + a0*t²/2 + j*t³/6
 *   v(t) = v0 + a0*t + j*t²/2
 *   a(t) = a0 + j*t
 *
 * 这是 Ruckig 中所有轨迹计算的基础运动学方程。
 * 轨迹由分段恒定加加速度（加加速度）的多个阶段组成。
 *
 * @param t 经过的时间
 * @param p0 初始位置
 * @param v0 初始速度
 * @param a0 初始加速度
 * @param j 恒定加加速度
 * @return 元组 (新位置, 新速度, 新加速度)
 */
inline std::tuple<double, double, double> integrate(double t, double p0, double v0, double a0, double j) {
    return std::make_tuple(
        p0 + t * (v0 + t * (a0 / 2 + t * j / 6)),
        v0 + t * (a0 + t * j / 2),
        a0 + t * j
    );
}

} // namespace ruckig
