#pragma once

#include <algorithm>
#include <limits>
#include <numeric>
#include <optional>
#include <string>

#include <ruckig/profile.hpp>


namespace ruckig {

/**
 * @brief 表示单个自由度可行轨迹时间的"阻挡区间"
 *
 * Ruckig 算法核心思想（对应论文）：
 * Step 1 为每个自由度计算时间最优轨迹，得到最短可能时间 t_min。
 * 但有时最短时间轨迹不一定是能用的——其他自由度可能无法在这个时间
 * 完成同步。于是，对于每个自由度，除了最短时间 t_min 之外，还存在
 * 一些"不可用的时间区间"（被阻挡的区间），在这些区间内没有有效的轨迹。
 *
 * 一个 Block 包含：
 *   - p_min: 时间最优（最短）的轨迹轮廓
 *   - t_min: 对应的最短时间
 *   - a, b: 最多两个被阻挡的区间（即不可用的时间段）
 *
 * 所有可选的时间点按升序排列为：t_min, a->left, a->right, b->left, b->right
 * 其中 t_min 和 a->left 之间、a->right 和 b->left 之间是可行区间。
 */
class Block {
    /**
     * @brief 从有效轮廓列表中移除指定索引的轮廓
     *
     * 用于处理数值不稳定性导致的冗余轮廓。
     */
    template<size_t N>
    inline static void remove_profile(std::array<Profile, N>& valid_profiles, size_t& valid_profile_counter, size_t index) {
        for (size_t i = index; i < valid_profile_counter - 1; ++i) {
            valid_profiles[i] = valid_profiles[i + 1];
        }
        valid_profile_counter -= 1;
    }

public:
    /**
     * @brief 表示一个被阻挡的时间区间 [left, right]
     *
     * 在该区间内没有有效的轨迹轮廓。
     * profile 成员保存区间右端点对应的轮廓（用于 Step 2 的时间同步）。
     */
    struct Interval {
        double left, right; // [s]
        Profile profile; // 右端点对应的轮廓

        explicit Interval(double left, double right): left(left), right(right) { }

        /**
         * @brief 由两个轮廓构造区间，取两者时间较短者为 left
         */
        explicit Interval(const Profile& profile_left, const Profile& profile_right) {
            const double left_duration = profile_left.t_sum.back() + profile_left.brake.duration + profile_left.accel.duration;
            const double right_duration = profile_right.t_sum.back() + profile_right.brake.duration + profile_right.accel.duration;
            if (left_duration < right_duration) {
                left = left_duration;
                right = right_duration;
                profile = profile_right;
            } else {
                left = right_duration;
                right = left_duration;
                profile = profile_left;
            }
        };
    };

    Profile p_min; // 保存最短时间轮廓，避免在 Step 2 中重新计算
    double t_min; // 最短时间 [s]

    // 最多两个被阻挡的区间（a 和 b），顺序无关
    std::optional<Interval> a, b;

    /**
     * @brief 从一组有效轮廓中计算 Block（阻挡区间）
     *
     * 有效轮廓的数量决定阻挡区间的结构：
     *   - 1 个轮廓：只有 t_min，无阻挡区间
     *   - 2 个轮廓：若时间不同，t_min 对应较短的，另一个形成阻挡区间 a
     *   - 3 个轮廓：t_min 对应最短的，其余两个形成一个阻挡区间 a
     *   - 5 个轮廓：t_min + 两个阻挡区间 a 和 b
     *
     * @tparam N 有效轮廓数组的大小
     * @tparam numerical_robust 是否启用数值鲁棒性处理
     */
    template<size_t N, bool numerical_robust = true>
    static bool calculate_block(Block& block, std::array<Profile, N>& valid_profiles, size_t valid_profile_counter) {
        if (valid_profile_counter == 1) {
            block.set_min_profile(valid_profiles[0]);
            return true;

        } else if (valid_profile_counter == 2) {
            if (std::abs(valid_profiles[0].t_sum.back() - valid_profiles[1].t_sum.back()) < 8 * std::numeric_limits<double>::epsilon()) {
                block.set_min_profile(valid_profiles[0]);
                return true;
            }

            if constexpr (numerical_robust) {
                const size_t idx_min = (valid_profiles[0].t_sum.back() < valid_profiles[1].t_sum.back()) ? 0 : 1;
                const size_t idx_else_1 = (idx_min + 1) % 2;

                block.set_min_profile(valid_profiles[idx_min]);
                block.a = Interval(valid_profiles[idx_min], valid_profiles[idx_else_1]);
                return true;
            }

        // 以下情况仅因数值误差导致
        } else if (valid_profile_counter == 4) {
            // 查找"相同"的轮廓（方向不同但时间相近）
            if (std::abs(valid_profiles[0].t_sum.back() - valid_profiles[1].t_sum.back()) < 32 * std::numeric_limits<double>::epsilon() && valid_profiles[0].direction != valid_profiles[1].direction) {
                remove_profile<N>(valid_profiles, valid_profile_counter, 1);
            } else if (std::abs(valid_profiles[2].t_sum.back() - valid_profiles[3].t_sum.back()) < 256 * std::numeric_limits<double>::epsilon() && valid_profiles[2].direction != valid_profiles[3].direction) {
                remove_profile<N>(valid_profiles, valid_profile_counter, 3);
            } else if (std::abs(valid_profiles[0].t_sum.back() - valid_profiles[3].t_sum.back()) < 256 * std::numeric_limits<double>::epsilon() && valid_profiles[0].direction != valid_profiles[3].direction) {
                remove_profile<N>(valid_profiles, valid_profile_counter, 3);
            } else {
                return false;
            }

        } else if (valid_profile_counter % 2 == 0) {
            return false;
        }

        // 找到时间最短的轮廓
        const auto idx_min_it = std::min_element(valid_profiles.cbegin(), valid_profiles.cbegin() + valid_profile_counter, [](const Profile& a, const Profile& b) { return a.t_sum.back() < b.t_sum.back(); });
        const size_t idx_min = std::distance(valid_profiles.cbegin(), idx_min_it);

        block.set_min_profile(valid_profiles[idx_min]);

        if (valid_profile_counter == 3) {
            const size_t idx_else_1 = (idx_min + 1) % 3;
            const size_t idx_else_2 = (idx_min + 2) % 3;

            block.a = Interval(valid_profiles[idx_else_1], valid_profiles[idx_else_2]);
            return true;

        } else if (valid_profile_counter == 5) {
            const size_t idx_else_1 = (idx_min + 1) % 5;
            const size_t idx_else_2 = (idx_min + 2) % 5;
            const size_t idx_else_3 = (idx_min + 3) % 5;
            const size_t idx_else_4 = (idx_min + 4) % 5;

            if (valid_profiles[idx_else_1].direction == valid_profiles[idx_else_2].direction) {
                block.a = Interval(valid_profiles[idx_else_1], valid_profiles[idx_else_2]);
                block.b = Interval(valid_profiles[idx_else_3], valid_profiles[idx_else_4]);
            } else {
                block.a = Interval(valid_profiles[idx_else_1], valid_profiles[idx_else_4]);
                block.b = Interval(valid_profiles[idx_else_2], valid_profiles[idx_else_3]);
            }
            return true;
        }

        return false;
    }

    void set_min_profile(const Profile& profile) {
        p_min = profile;
        t_min = p_min.t_sum.back() + p_min.brake.duration + p_min.accel.duration;
        a = std::nullopt;
        b = std::nullopt;
    }

    /**
     * @brief 判断给定时间点是否在阻挡区间内
     *
     * 如果 t < t_min，或 t 在区间 a 或 b 内，则被阻挡。
     */
    inline bool is_blocked(double t) const {
        return (t < t_min) || (a && a->left < t && t < a->right) || (b && b->left < t && t < b->right);
    }

    /**
     * @brief 获取给定时间点对应的轨迹轮廓
     *
     * 根据时间 t 从大到小匹配：优先匹配 b 区间右端点，然后 a 区间右端点，
     * 最后使用最短时间轮廓。
     */
    const Profile& get_profile(double t) const {
        if (b && t >= b->right) {
            return b->profile;
        }
        if (a && t >= a->right) {
            return a->profile;
        }
        return p_min;
    }

    std::string to_string() const {
        std::string result = "[" + std::to_string(t_min) + " ";
        if (a) {
            result += std::to_string(a->left) + "] [" + std::to_string(a->right) + " ";
        }
        if (b) {
            result += std::to_string(b->left) + "] [" + std::to_string(b->right) + " ";
        }
        return result + "-";
    }
};

} // namespace ruckig
