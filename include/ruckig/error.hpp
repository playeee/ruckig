#pragma once

#include <exception>
#include <stdexcept>
#include <string>


namespace ruckig {

/**
 * @brief Ruckig 异常基类
 *
 * 所有 Ruckig 抛出的异常都继承自该结构体。
 * 继承自 std::runtime_error，可以通过标准的异常处理机制捕获。
 * 错误消息会带有 "[ruckig]" 前缀以便识别。
 */
struct RuckigError: public std::runtime_error {
    /**
     * @param message 错误描述信息
     */
    explicit RuckigError(const std::string& message): std::runtime_error("\n[ruckig] " + message) { }
};

} // namespace ruckig
