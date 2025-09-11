/**
 * @file   result.hpp
 * @brief  通用结果与错误码定义（供后续强类型返回值改造使用）
 * @author Yang Nan
 * @date   2025-09-11
 */

#ifndef ROBOT_BASE_INTERFACES__COMMON__RESULT_HPP_
#define ROBOT_BASE_INTERFACES__COMMON__RESULT_HPP_

#include <optional>
#include <string>
#include <utility>

namespace robot_base_interfaces {
namespace common {

/**
 * @brief 通用错误码
 */
enum class ErrorCode {
    OK = 0,
    INVALID_ARGUMENT = 1,
    CAPABILITY_LIMITED = 2,
    COMMUNICATION_ERROR = 3,
    TIMEOUT = 4,
    NOT_IMPLEMENTED = 5,
    PERMISSION_DENIED = 6,
    UNAVAILABLE = 7,
    INTERNAL_ERROR = 100
};

/**
 * @brief 泛型结果类型（成功时携带值，失败时携带错误码与消息）
 */
template <typename T>
struct Result {
    bool ok = false;
    ErrorCode code = ErrorCode::OK;
    std::string message;     // 可选错误/提示信息
    std::optional<T> value;  // 成功时有效

    static Result<T> success(T v) {
        Result<T> r;
        r.ok = true;
        r.code = ErrorCode::OK;
        r.value = std::move(v);
        return r;
    }

    static Result<T> failure(ErrorCode c, std::string msg = "") {
        Result<T> r;
        r.ok = false;
        r.code = c;
        r.message = std::move(msg);
        return r;
    }
};

} // namespace common
} // namespace robot_base_interfaces

#endif // ROBOT_BASE_INTERFACES__COMMON__RESULT_HPP_


