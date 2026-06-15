#ifndef __LTR_COMMON_DECL_HPP__
#define __LTR_COMMON_DECL_HPP__

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <climits>
#include <ltr/common/lock/lock.hpp>

// Common type definitions
namespace ltr
{
namespace common
{
// Exception macros
#define LTR_THROW(ExceptionType, message) \
    throw ExceptionType(message)

// Common types
// JsonMap and JsonArray are defined in json/json.hpp

// Forward declaration
class Mutex;

// Constants
#define LTR_QUEUE_MAX_LEN        INT_MAX
#define LTR_EMPTY_STR            ""

// Macros
#ifdef __GLIBC__
#define LTR_UNLIKELY(x)  __builtin_expect(!!(x), 0)
#define LTR_LIKELY(x)    __builtin_expect(!!(x), 1)
#else
#define LTR_UNLIKELY(x)  (x)
#define LTR_LIKELY(x)    (x)
#endif//__GLIBC__

// Thread template macros
#define __LTR_THREAD_DECL_TMPL_FUNC_ARG__    \
    template<class Func, class... Args>

#define __LTR_THREAD_TMPL_FUNC_ARG__         \
    Func&& func, Args&&... args

#define __LTR_THREAD_BIND_FUNC_ARG__         \
    std::forward<Func>(func), std::forward<Args>(args)...

}
}

#endif//__LTR_COMMON_DECL_HPP__

