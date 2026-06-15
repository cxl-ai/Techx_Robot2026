#ifndef __LTR_ASSERT_HPP__
#define __LTR_ASSERT_HPP__

#include <ltr/common/decl.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <sys/syscall.h>
#include <unistd.h>
#include <cerrno>

#define LTR_ASSERT_OUT(debug, file, func, line, r)           \
    if (debug)                                              \
    {                                                       \
        std::cout << "[" << ::time(NULL)                    \
            << "] [" << ::syscall(SYS_gettid)               \
            << "] LTR_ASSERT DEBUG at __FILE__:" << file     \
            << ", __FUNCTION__:" << func                    \
            << ", __LINE__:" << line                        \
            << ", r:" << r                                  \
            << ", errno:" << errno                          \
            << std::endl;                                   \
    }                                                       \
    else                                                    \
    {                                                       \
        std::cout << "[" << ::time(NULL)                    \
            << "] [" << ::syscall(SYS_gettid)               \
            << "] LTR_ASSERT ABORT at __FILE__:" << file     \
            << ", __FUNCTION__:" << func                    \
            << ", __LINE__:" << line                        \
            << ", r:" << r                                  \
            << ", errno:" << errno                          \
            << std::endl;                                   \
    }

#define LTR_ASSERT_ABORT(debug, file, func, line, r)         \
    if (debug)                                              \
    {                                                       \
        LTR_ASSERT_OUT(1, file, func, line, r);              \
    }                                                       \
    else                                                    \
    {                                                       \
        LTR_ASSERT_OUT(0, file, func, line, r);              \
        abort();                                            \
    }

#define LTR_ASSERT_EQ(x, r)                                  \
    ltr::common::AssertEqual(x, r, 0, __FILE__,                 \
        __PRETTY_FUNCTION__, __LINE__)

#define LTR_ASSERT_EQ_DEBUG(x, r)                            \
    ltr::common::AssertEqual(x, r, 1, __FILE__,                 \
        __PRETTY_FUNCTION__, __LINE__)

#define LTR_ASSERT_NOT_EQ(x, r)                              \
    ltr::common::AssertNotEqual(x, r, 0, __FILE__,              \
        __PRETTY_FUNCTION__, __LINE__)

#define LTR_ASSERT_NOT_EQ_DEBUG(x, r)                        \
    ltr::common::AssertNotEqual(x, r, 1, __FILE__,              \
        __PRETTY_FUNCTION__, __LINE__)

#define LTR_ASSERT_ENO_EQ(x, r, eno)                         \
    ltr::common::AssertEqual(x, r, eno, 0, __FILE__,            \
        __PRETTY_FUNCTION__, __LINE__)

#define LTR_ASSERT_ENO_EQ_DEBUG(x, r, eno)                   \
    ltr::common::AssertEqual(x, r, eno, 1, __FILE__,            \
        __PRETTY_FUNCTION__, __LINE__)

#define LTR_ASSERT_0(x)                  \
    LTR_ASSERT_EQ(x, 0)

#define LTR_ASSERT_DEBUG_0(x)            \
    LTR_ASSERT_EQ_DEBUG(x, 0)

namespace ltr
{
namespace common
{
inline int AssertEqual(int r, int expectRet, bool debug,
    const char* file, const char* func, int line)
{
    if (LTR_UNLIKELY(r != expectRet))
    {
        LTR_ASSERT_ABORT(debug, file, func, line, r);
    }

    return r;
}

inline int AssertNotEqual(int r, int expectRet, bool debug,
    const char* file, const char* func, int line)
{
    if (LTR_UNLIKELY(r == expectRet))
    {
        LTR_ASSERT_ABORT(debug, file, func, line, r);
    }

    return r;
}

inline int AssertEqual(int r, int expectRet, int expectErrno, bool debug,
    const char* file, const char* func, int line)
{
    if (LTR_UNLIKELY(r != expectRet) && LTR_UNLIKELY(errno != expectErrno))
    {
        LTR_ASSERT_ABORT(debug, file, func, line, r);
    }

    return r;
}
}
}
#endif//__LTR_ASSERT_HPP__

