#ifndef __LTR_COMMON_EXCEPTION_HPP__
#define __LTR_COMMON_EXCEPTION_HPP__

#include <stdexcept>
#include <string>

namespace ltr
{
namespace common
{

class CommonException : public std::runtime_error
{
public:
    explicit CommonException(const std::string& message);
};

class TimeoutException : public CommonException
{
public:
    explicit TimeoutException(const std::string& message) : CommonException(message) {}
};

class BadCastException : public CommonException
{
public:
    explicit BadCastException(const std::string& message) : CommonException(message) {}
};

}
}

#endif//__LTR_COMMON_EXCEPTION_HPP__

