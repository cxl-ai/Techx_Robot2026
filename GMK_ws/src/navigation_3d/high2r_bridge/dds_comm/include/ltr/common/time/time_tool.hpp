#ifndef __LTR_TIME_TOOL_HPP__
#define __LTR_TIME_TOOL_HPP__

#include <cstdint>

namespace ltr
{
namespace common
{
// Time utility functions
int64_t GetCurrentTimeMillisecond();
int64_t GetCurrentTimeMicrosecond();
int64_t GetCurrentTimeNanosecond();
int64_t GetCurrentMonotonicTimeNanosecond();

}
}

#endif//__LTR_TIME_TOOL_HPP__

