#ifndef __LTR_TIME_SLEEP_HPP__
#define __LTR_TIME_SLEEP_HPP__

#include <cstdint>

namespace ltr
{
namespace common
{
// Sleep functions
void MicroSleep(int64_t microseconds);
void MilliSleep(int64_t milliseconds);
void Sleep(int64_t seconds);

}
}

#endif//__LTR_TIME_SLEEP_HPP__

