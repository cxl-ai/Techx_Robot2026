#ifndef __LTR_THREAD_HPP__
#define __LTR_THREAD_HPP__

#include <pthread.h>
#include <functional>
#include <memory>
#include <string>

namespace ltr
{
namespace common
{
// CPU ID constants
const int32_t LTR_CPU_ID_NONE = -1;

class Thread
{
public:
    Thread()
        : mThreadId(0), mCpuId(LTR_CPU_ID_NONE)
    {}

    Thread(const std::string& name, int32_t cpuId)
        : mThreadId(0), mName(name), mCpuId(cpuId)
    {}

    template<typename Func>
    explicit Thread(Func func)
        : Thread()
    {
        Run(func);
    }

    template<typename Func>
    explicit Thread(const std::string& name, int32_t cpuId, Func func)
        : Thread(name, cpuId)
    {
        Run(func);
    }

    virtual ~Thread();

    uint64_t GetThreadId() const;

    void SetCpu();
    void SetName();
    void SetPriority(int32_t priority);

    void Wait();

protected:
    template<typename Func>
    void Run(Func func)
    {
        mFunc = std::bind(func);
        CreateThreadNative();
    }

    void CreateThreadNative();

protected:
    pthread_t mThreadId;
    std::string mName;
    int32_t mCpuId;
    std::function<int32_t()> mFunc;
};

typedef std::shared_ptr<Thread> ThreadPtr;

template<typename Func>
static inline ThreadPtr CreateThread(Func func)
{
    return ThreadPtr(new Thread(func));
}

template<typename Func>
static inline ThreadPtr CreateThreadEx(const std::string& name, int32_t cpuId, Func func)
{
    return ThreadPtr(new Thread(name, cpuId, func));
}

}
}
#endif//__LTR_THREAD_HPP__

