#ifndef __LTR_RECURRENT_THREAD_HPP__
#define __LTR_RECURRENT_THREAD_HPP__

#include <thread>
#include <functional>
#include <atomic>
#include <chrono>
#include <memory>

namespace ltr
{
namespace common
{
class RecurrentThread
{
public:
    explicit RecurrentThread(uint64_t intervalMicrosec, std::function<void()> func)
        : mQuit(false), mIntervalMicrosec(intervalMicrosec), mFunc(func)
    {
        if (mIntervalMicrosec == 0)
        {
            mThread = std::thread(&RecurrentThread::ThreadFunc_0, this);
        }
        else
        {
            mThread = std::thread(&RecurrentThread::ThreadFunc, this);
        }
    }

    virtual ~RecurrentThread()
    {
        mQuit = true;
        if (mThread.joinable())
        {
            mThread.join();
        }
    }

    void Wait(int64_t microsec = 0)
    {
        if (microsec > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(microsec));
        }
    }

private:
    void ThreadFunc()
    {
        while (!mQuit)
        {
            if (mFunc)
            {
                mFunc();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(mIntervalMicrosec));
        }
    }

    void ThreadFunc_0()
    {
        while (!mQuit)
        {
            if (mFunc)
            {
                mFunc();
            }
        }
    }

private:
    std::atomic<bool> mQuit;
    uint64_t mIntervalMicrosec;
    std::function<void()> mFunc;
    std::thread mThread;
};

using RecurrentThreadPtr = std::shared_ptr<RecurrentThread>;

template<typename Func>
RecurrentThreadPtr CreateRecurrentThread(uint64_t intervalMicrosec, Func func)
{
    return std::make_shared<RecurrentThread>(intervalMicrosec, func);
}

}
}

#endif//__LTR_RECURRENT_THREAD_HPP__

