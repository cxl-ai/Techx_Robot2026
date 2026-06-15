#ifndef __LTR_COMMON_LOCK_HPP__
#define __LTR_COMMON_LOCK_HPP__

#include <mutex>
#include <condition_variable>
#include <chrono>

namespace ltr
{
namespace common
{

// Mutex wrapper for compatibility
class Mutex
{
public:
    void Lock()
    {
        mMutex.lock();
    }

    void Unlock()
    {
        mMutex.unlock();
    }

    bool Trylock()
    {
        return mMutex.try_lock();
    }

    std::mutex& GetNative()
    {
        return mMutex;
    }

private:
    std::mutex mMutex;
};

// Condition variable wrapper
class Cond
{
public:
    Cond() = default;
    ~Cond() = default;

    void Wait(Mutex& mutex)
    {
        std::unique_lock<std::mutex> lock(mutex.GetNative(), std::adopt_lock);
        mCond.wait(lock);
        lock.release();
    }

    bool Wait(Mutex& mutex, uint64_t microsec)
    {
        std::unique_lock<std::mutex> lock(mutex.GetNative(), std::adopt_lock);
        auto timeout = std::chrono::microseconds(microsec);
        bool result = mCond.wait_for(lock, timeout) == std::cv_status::no_timeout;
        lock.release();
        return result;
    }

    void Notify()
    {
        mCond.notify_one();
    }

    void NotifyAll()
    {
        mCond.notify_all();
    }

private:
    std::condition_variable mCond;
};

// Mutex + Condition variable combined
class MutexCond
{
public:
    MutexCond() = default;
    ~MutexCond() = default;

    void Lock()
    {
        mMutex.Lock();
    }

    void Unlock()
    {
        mMutex.Unlock();
    }

    bool Wait(int64_t microsec = 0)
    {
        if (microsec == 0)
        {
            mCond.Wait(mMutex);
            return true;
        }
        else
        {
            return mCond.Wait(mMutex, static_cast<uint64_t>(microsec));
        }
    }

    void Notify()
    {
        mCond.Notify();
    }

    void NotifyAll()
    {
        mCond.NotifyAll();
    }

private:
    Mutex mMutex;
    Cond mCond;
};

// Lock guard template
template<typename LOCK_TYPE>
class LockGuard
{
public:
    explicit LockGuard(LOCK_TYPE& lockPtr)
    {
        mLockPtr = &lockPtr;
        mLockPtr->Lock();
    }

    explicit LockGuard(LOCK_TYPE* lockPtr)
    {
        mLockPtr = lockPtr;
        mLockPtr->Lock();
    }

    ~LockGuard()
    {
        mLockPtr->Unlock();
    }

private:
    LOCK_TYPE* mLockPtr;
};

}
}

#endif//__LTR_COMMON_LOCK_HPP__
