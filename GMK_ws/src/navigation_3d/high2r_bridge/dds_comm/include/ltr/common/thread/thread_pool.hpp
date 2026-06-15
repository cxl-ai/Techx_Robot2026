#ifndef __LTR_THREAD_POOL_HPP__
#define __LTR_THREAD_POOL_HPP__

#include <ltr/common/thread/thread.hpp>
#include <ltr/common/thread/thread_task.hpp>
#include <ltr/common/block_queue.hpp>
#include <ltr/common/decl.hpp>
#include <ltr/common/time/time_tool.hpp>
#include <vector>
#include <memory>

namespace ltr
{
namespace common
{
class ThreadPool
{
public:
    enum
    {
        MIN_THREAD_NUMBER = 1,
        MAX_THREAD_NUMBER = 1000,
        QUEUE_GET_TIMEOUT_MICROSEC = 1000000,
        MAX_QUEUE_SIZE = LTR_QUEUE_MAX_LEN,
        MAX_QUEUE_MICROSEC = 25200000000  // 7 days
    };

    explicit ThreadPool(uint32_t threadNumber = MIN_THREAD_NUMBER,
        uint32_t queueMaxSize = LTR_QUEUE_MAX_LEN,
        uint64_t taskMaxQueueMicrosec = MAX_QUEUE_MICROSEC);

    ~ThreadPool();

    __LTR_THREAD_DECL_TMPL_FUNC_ARG__
    bool AddTask(__LTR_THREAD_TMPL_FUNC_ARG__)
    {
        ThreadTaskPtr taskPtr = std::make_shared<ThreadTask>(__LTR_THREAD_BIND_FUNC_ARG__);
        return AddTaskInner(taskPtr);
    }

    int32_t DoTask();
    uint64_t GetTaskSize();

    bool IsQuit();
    void Quit(bool waitThreadExit = true);

    bool IsTaskOverdue(uint64_t enqueueTime);

private:
    bool AddTaskInner(ThreadTaskPtr taskptr);

    void InitCreateThread();
    void WaitThreadExit();

private:
    volatile bool mQuit;

    uint32_t mThreadNumber;
    uint32_t mTaskQueueMaxSize;
    uint64_t mTaskMaxQueueTime;

    BlockQueue<ThreadTaskPtr> mTaskQueue;
    std::vector<ThreadPtr> mThreadList;
};

typedef std::shared_ptr<ThreadPool> ThreadPoolPtr;

}
}
#endif//__LTR_THREAD_POOL_HPP__

