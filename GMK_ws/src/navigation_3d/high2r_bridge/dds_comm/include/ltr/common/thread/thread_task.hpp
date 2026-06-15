#ifndef __LTR_THREAD_TASK_HPP__
#define __LTR_THREAD_TASK_HPP__

#include <ltr/common/any.hpp>
#include <ltr/common/decl.hpp>
#include <ltr/common/time/time_tool.hpp>
#include <functional>
#include <memory>

namespace ltr
{
namespace common
{
class ThreadTask
{
public:
    __LTR_THREAD_DECL_TMPL_FUNC_ARG__
    explicit ThreadTask(__LTR_THREAD_TMPL_FUNC_ARG__)
    {
        mFunc = std::bind(__LTR_THREAD_BIND_FUNC_ARG__);
    }

    virtual void Execute()
    {
        if (mFunc)
        {
            mFunc();
        }
    }

    void SetEnqueueTime()
    {
        mEnqueueTimeMicrosec = GetCurrentTimeMicrosecond();
    }

    uint64_t GetEnqueueTime() const
    {
        return mEnqueueTimeMicrosec;
    }

protected:
    uint64_t mEnqueueTimeMicrosec;
    std::function<Any()> mFunc;
};

typedef std::shared_ptr<ThreadTask> ThreadTaskPtr;

}
}
#endif//__LTR_THREAD_TASK_HPP__

