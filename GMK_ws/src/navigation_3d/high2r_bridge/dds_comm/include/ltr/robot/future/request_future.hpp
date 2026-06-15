#ifndef __LTR_DDS_ROBOT_REQUEST_FUTURE_HPP__
#define __LTR_DDS_ROBOT_REQUEST_FUTURE_HPP__

#include <ltr/common/lock/lock.hpp>
#include <ltr/robot/internal/internal.hpp>
#include <memory>
#include <unordered_map>

namespace ltr
{
namespace robot
{
class RequestFutureQueue;
using RequestFutureQueuePtr = std::shared_ptr<RequestFutureQueue>;

/*
 * @brief RequestFuture - Future for async request/response
 */
class RequestFuture : public std::enable_shared_from_this<RequestFuture>
{
public:
    enum
    {
        DEFER = 0,
        READY = 1
    };

    explicit RequestFuture();
    explicit RequestFuture(int64_t requestId);
    ~RequestFuture();

    void SetRequestId(int64_t requestId);
    int64_t GetRequestId() const;

    bool SetQueue(const std::shared_ptr<RequestFutureQueue>& futureQueuePtr);

    const ResponsePtr& GetResponse(int64_t microsec);

public:
    void Ready(const ResponsePtr& requestPtr);

private:
    bool IsDeferred();
    bool IsReady();

private:
    int32_t mState;
    int64_t mRequestId;
    ResponsePtr mResponsePtr;
    common::MutexCond mMutexCond;
    RequestFutureQueuePtr mFutureQueuePtr;
};

using RequestFuturePtr = std::shared_ptr<RequestFuture>;

/*
 * @brief RequestFutureQueue - Queue for managing request futures
 */
class RequestFutureQueue
{
public:
    using iterator = std::unordered_map<int64_t,RequestFuturePtr>::iterator;
    using const_iterator = std::unordered_map<int64_t,RequestFuturePtr>::const_iterator;

    RequestFutureQueue();
    ~RequestFutureQueue();

    RequestFuturePtr Get(int64_t requestId);
    bool Put(int64_t requestId, const RequestFuturePtr& futurePtr);
    void Remove(int64_t requestId);

    size_t Size();

private:
    common::Mutex mMutex;
    std::unordered_map<int64_t,RequestFuturePtr> mFutureMap;
};

}
}

#endif//__LTR_DDS_ROBOT_REQUEST_FUTURE_HPP__

