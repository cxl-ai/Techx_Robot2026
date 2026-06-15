#ifndef __LTR_ROBOT_SDK_SERVER_STUB_HPP__
#define __LTR_ROBOT_SDK_SERVER_STUB_HPP__

#include <ltr/robot/internal/internal.hpp>
#include <ltr/robot/channel/channel_labor.hpp>
#include <ltr/common/thread/thread.hpp>
#include <ltr/common/block_queue.hpp>
#include <functional>
#include <memory>
#include <string>

namespace ltr
{
namespace robot
{
using ServerRequestHandler = std::function<void(const RequestPtr& requestPtr)>;

class ServerStub
{
public:
    explicit ServerStub();
    ~ServerStub();

    void Init(const std::string& name, const ServerRequestHandler& handler, bool enableProiQueue = false);
    bool Send(const Response& response, int64_t timeout = 0);

private:
    void Enqueue(const void* message);
    void RequestFunc(const void* message);

    int32_t QueueThreadFunction();
    int32_t ProiQueueThreadFunction();

private:
    bool mEnableProiQueue;
    bool mRunning;
    ServerRequestHandler mRequestHandler;
    ServerChannelLaborPtr<Response,Request> mChannelLaborPtr;
    common::BlockQueuePtr<RequestPtr> mQueuePtr;
    common::BlockQueuePtr<RequestPtr> mProiQueuePtr;
    common::ThreadPtr mQueueThreadPtr;
    common::ThreadPtr mProiQueueThreadPtr;
};

using ServerStubPtr = std::shared_ptr<ServerStub>;

}
}

#endif//__LTR_ROBOT_SDK_SERVER_STUB_HPP__

