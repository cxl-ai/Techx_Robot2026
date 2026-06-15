#ifndef __LTR_ROBOT_SDK_CLIENT_STUB_HPP__
#define __LTR_ROBOT_SDK_CLIENT_STUB_HPP__

#include <ltr/robot/future/request_future.hpp>
#include <ltr/robot/channel/channel_labor.hpp>
#include <ltr/robot/internal/internal.hpp>
#include <string>
#include <memory>

namespace ltr
{
namespace robot
{
class ClientStub
{
public:
    explicit ClientStub();
    ~ClientStub();

    void Init(const std::string& name);

    bool Send(const Request& req, int64_t waitTimeout);
    RequestFuturePtr SendRequest(const Request& req, int64_t waitTimeout);

private:
    void ResponseFunc(const void* message);

private:
    ClientChannelLaborPtr<Request,Response> mChannelLaborPtr;
    RequestFutureQueuePtr mFutureQueuePtr;
};

using ClientStubPtr = std::shared_ptr<ClientStub>;

}
}

#endif//__LTR_ROBOT_SDK_CLIENT_STUB_HPP__
