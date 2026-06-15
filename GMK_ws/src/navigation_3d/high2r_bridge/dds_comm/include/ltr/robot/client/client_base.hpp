#ifndef __LTR_ROBOT_SDK_CLIENT_BASE_HPP__
#define __LTR_ROBOT_SDK_CLIENT_BASE_HPP__

#include <ltr/robot/client/client_stub.hpp>
#include <ltr/robot/internal/internal.hpp>
#include <string>
#include <vector>
#include <cstdint>

namespace ltr
{
namespace robot
{
/*
 * @brief Default client timeout. 1s
 */
const int64_t ROBOT_CLIENT_TIMEOUT = 1000000;

/*
 * @brief ClientBase - Base class for all robot clients
 */
class ClientBase
{
public:
    explicit ClientBase(const std::string& name);
    virtual ~ClientBase();

    virtual void Init() = 0;

    void SetTimeout(int64_t timeout);
    void SetTimeout(float timeout);

protected:
    void InitClientStub();

    int32_t Call(int32_t apiId, const std::string& parameter, std::string& data, int32_t priority, int64_t leaseId);
    int32_t Call(int32_t apiId, const std::string& parameter, int32_t priority, int64_t leaseId);

    int32_t Call(int32_t apiId, const std::vector<uint8_t>& parameter, std::vector<uint8_t>& bin_data, int32_t priority, int64_t leaseId);
    int32_t Call(int32_t apiId, const std::vector<uint8_t>& parameter, int32_t priority, int64_t leaseId);

    int32_t Call(int32_t apiId, const std::string& parameter, const std::vector<uint8_t>& binary, int32_t priority, int64_t leaseId);

    int32_t Call(int32_t apiId, const std::string& parameter, std::string& data, int32_t priority, int64_t leaseId, int64_t timeout);

    void SetHeader(RequestHeader& header, int32_t apiId, int64_t leaseId, int32_t priority, bool noReply);

protected:
    std::string mName;
    int64_t mTimeout;
    ClientStubPtr mClientStubPtr;
};

using ClientBasePtr = std::shared_ptr<ClientBase>;

}
}

#endif//__LTR_ROBOT_SDK_CLIENT_BASE_HPP__

