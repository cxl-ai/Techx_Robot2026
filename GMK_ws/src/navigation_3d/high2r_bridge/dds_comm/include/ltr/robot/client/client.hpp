#ifndef __LTR_ROBOT_SDK_CLIENT_HPP__
#define __LTR_ROBOT_SDK_CLIENT_HPP__

#include <ltr/robot/client/client_base.hpp>
#include <ltr/robot/client/lease_client.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <cstdint>

#define LTR_ROBOT_CLIENT_REG_API_NO_PRIO(apiId) \
    LTR_ROBOT_CLIENT_REG_API(apiId, 0)

#define LTR_ROBOT_CLIENT_REG_API(apiId, priority) \
    RegistApi(apiId, priority)

namespace ltr
{
namespace robot
{
/*
 * @brief Client - Base client implementation with lease support
 */
class Client: public ClientBase
{
public:
    explicit Client(const std::string& name, bool enableLease = false);
    virtual ~Client();

    void Init() override;

    void WaitLeaseApplied();
    int64_t GetLeaseId();

    const std::string& GetApiVersion() const;
    std::string GetServerApiVersion();

protected:
    void SetApiVersion(const std::string& apiVersion);

    int32_t Noop();

    int32_t Call(int32_t apiId, const std::string& parameter, std::string& data);
    int32_t Call(int32_t apiId, const std::string& parameter);

    int32_t Call(int32_t apiId, const std::vector<uint8_t>& parameter, std::vector<uint8_t>& data);
    int32_t Call(int32_t apiId, const std::vector<uint8_t>& parameter);

    int32_t Call(int32_t apiId, const std::string& parameter, const std::vector<uint8_t>& binary);

    void RegistApi(int32_t apiId, int32_t priority = 0);
    int32_t CheckApi(int32_t apiId, int32_t& priority, int64_t& leaseId);

private:
    bool mEnableLease;
    std::string mApiVersion;
    std::unordered_map<int32_t,int32_t> mApiMap;
    LeaseClientPtr mLeaseClientPtr;
};

using ClientPtr = std::shared_ptr<Client>;

}
}

#endif//__LTR_ROBOT_SDK_CLIENT_HPP__

