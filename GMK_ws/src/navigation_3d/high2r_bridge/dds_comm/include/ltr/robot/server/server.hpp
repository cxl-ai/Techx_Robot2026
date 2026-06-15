#ifndef __LTR_ROBOT_SDK_SERVER_HPP__
#define __LTR_ROBOT_SDK_SERVER_HPP__

#include <ltr/robot/server/server_base.hpp>
#include <ltr/robot/server/lease_server.hpp>
#include <functional>
#include <unordered_map>
#include <set>
#include <string>
#include <vector>
#include <cstdint>

#define LTR_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(apiId, handler)            \
    LTR_ROBOT_SERVER_REG_API_HANDLER(apiId, handler, false)

#define LTR_ROBOT_SERVER_REG_API_BINARY_HANDLER_NO_LEASE(apiId, handler)     \
    LTR_ROBOT_SERVER_REG_API_BINARY_HANDLER(apiId, handler, false)

#define LTR_ROBOT_SERVER_REG_API_HANDLER(apiId, handler, checkLease)         \
    RegistHandler(apiId, std::bind(handler, this, std::placeholders::_1, std::placeholders::_2), checkLease)

#define LTR_ROBOT_SERVER_REG_API_BINARY_HANDLER(apiId, handler, checkLease)  \
    RegistBinaryHandler(apiId, std::bind(handler, this, std::placeholders::_1, std::placeholders::_2), checkLease)

namespace ltr
{
namespace robot
{
using RequestHandler = std::function<int32_t(const std::string& parameter, std::string& data)>;
using BinaryRequestHandler = std::function<int32_t(const std::vector<uint8_t>& parameter, std::vector<uint8_t>& data)>;

class Server : public ServerBase
{
public:
    explicit Server(const std::string& name);
    virtual ~Server();

    virtual void Init() = 0;

    void StartLease(int64_t leaseTerm);
    void StartLease(float leaseTerm);

    const std::string& GetName();
    const std::string& GetApiVersion() const;

    int32_t GetCurrentApiId() const;

protected:
    void SetApiVersion(const std::string& version);

    void ServerRequestHandler(const RequestPtr& request) override;

    void RegistHandler(int32_t apiId, const RequestHandler& handler, bool checkLease = false);
    void RegistBinaryHandler(int32_t apiId, const BinaryRequestHandler& binaryHandler, bool checkLease = false);

    bool IsBinary(int32_t apiId);

    RequestHandler GetHandler(int32_t apiId, bool& ignoreLease) const;
    BinaryRequestHandler GetBinaryHandler(int32_t apiId, bool& ignoreLease) const;

    bool CheckLeaseDenied(int64_t leaseId);

private:
    bool mEnableLease;
    std::string mApiVersion;
    std::unordered_map<int32_t,std::pair<RequestHandler,bool>> mApiHandlerMap;
    std::unordered_map<int32_t,std::pair<BinaryRequestHandler,bool>> mApiBinaryHandlerMap;

    std::set<int32_t> mApiBinarySet;

    LeaseServerPtr mLeaseServerPtr;

private:
    static thread_local int32_t mCurrentApiId;
};

using ServerPtr = std::shared_ptr<Server>;

}
}

#endif//__LTR_ROBOT_SDK_SERVER_HPP__

