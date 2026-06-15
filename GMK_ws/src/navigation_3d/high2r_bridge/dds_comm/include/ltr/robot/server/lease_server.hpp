#ifndef __LTR_ROBOT_LEASE_SERVER_HPP__
#define __LTR_ROBOT_LEASE_SERVER_HPP__

#include <ltr/robot/server/server_base.hpp>
#include <ltr/common/lock/lock.hpp>
#include <string>

namespace ltr
{
namespace robot
{
class LeaseCache
{
public:
    LeaseCache();
    ~LeaseCache();

    void Set(int64_t id, const std::string& name, int64_t lastModified = 0);
    void Renewal(int64_t lastModified = 0);
    void Clear();

    int64_t GetLastModified() const;
    int64_t GetId() const;
    const std::string& GetName() const;

private:
    int64_t mLastModified;
    int64_t mId;
    std::string mName;
};

class LeaseServer : public ServerBase
{
public:
    explicit LeaseServer(const std::string& name, int64_t term);
    ~LeaseServer();

    void Init() override;

    bool CheckRequestLeaseDenied(int64_t leaseId);
    
    // Public methods for handling lease requests (can be called from Server handlers)
    int32_t HandleApply(const std::string& parameter, std::string& data);
    int32_t HandleRenewal(int64_t leaseId);
    
private:
    void ServerRequestHandler(const RequestPtr& request) override;

    int32_t Apply(const std::string& parameter, std::string& data);
    int32_t Renewal(int64_t leaseId);

    int64_t GenerateId(const std::string& name);

private:
    int64_t mTerm;
    LeaseCache mCache;
    common::Mutex mMutex;
    ServerStubPtr mServerStubPtr;
};

using LeaseServerPtr = std::shared_ptr<LeaseServer>;

}
}

#endif//__LTR_ROBOT_LEASE_SERVER_HPP__
