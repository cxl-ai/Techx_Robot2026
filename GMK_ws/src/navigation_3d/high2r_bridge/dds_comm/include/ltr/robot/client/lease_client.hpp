#ifndef __LTR_ROBOT_SDK_LEASE_CLIENT_HPP__
#define __LTR_ROBOT_SDK_LEASE_CLIENT_HPP__

#include <ltr/robot/client/client_base.hpp>
#include <ltr/common/thread/recurrent_thread.hpp>
#include <ltr/common/lock/lock.hpp>
#include <string>
#include <memory>

namespace ltr
{
namespace robot
{
class LeaseContext
{
public:
    LeaseContext();
    ~LeaseContext();

    void Update(int64_t id, int64_t term);
    void Reset();

    bool Valid() const;

    int64_t GetId() const;
    int64_t GetTerm() const;

private:
    int64_t mId;
    int64_t mTerm;
};

using LeaseContextPtr = std::shared_ptr<LeaseContext>;

class LeaseClient : public ClientBase
{
public:
    explicit LeaseClient(const std::string& name);
    ~LeaseClient();

    void Init() override;

    void WaitApplied();
    int64_t GetId();
    bool Applied();

private:
    void Apply();
    void Renewal();

    void ThreadFunction();

    int64_t GetWaitMicrosec();

private:
    std::string mName;
    std::string mContextName;
    LeaseContext mContext;
    common::RecurrentThreadPtr mThreadPtr;
    common::Mutex mMutex;
};

using LeaseClientPtr = std::shared_ptr<LeaseClient>;

}
}

#endif//__LTR_ROBOT_SDK_LEASE_CLIENT_HPP__
