#ifndef __LTR_DDS_QOS_POLICY_HPP__
#define __LTR_DDS_QOS_POLICY_HPP__

#include <dds/dds.hpp>
#include <ltr/common/dds/dds_native.hpp>
#include <string>
#include <vector>

namespace ltr
{
namespace common
{
class DdsQosPolicyName
{
public:
    explicit DdsQosPolicyName(const std::string& name) :
        mName(name)
    {}

    virtual ~DdsQosPolicyName()
    {}

    const std::string& GetName() const
    {
        return mName;
    }

protected:
    std::string mName;
};

class DdsDuration : public DdsNative<::dds::core::Duration>
{
public:
    explicit DdsDuration(int64_t nanoSecond);
    ~DdsDuration();
};

// QoS Policy classes - simplified version
// Full implementation would include all DDS QoS policies
class DdsQosDurabilityPolicy : public DdsNative<::dds::core::policy::Durability>, public DdsQosPolicyName
{
public:
    explicit DdsQosDurabilityPolicy(int32_t kind);
    ~DdsQosDurabilityPolicy();
};

class DdsQosReliabilityPolicy : public DdsNative<::dds::core::policy::Reliability>, public DdsQosPolicyName
{
public:
    explicit DdsQosReliabilityPolicy(int32_t kind, int64_t maxBlockingTime);
    ~DdsQosReliabilityPolicy();
};

class DdsQosHistoryPolicy : public DdsNative<::dds::core::policy::History>, public DdsQosPolicyName
{
public:
    explicit DdsQosHistoryPolicy(int32_t kind, int32_t depth);
    ~DdsQosHistoryPolicy();
};

}
}

#endif//__LTR_DDS_QOS_POLICY_HPP__

