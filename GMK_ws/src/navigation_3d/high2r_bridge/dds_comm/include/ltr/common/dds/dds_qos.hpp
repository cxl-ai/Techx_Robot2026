#ifndef __LTR_DDS_QOS_HPP__
#define __LTR_DDS_QOS_HPP__

#include <ltr/common/dds/dds_qos_policy.hpp>
#include <dds/dds.hpp>
#include <set>
#include <string>

using namespace org::eclipse::cyclonedds;

namespace ltr
{
namespace common
{
#define LTR_DECL_DDS_QOS(QosType, QosNative)         \
class QosType: public QosNative                     \
{                                                   \
public:                                             \
    using NativeQosType = QosNative::NativeType;    \
    explicit QosType()                              \
    {                                               \
        InitPolicyDefault();                        \
    }                                               \
    template<typename POLICY>                       \
    void SetPolicy(const POLICY& policy)            \
    {                                               \
        mNative.policy(policy.GetNative());         \
        mPolicyNameSet.insert(policy.GetName());    \
    }                                               \
    bool HasPolicy(const std::string& name) const   \
    {                                               \
        return mPolicyNameSet.find(name) != mPolicyNameSet.end();   \
    }                                               \
    void CopyToNativeQos(NativeQosType& qos) const \
    {                                               \
        qos = mNative;                              \
    }                                               \
private:                                            \
    void InitPolicyDefault()                        \
    {                                               \
        /* Default constructor already initializes with defaults */ \
    }                                               \
private:                                            \
    std::set<std::string> mPolicyNameSet;           \
};

/*
 * DdsParticipantQos
 */
LTR_DECL_DDS_QOS(DdsParticipantQos, DdsNative<::dds::domain::qos::DomainParticipantQos>)

/*
 * DdsTopicQos
 */
LTR_DECL_DDS_QOS(DdsTopicQos, DdsNative<::dds::topic::qos::TopicQos>)

/*
 * DdsPublisherQos
 */
LTR_DECL_DDS_QOS(DdsPublisherQos, DdsNative<::dds::pub::qos::PublisherQos>)

/*
 * DdsSubscriberQos
 */
LTR_DECL_DDS_QOS(DdsSubscriberQos, DdsNative<::dds::sub::qos::SubscriberQos>)

/*
 * DdsWriterQos
 */
LTR_DECL_DDS_QOS(DdsWriterQos, DdsNative<::dds::pub::qos::DataWriterQos>)

/*
 * DdsReaderQos
 */
LTR_DECL_DDS_QOS(DdsReaderQos, DdsNative<::dds::sub::qos::DataReaderQos>)

}
}

#endif//__LTR_DDS_QOS_HPP__

