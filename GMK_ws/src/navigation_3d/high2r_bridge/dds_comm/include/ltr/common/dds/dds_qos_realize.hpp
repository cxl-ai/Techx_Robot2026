#ifndef __LTR_DDS_QOS_REALIZE_HPP__
#define __LTR_DDS_QOS_REALIZE_HPP__

#include <ltr/common/dds/dds_qos.hpp>
#include <ltr/common/dds/dds_parameter.hpp>

namespace ltr
{
namespace common
{
// QoS realization functions
// These functions convert parameter configurations to QoS objects
// Simplified version - full implementation would handle all QoS policies

void Realize(const DdsParticipantParameter& parameter, DdsParticipantQos& qos);
void Realize(const DdsTopicParameter& parameter, DdsTopicQos& qos);

// Default QoS initialization
void InitDefaultQos(DdsParticipantQos& qos);
void InitDefaultQos(DdsTopicQos& qos);
void InitDefaultQos(DdsPublisherQos& qos);
void InitDefaultQos(DdsSubscriberQos& qos);
void InitDefaultQos(DdsWriterQos& qos);
void InitDefaultQos(DdsReaderQos& qos);

}
}

#endif//__LTR_DDS_QOS_REALIZE_HPP__

