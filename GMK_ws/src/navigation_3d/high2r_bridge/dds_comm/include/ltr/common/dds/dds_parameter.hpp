#ifndef __LTR_DDS_PARAMETER_HPP__
#define __LTR_DDS_PARAMETER_HPP__

#include <ltr/common/decl.hpp>
#include <cstdint>
#include <string>

#define LTR_DDS_PARAM_KEY_PARTICIPANT        "Participant"
#define LTR_DDS_PARAM_KEY_DOMAINID           "DomainId"
#define LTR_DDS_PARAM_KEY_CONFIG             "Config"
#define LTR_DDS_PARAM_KEY_NAME               "Name"
#define LTR_DDS_PARAM_KEY_TOPIC              "Topic"
#define LTR_DDS_PARAM_KEY_TOPICNAME          "TopicName"
#define LTR_DDS_PARAM_KEY_PUBLISHER          "Publisher"
#define LTR_DDS_PARAM_KEY_SUBSCRIBER         "Subscriber"
#define LTR_DDS_PARAM_KEY_WRITER             "Writer"
#define LTR_DDS_PARAM_KEY_READER             "Reader"
#define LTR_DDS_PARAM_KEY_QOS                "Qos"

namespace ltr
{
namespace common
{
// Simplified parameter classes
// Full implementation would include all QoS parameter types

class DdsParticipantParameter
{
public:
    DdsParticipantParameter();
    DdsParticipantParameter(uint32_t domainId, const std::string& config = "");
    ~DdsParticipantParameter();

    void SetDomainId(int32_t domainId);
    uint32_t GetDomainId() const;

    void SetConfig(const std::string& config);
    const std::string& GetConfig() const;

private:
    uint32_t mDomainId;
    std::string mConfig;
};

class DdsTopicParameter
{
public:
    DdsTopicParameter();
    DdsTopicParameter(const std::string& name);
    ~DdsTopicParameter();

    void SetName(const std::string& name);
    const std::string& GetName() const;

private:
    std::string mName;
};

}
}

#endif//__LTR_DDS_PARAMETER_HPP__

