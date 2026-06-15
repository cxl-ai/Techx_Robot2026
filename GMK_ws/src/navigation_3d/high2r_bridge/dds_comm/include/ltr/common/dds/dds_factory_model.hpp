#ifndef __LTR_DDS_FACTORY_MODEL_HPP__
#define __LTR_DDS_FACTORY_MODEL_HPP__

#include <ltr/common/dds/dds_parameter.hpp>
#include <ltr/common/dds/dds_topic_channel.hpp>
#include <ltr/common/dds/dds_entity.hpp>
#include <ltr/common/dds/dds_qos.hpp>
#include <ltr/common/dds/dds_qos_realize.hpp>
#include <ltr/common/dds/dds_callback.hpp>
#include <ltr/common/json/json.hpp>
#include <memory>

namespace ltr
{
namespace common
{
// Forward declaration
class Logger;

class DdsFactoryModel
{
public:
    explicit DdsFactoryModel();
    ~DdsFactoryModel();

    void Init(uint32_t domainId, const std::string& ddsConfig = "");
    void Init(const std::string& ddsParameterFileName = "");
    void Init(const JsonMap& param);

    template<typename MSG>
    DdsTopicChannelPtr<MSG> CreateTopicChannel(const std::string& topic)
    {
        DdsTopicChannelPtr<MSG> channel = DdsTopicChannelPtr<MSG>(new DdsTopicChannel<MSG>());
        channel->SetTopic(mParticipant, topic, mTopicQos);
        return channel;
    }

    template<typename MSG>
    void SetWriter(DdsTopicChannelPtr<MSG>& channelPtr)
    {
        channelPtr->SetWriter(mPublisher, mWriterQos);
    }

    template<typename MSG>
    void SetReader(DdsTopicChannelPtr<MSG>& channelPtr, const std::function<void(const void*)>& handler, int32_t queuelen = 0)
    {
        std::cout << "[DdsFactoryModel] SetReader called, handler valid=" << (handler != nullptr) << ", queuelen=" << queuelen << std::flush << std::endl;
        DdsReaderCallback cb(handler);
        std::cout << "[DdsFactoryModel] DdsReaderCallback created, HasMessageHandler=" << cb.HasMessageHandler() << std::flush << std::endl;
        channelPtr->SetReader(mSubscriber, mReaderQos, cb, queuelen);
        std::cout << "[DdsFactoryModel] SetReader completed" << std::flush << std::endl;
    }

private:
    void InitQos();
    void CreateParticipant(uint32_t domainId, const std::string& config);
    void CreatePublisher();
    void CreateSubscriber();

private:
    DdsParticipantPtr mParticipant;
    DdsPublisherPtr mPublisher;
    DdsSubscriberPtr mSubscriber;

    DdsParticipantQos mParticipantQos;
    DdsTopicQos mTopicQos;
    DdsPublisherQos mPublisherQos;
    DdsSubscriberQos mSubscriberQos;
    DdsWriterQos mWriterQos;
    DdsReaderQos mReaderQos;

    // Logger* mLogger;  // TODO: Implement logger
};

using DdsFactoryModelPtr = std::shared_ptr<DdsFactoryModel>;

}
}

#endif//__LTR_DDS_FACTORY_MODEL_HPP__
