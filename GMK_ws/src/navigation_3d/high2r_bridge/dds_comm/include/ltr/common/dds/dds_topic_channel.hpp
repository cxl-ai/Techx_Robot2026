#ifndef __LTR_DDS_TOPIC_CHANNEL_HPP__
#define __LTR_DDS_TOPIC_CHANNEL_HPP__

#include <ltr/common/dds/dds_entity.hpp>
#include <ltr/common/dds/dds_qos.hpp>
#include <ltr/common/time/sleep.hpp>

namespace ltr
{
namespace common
{
/*
 * @brief: DdsTopicChannelAbstract
 */
class DdsTopicChannelAbstract
{
public:
    virtual bool Write(const void* message, int64_t waitMicrosec) = 0;
    virtual int64_t GetLastDataAvailableTime() const = 0;
};

using DdsTopicChannelAbstractPtr = std::shared_ptr<DdsTopicChannelAbstract>;

#define LTR_DDS_WAIT_MATCHED_TIME_MICRO_SEC 100000

/*
 * @brief: DdsTopicChannel
 */
template<typename MSG>
class DdsTopicChannel : public DdsTopicChannelAbstract
{
public:
    explicit DdsTopicChannel()
    {}

    ~DdsTopicChannel()
    {}

    void SetTopic(const DdsParticipantPtr& participant, const std::string& name, const DdsTopicQos& qos)
    {
        mTopic = DdsTopicPtr<MSG>(new DdsTopic<MSG>(participant, name, qos));
    }

    void SetWriter(const DdsPublisherPtr& publisher, const DdsWriterQos& qos)
    {
        mWriter = DdsWriterPtr<MSG>(new DdsWriter<MSG>(publisher, mTopic, qos));
        MicroSleep(LTR_DDS_WAIT_MATCHED_TIME_MICRO_SEC);
    }

    void SetReader(const DdsSubscriberPtr& subscriber, const DdsReaderQos& qos, const DdsReaderCallback& cb, int32_t queuelen)
    {
        std::cout << "[DdsTopicChannel] SetReader called" << std::flush << std::endl;
        mReader = DdsReaderPtr<MSG>(new DdsReader<MSG>(subscriber, mTopic, qos));
        std::cout << "[DdsTopicChannel] DdsReader created" << std::flush << std::endl;
        mReader->SetListener(cb, queuelen);
        std::cout << "[DdsTopicChannel] SetReader completed" << std::flush << std::endl;
    }

    DdsWriterPtr<MSG> GetWriter() const
    {
        return mWriter;
    }

    DdsReaderPtr<MSG> GetReader() const
    {
        return mReader;
    }

    bool Write(const void* message, int64_t waitMicrosec)
    {
        return Write(*(const MSG*)message, waitMicrosec);
    }

    bool Write(const MSG& message, int64_t waitMicrosec)
    {
        if (mWriter)
        {
            return mWriter->Write(message, waitMicrosec);
        }
        return false;
    }

    int64_t GetLastDataAvailableTime() const
    {
        if (mReader)
        {
            return mReader->GetLastDataAvailableTime();
        }

        return 0;
    }

private:
    DdsTopicPtr<MSG> mTopic;
    DdsWriterPtr<MSG> mWriter;
    DdsReaderPtr<MSG> mReader;
};

template<typename MSG>
using DdsTopicChannelPtr = std::shared_ptr<DdsTopicChannel<MSG>>;

}
}

#endif//__LTR_DDS_TOPIC_CHANNEL_HPP__

