#ifndef __LTR_DDS_ENTITY_HPP__
#define __LTR_DDS_ENTITY_HPP__

#include <dds/dds.hpp>
#include <ltr/common/dds/dds_qos.hpp>
#include <ltr/common/dds/dds_exception.hpp>
#include <ltr/common/dds/dds_callback.hpp>
#include <ltr/common/dds/dds_traits.hpp>
#include <ltr/common/time/sleep.hpp>
#include <ltr/common/time/time_tool.hpp>
#include <ltr/robot/internal/internal_idl_decl/Request_.hpp>
#include <ltr/robot/internal/internal_idl_decl/Response_.hpp>
#include <org/eclipse/cyclonedds/core/cdr/cdr_stream.hpp>
#include <memory>
#include <functional>
#include <type_traits>
#include <iostream>

#define __LTR_DDS_NULL__ ::dds::core::null

/*
 * dds wait sub/pub matched default time slice.
 * default 10000 us
 */
#define __LTR_DDS_WAIT_MATCHED_TIME_SLICE 10000
#define __LTR_DDS_WAIT_MATCHED_TIME_MAX   1000000

using namespace org::eclipse::cyclonedds;

namespace ltr
{
namespace common
{

/*
 * @brief: DdsParticipant
 */
class DdsParticipant
{
public:
    using NATIVE_TYPE = ::dds::domain::DomainParticipant;

    explicit DdsParticipant(uint32_t domainId, const DdsParticipantQos& qos, const std::string& config = "");
    ~DdsParticipant();

    const NATIVE_TYPE& GetNative() const;

private:
    NATIVE_TYPE mNative;
};

using DdsParticipantPtr = std::shared_ptr<DdsParticipant>;


/*
 * @brief: DdsPublisher
 */
class DdsPublisher
{
public:
    using NATIVE_TYPE = ::dds::pub::Publisher;

    explicit DdsPublisher(const DdsParticipantPtr& participant, const DdsPublisherQos& qos);
    ~DdsPublisher();

    const NATIVE_TYPE& GetNative() const;

private:
    NATIVE_TYPE mNative;
};

using DdsPublisherPtr = std::shared_ptr<DdsPublisher>;


/*
 * @brief: DdsSubscriber
 */
class DdsSubscriber
{
public:
    using NATIVE_TYPE = ::dds::sub::Subscriber;

    explicit DdsSubscriber(const DdsParticipantPtr& participant, const DdsSubscriberQos& qos);
    ~DdsSubscriber();

    const NATIVE_TYPE& GetNative() const;

private:
    NATIVE_TYPE mNative;
};

using DdsSubscriberPtr = std::shared_ptr<DdsSubscriber>;


/*
 * @brief: DdsTopic
 */
template<typename MSG>
class DdsTopic
{
public:
    using NATIVE_TYPE = ::dds::topic::Topic<MSG>;

    explicit DdsTopic(const DdsParticipantPtr& participant, const std::string& name, const DdsTopicQos& qos) :
        mNative(__LTR_DDS_NULL__)
    {
        LTR_DDS_EXCEPTION_TRY

        auto topicQos = participant->GetNative().default_topic_qos();
        qos.CopyToNativeQos(topicQos);

        mNative = NATIVE_TYPE(participant->GetNative(), name, topicQos);

        LTR_DDS_EXCEPTION_CATCH(nullptr, true)
    }

    ~DdsTopic()
    {
        mNative = __LTR_DDS_NULL__;
    }

    const NATIVE_TYPE& GetNative() const
    {
        return mNative;
    }

private:
    NATIVE_TYPE mNative;
};

template<typename MSG>
using DdsTopicPtr = std::shared_ptr<DdsTopic<MSG>>;


/*
 * @brief: DdsWriter
 */
template<typename MSG>
class DdsWriter
{
public:
    using NATIVE_TYPE = ::dds::pub::DataWriter<MSG>;

    explicit DdsWriter(const DdsPublisherPtr publisher, const DdsTopicPtr<MSG>& topic, const DdsWriterQos& qos) :
        mNative(__LTR_DDS_NULL__)
    {
        LTR_DDS_EXCEPTION_TRY

        auto writerQos = publisher->GetNative().default_datawriter_qos();
        qos.CopyToNativeQos(writerQos);

        mNative = NATIVE_TYPE(publisher->GetNative(), topic->GetNative(), writerQos);

        LTR_DDS_EXCEPTION_CATCH(nullptr, true)
    }

    ~DdsWriter()
    {
        mNative = __LTR_DDS_NULL__;
    }

    const NATIVE_TYPE& GetNative() const
    {
        return mNative;
    }

    bool Write(const MSG& message, int64_t waitMicrosec)
    {
        if (waitMicrosec > 0)
        {
            WaitReader(waitMicrosec);
        }

        LTR_DDS_EXCEPTION_TRY
        {
            mNative.write(message);
            return true;
        }
        LTR_DDS_EXCEPTION_CATCH(nullptr, false)

        return false;
    }

private:
    void WaitReader(int64_t waitMicrosec)
    {
        if (waitMicrosec < __LTR_DDS_WAIT_MATCHED_TIME_SLICE)
        {
            return;
        }

        int64_t waitTime = (waitMicrosec / 2);
        if (waitTime > __LTR_DDS_WAIT_MATCHED_TIME_MAX)
        {
            waitTime = __LTR_DDS_WAIT_MATCHED_TIME_MAX;
        }

        while (waitTime > 0 && mNative.publication_matched_status().current_count() == 0)
        {
            MicroSleep(__LTR_DDS_WAIT_MATCHED_TIME_SLICE);
            waitTime -= __LTR_DDS_WAIT_MATCHED_TIME_SLICE;
        }
    }

private:
    NATIVE_TYPE mNative;
};

template<typename MSG>
using DdsWriterPtr = std::shared_ptr<DdsWriter<MSG>>;


/*
 * @brief: DdsReaderListener
 */
template<typename MSG>
class DdsReaderListener : public ::dds::sub::NoOpDataReaderListener<MSG>
{
public:
    using NATIVE_TYPE = ::dds::sub::DataReaderListener<MSG>;
    using MSG_PTR = std::shared_ptr<MSG>;

    explicit DdsReaderListener() :
        mHasQueue(false), mQuit(false), mMask(::dds::core::status::StatusMask::none()), mLastDataAvailableTime(0)
    {
        // Force initialization of get_type_props to ensure type properties are registered
        // This is critical for correct DDS deserialization
        std::cout << "[DdsReaderListener] Constructor called" << std::flush << std::endl;
        using namespace org::eclipse::cyclonedds::core::cdr;
        auto &props = get_type_props<MSG>();
        std::cout << "[DdsReaderListener] Initialized get_type_props for type, props.size()=" 
                  << props.size() << std::flush << std::endl;
        (void)props;  // Suppress unused variable warning
    }

    ~DdsReaderListener()
    {
        if (mHasQueue)
        {
            mQuit = true;
            // TODO: Implement queue interruption if needed
        }
    }

    void SetCallback(const DdsReaderCallback& cb)
    {
        std::cout << "[DdsReaderListener] SetCallback called" << std::endl;
        if (cb.HasMessageHandler())
        {
            std::cout << "[DdsReaderListener] Message handler is valid, setting data_available mask" << std::endl;
            mMask |= ::dds::core::status::StatusMask::data_available();
        }
        else
        {
            std::cout << "[DdsReaderListener] WARNING: Message handler is null!" << std::endl;
        }

        mCallbackPtr.reset(new DdsReaderCallback(cb));
        std::cout << "[DdsReaderListener] Callback pointer set, HasMessageHandler=" << mCallbackPtr->HasMessageHandler() << std::endl;
    }

    void SetQueue(int32_t len)
    {
        std::cout << "[DdsReaderListener] SetQueue called, len=" << len << std::flush << std::endl;
        if (len <= 0)
        {
            std::cout << "[DdsReaderListener] Queue disabled (len <= 0), using direct callback" << std::flush << std::endl;
            mHasQueue = false;
            return;
        }

        std::cout << "[DdsReaderListener] WARNING: Queue enabled but not implemented! Callbacks will not be called!" << std::flush << std::endl;
        mHasQueue = true;
        // TODO: Implement BlockQueue if needed
    }

    int64_t GetLastDataAvailableTime() const
    {
        return mLastDataAvailableTime;
    }

    NATIVE_TYPE* GetNative() const
    {
        return (NATIVE_TYPE*)this;
    }

    const ::dds::core::status::StatusMask& GetStatusMask() const
    {
        return mMask;
    }

private:
    void on_data_available(::dds::sub::DataReader<MSG>& reader)
    {
        ::dds::sub::LoanedSamples<MSG> samples;
        samples = reader.take();

        std::cout << "[DdsReaderListener] on_data_available called, samples.length()=" << samples.length() << std::endl;

        if (samples.length() <= 0)
        {
            std::cout << "[DdsReaderListener] No samples available" << std::endl;
            return;
        }

        typename ::dds::sub::LoanedSamples<MSG>::const_iterator iter;
        for (iter=samples.begin(); iter!=samples.end(); ++iter)
        {
            const MSG& m = iter->data();
            bool isValid = iter->info().valid();
            std::cout << "[DdsReaderListener] Sample valid=" << isValid << std::endl;
            
            if (isValid)
            {
                mLastDataAvailableTime = GetCurrentMonotonicTimeNanosecond();
                std::cout << "[DdsReaderListener] Valid sample received, calling callback" << std::endl;

                if (mHasQueue)
                {
                    // TODO: Implement queue handling if needed
                }
                else
                {
                    if (mCallbackPtr)
                    {
                        std::cout << "[DdsReaderListener] Calling OnDataAvailable callback" << std::endl;
                        mCallbackPtr->OnDataAvailable((const void*)&m);
                    }
                    else
                    {
                        std::cout << "[DdsReaderListener] ERROR: Callback pointer is null!" << std::endl;
                    }
                }
            }
            else
            {
                std::cout << "[DdsReaderListener] Sample is not valid, skipping" << std::endl;
            }
        }
    }

private:
    bool mHasQueue;
    volatile bool mQuit;

    ::dds::core::status::StatusMask mMask;
    int64_t mLastDataAvailableTime;

    DdsReaderCallbackPtr mCallbackPtr;
};

template<typename MSG>
using DdsReaderListenerPtr = std::shared_ptr<DdsReaderListener<MSG>>;


/*
 * @brief: DdsReader
 */
template<typename MSG>
class DdsReader
{
public:
    using NATIVE_TYPE = ::dds::sub::DataReader<MSG>;

    explicit DdsReader(const DdsSubscriberPtr& subscriber, const DdsTopicPtr<MSG>& topic, const DdsReaderQos& qos) :
        mNative(__LTR_DDS_NULL__)
    {
        std::cout << "[DdsReader] Constructor called" << std::flush << std::endl;
        std::cout << "[DdsReader] Listener will be created (member variable)" << std::flush << std::endl;
        LTR_DDS_EXCEPTION_TRY

        auto readerQos = subscriber->GetNative().default_datareader_qos();
        qos.CopyToNativeQos(readerQos);

        std::cout << "[DdsReader] Creating DataReader..." << std::flush << std::endl;
        mNative = NATIVE_TYPE(subscriber->GetNative(), topic->GetNative(), readerQos);
        std::cout << "[DdsReader] DataReader created successfully" << std::flush << std::endl;

        LTR_DDS_EXCEPTION_CATCH(nullptr, true)
    }

    ~DdsReader()
    {
        mNative = __LTR_DDS_NULL__;
    }

    const NATIVE_TYPE& GetNative() const
    {
        return mNative;
    }

    void SetListener(const DdsReaderCallback& cb, int32_t qlen)
    {
        std::cout << "[DdsReader] SetListener called, qlen=" << qlen << std::flush << std::endl;
        mListener.SetCallback(cb);
        mListener.SetQueue(qlen);
        std::cout << "[DdsReader] Setting listener to DataReader, status mask=" << std::flush << std::endl;
        mNative.listener(mListener.GetNative(), mListener.GetStatusMask());
        std::cout << "[DdsReader] Listener set successfully" << std::flush << std::endl;
    }

    int64_t GetLastDataAvailableTime() const
    {
        return mListener.GetLastDataAvailableTime();
    }

private:
    NATIVE_TYPE mNative;
    DdsReaderListener<MSG> mListener;
};

template<typename MSG>
using DdsReaderPtr = std::shared_ptr<DdsReader<MSG>>;

}
}

#endif//__LTR_DDS_ENTITY_HPP__
