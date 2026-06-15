#ifndef __LTR_ROBOT_SDK_CHANNEL_LABOR_HPP__
#define __LTR_ROBOT_SDK_CHANNEL_LABOR_HPP__

#include <ltr/robot/channel/channel_factory.hpp>
#include <ltr/robot/channel/channel_namer.hpp>
#include <ltr/common/time/time_tool.hpp>
#include <functional>
#include <memory>

namespace ltr
{
namespace robot
{
/*
 * @brief ChannelLabor - Template class for bidirectional channel communication
 */
template<typename SEND_MSG, typename RECV_MSG>
class ChannelLabor
{
public:
    ChannelLabor() = default;
    virtual ~ChannelLabor() = default;

    void InitChannel(const std::string& name, const std::function<void(const void*)>& recvMesageCallback, int32_t queuelen = 0)
    {
        std::string sendChannelName = mNamerPtr->GetSendChannelName(name);
        std::string recvChannelName = mNamerPtr->GetRecvChannelName(name);

        mSendChannlPtr = ChannelFactory::Instance()->CreateSendChannel<SEND_MSG>(sendChannelName);
        mRecvChannlPtr = ChannelFactory::Instance()->CreateRecvChannel<RECV_MSG>(recvChannelName, recvMesageCallback, queuelen);
    }

    bool Send(const SEND_MSG& msg, int64_t waitTimeout)
    {
        if (mSendChannlPtr)
        {
            return mSendChannlPtr->Write(msg, waitTimeout);
        }
        return false;
    }

    int64_t GetLastDataAvailableTime() const
    {
        if (mRecvChannlPtr)
        {
            return mRecvChannlPtr->GetLastDataAvailableTime();
        }
        return -1;
    }

protected:
    ChannelNamerPtr mNamerPtr;

private:
    ChannelPtr<SEND_MSG> mSendChannlPtr;
    ChannelPtr<RECV_MSG> mRecvChannlPtr;
};

template<typename SEND_MSG, typename RECV_MSG>
using ChannelLaborPtr = std::shared_ptr<ChannelLabor<SEND_MSG,RECV_MSG>>;

/*
 * @brief ClientChannelLabor - Channel labor for client side
 */
template<typename SEND_MSG, typename RECV_MSG>
class ClientChannelLabor : public ChannelLabor<SEND_MSG,RECV_MSG>
{
public:
    ClientChannelLabor()
    {
        ChannelLabor<SEND_MSG,RECV_MSG>::mNamerPtr = ChannelNamerPtr(new ClientChannelNamer());
    }

    ~ClientChannelLabor() = default;
};

template<typename SEND_MSG, typename RECV_MSG>
using ClientChannelLaborPtr = std::shared_ptr<ClientChannelLabor<SEND_MSG,RECV_MSG>>;

/*
 * @brief ServerChannelLabor - Channel labor for server side
 */
template<typename SEND_MSG, typename RECV_MSG>
class ServerChannelLabor : public ChannelLabor<SEND_MSG,RECV_MSG>
{
public:
    ServerChannelLabor()
    {
        ChannelLabor<SEND_MSG,RECV_MSG>::mNamerPtr = ChannelNamerPtr(new ServerChannelNamer());
    }

    ~ServerChannelLabor() = default;
};

template<typename SEND_MSG, typename RECV_MSG>
using ServerChannelLaborPtr = std::shared_ptr<ServerChannelLabor<SEND_MSG,RECV_MSG>>;

}
}

#endif//__LTR_ROBOT_SDK_CHANNEL_LABOR_HPP__

