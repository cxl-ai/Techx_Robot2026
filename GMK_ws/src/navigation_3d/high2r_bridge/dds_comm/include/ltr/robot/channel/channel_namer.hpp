#ifndef __LTR_ROBOT_SDK_CHANNEL_NAMER_HPP__
#define __LTR_ROBOT_SDK_CHANNEL_NAMER_HPP__

#include <ltr/common/decl.hpp>
#include <string>
#include <memory>

namespace ltr
{
namespace robot
{
const std::string ROBOT_SDK_CHANNEL_PREFIX = "rt/api/";
const std::string ROBOT_SDK_CHANNEL_SUFFIX_CLIENT = "/request";
const std::string ROBOT_SDK_CHANNEL_SUFFIX_SERVER = "/response";

/*
 * @brief ChannelNamer - Base class for channel naming
 */
class ChannelNamer
{
public:
    virtual ~ChannelNamer() = default;
    virtual std::string GetSendChannelName(const std::string& name) = 0;
    virtual std::string GetRecvChannelName(const std::string& name) = 0;
};

using ChannelNamerPtr = std::shared_ptr<ChannelNamer>;

/*
 * @brief ClientChannelNamer - Channel namer for client side
 */
class ClientChannelNamer : public ChannelNamer
{
public:
    ClientChannelNamer() = default;
    ~ClientChannelNamer() = default;

    std::string GetSendChannelName(const std::string& name) override;
    std::string GetRecvChannelName(const std::string& name) override;
};

using ClientChannelNamerPtr = std::shared_ptr<ClientChannelNamer>;

/*
 * @brief ServerChannelNamer - Channel namer for server side
 */
class ServerChannelNamer : public ChannelNamer
{
public:
    ServerChannelNamer() = default;
    virtual ~ServerChannelNamer() = default;

    std::string GetSendChannelName(const std::string& name) override;
    std::string GetRecvChannelName(const std::string& name) override;
};

using ServerChannelNamerPtr = std::shared_ptr<ServerChannelNamer>;

}
}

#endif//__LTR_ROBOT_SDK_CHANNEL_NAMER_HPP__

