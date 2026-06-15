#include <ltr/robot/channel/channel_factory.hpp>
#include <ltr/common/dds/dds_parameter.hpp>
#include <ltr/common/dds/dds_qos_realize.hpp>
#include <stdexcept>

namespace ltr
{
namespace robot
{

ChannelFactory::ChannelFactory() : mInited(false), mDdsFactoryPtr(nullptr)
{}

ChannelFactory::~ChannelFactory()
{
    Release();
}

void ChannelFactory::Init(int32_t domainId, const std::string& networkInterface)
{
    common::LockGuard<common::Mutex> lock(mMutex);
    
    if (mInited)
    {
        Release();
    }

    mDdsFactoryPtr = std::make_shared<common::DdsFactoryModel>();
    
    // Build DDS config string if network interface is specified
    std::string ddsConfig;
    if (!networkInterface.empty())
    {
        // CycloneDDS configuration for network interface
        ddsConfig = "<CycloneDDS><Domain><Id>" + std::to_string(domainId) + 
                    "</Id><General><NetworkInterfaceAddress>" + networkInterface + 
                    "</NetworkInterfaceAddress></General></Domain></CycloneDDS>";
    }
    
    mDdsFactoryPtr->Init(static_cast<uint32_t>(domainId), ddsConfig);
    mInited = true;
}

void ChannelFactory::Init(const std::string& configFileName)
{
    common::LockGuard<common::Mutex> lock(mMutex);
    
    if (mInited)
    {
        Release();
    }

    mDdsFactoryPtr = std::make_shared<common::DdsFactoryModel>();
    mDdsFactoryPtr->Init(configFileName);
    mInited = true;
}

void ChannelFactory::Init(const common::JsonMap& jsonMap)
{
    common::LockGuard<common::Mutex> lock(mMutex);
    
    if (mInited)
    {
        Release();
    }

    mDdsFactoryPtr = std::make_shared<common::DdsFactoryModel>();
    mDdsFactoryPtr->Init(jsonMap);
    mInited = true;
}

void ChannelFactory::Release()
{
    common::LockGuard<common::Mutex> lock(mMutex);
    
    if (mDdsFactoryPtr)
    {
        mDdsFactoryPtr.reset();
    }
    
    mInited = false;
}

}
}

