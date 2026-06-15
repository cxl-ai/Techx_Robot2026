// Implementation file for dds_callback.hpp
// This would typically be in a .cpp file, but for header-only library,
// we can put implementation in a separate implementation file

#include <ltr/common/dds/dds_callback.hpp>

namespace ltr
{
namespace common
{

DdsReaderCallback::DdsReaderCallback() : mMessageHandler(nullptr)
{}

DdsReaderCallback::DdsReaderCallback(const DdsMessageHandler& handler) : mMessageHandler(handler)
{}

DdsReaderCallback::DdsReaderCallback(const DdsReaderCallback& cb) : mMessageHandler(cb.mMessageHandler)
{}

DdsReaderCallback& DdsReaderCallback::operator=(const DdsReaderCallback& cb)
{
    if (this != &cb)
    {
        mMessageHandler = cb.mMessageHandler;
    }
    return *this;
}

DdsReaderCallback::~DdsReaderCallback()
{}

bool DdsReaderCallback::HasMessageHandler() const
{
    return mMessageHandler != nullptr;
}

void DdsReaderCallback::OnDataAvailable(const void* message)
{
    if (mMessageHandler)
    {
        mMessageHandler(message);
    }
}

}
}

