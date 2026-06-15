#ifndef __LTR_DDS_EXCEPTION_HPP__
#define __LTR_DDS_EXCEPTION_HPP__

#include <ltr/common/exception.hpp>
#include <ltr/common/log/log.hpp>
#include <dds/dds.hpp>

#define LTR_DDS_EXCEPTION_TRY \
    try {

#define LTR_DDS_EXCEPTION_CATCH(logger, rethrow) \
    } \
    catch (const ::dds::core::Exception& e) { \
        if ((logger) != nullptr) { \
            LOG_ERROR(logger, "DDS Exception: ", e.what()); \
        } \
        if (rethrow) { \
            throw ltr::common::DdsException(e.what()); \
        } \
    } \
    catch (const std::exception& e) { \
        if ((logger) != nullptr) { \
            LOG_ERROR(logger, "Exception: ", e.what()); \
        } \
        if (rethrow) { \
            throw ltr::common::CommonException(e.what()); \
        } \
    }

namespace ltr
{
namespace common
{

class DdsException : public CommonException
{
public:
    explicit DdsException(const std::string& message)
        : CommonException(message) {}
};

}
}

#endif//__LTR_DDS_EXCEPTION_HPP__

