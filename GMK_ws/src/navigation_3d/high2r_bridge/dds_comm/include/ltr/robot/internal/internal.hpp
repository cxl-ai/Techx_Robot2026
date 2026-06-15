#ifndef __LTR_ROBOT_SDK_INTERNAL_HPP__
#define __LTR_ROBOT_SDK_INTERNAL_HPP__

#include <ltr/robot/internal/internal_request_response.hpp>
#include <memory>

namespace ltr
{
namespace robot
{
using RequestPtr = std::shared_ptr<Request>;
using ResponsePtr = std::shared_ptr<Response>;
}
}

#endif//__LTR_ROBOT_SDK_INTERNAL_HPP__

