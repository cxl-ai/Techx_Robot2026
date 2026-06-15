#ifndef __LTR_ROBOT_INTERNAL_REQUEST_RESPONSE_HPP__
#define __LTR_ROBOT_INTERNAL_REQUEST_RESPONSE_HPP__

#include <ltr/robot/internal/internal_idl_decl/Request_.hpp>
#include <ltr/robot/internal/internal_idl_decl/Response_.hpp>

namespace ltr
{
namespace robot
{
using RequestIdentity = ltr_api::msg::dds_::RequestIdentity_;
using RequestLease = ltr_api::msg::dds_::RequestLease_;
using RequestPolicy = ltr_api::msg::dds_::RequestPolicy_;
using RequestHeader = ltr_api::msg::dds_::RequestHeader_;
using Request = ltr_api::msg::dds_::Request_;

using ResponseStatus = ltr_api::msg::dds_::ResponseStatus_;
using ResponseHeader = ltr_api::msg::dds_::ResponseHeader_;
using Response = ltr_api::msg::dds_::Response_;
}
}

#endif//__LTR_ROBOT_INTERNAL_REQUEST_RESPONSE_HPP__

