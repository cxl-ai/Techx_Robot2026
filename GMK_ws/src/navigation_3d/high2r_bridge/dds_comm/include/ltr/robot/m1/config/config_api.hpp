#ifndef __LTR_ROBOT_M1_CONFIG_API_HPP__
#define __LTR_ROBOT_M1_CONFIG_API_HPP__

#include <ltr/common/decl.hpp>
#include <string>

namespace ltr
{
namespace robot
{
namespace m1
{
/*
 * service name
 */
const std::string CONFIG_SERVICE_NAME = "config";

/*
 * api version
 */
const std::string CONFIG_API_VERSION = "1.0.0.0";

/*
 * api id
 */
const int32_t CONFIG_API_ID_SET = 1001;
const int32_t CONFIG_API_ID_GET = 1002;
const int32_t CONFIG_API_ID_DEL = 1003;
const int32_t CONFIG_API_ID_GET_ALL = 1004;

}
}
}

#endif //__LTR_ROBOT_M1_CONFIG_API_HPP__

