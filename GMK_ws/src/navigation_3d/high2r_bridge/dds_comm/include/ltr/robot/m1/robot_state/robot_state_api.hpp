#ifndef __LTR_ROBOT_M1_ROBOT_STATE_API_HPP__
#define __LTR_ROBOT_M1_ROBOT_STATE_API_HPP__

#include <ltr/common/decl.hpp>
#include <string>
#include <vector>

namespace ltr
{
namespace robot
{
namespace m1
{
/*
 * service name
 */
const std::string ROBOT_STATE_SERVICE_NAME = "robot_state";

/*
 * api version
 */
const std::string ROBOT_STATE_API_VERSION = "1.0.0.0";

/*
 * api id
 */
const int32_t ROBOT_STATE_API_ID_GET_ROBOT_STATE    = 1001;
const int32_t ROBOT_STATE_API_ID_GET_MOTOR_STATES   = 1002;
const int32_t ROBOT_STATE_API_ID_GET_IMU_STATE      = 1003;
const int32_t ROBOT_STATE_API_ID_GET_BATTERY_STATE  = 1004;
const int32_t ROBOT_STATE_API_ID_SERVICE_LIST       = 1005;
const int32_t ROBOT_STATE_API_ID_SERVICE_SWITCH     = 1006;
const int32_t ROBOT_STATE_API_ID_SET_REPORT_FREQ    = 1007;

/*
 * ServiceState
 */
class ServiceState
{
public:
    ServiceState() : status(0), protect(0)
    {}

    std::string name;
    int32_t status;
    int32_t protect;
};

}
}
}

#endif //__LTR_ROBOT_M1_ROBOT_STATE_API_HPP__

