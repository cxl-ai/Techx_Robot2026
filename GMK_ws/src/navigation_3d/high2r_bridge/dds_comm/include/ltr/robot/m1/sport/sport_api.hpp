#ifndef __LTR_ROBOT_M1_SPORT_API_HPP__
#define __LTR_ROBOT_M1_SPORT_API_HPP__

#include <ltr/common/decl.hpp>
#include <string>

namespace ltr
{
namespace robot
{
namespace m1
{
/*service name*/
const std::string ROBOT_SPORT_SERVICE_NAME = "sport";

/*api version*/
const std::string ROBOT_SPORT_API_VERSION = "1.0.0.0";

/*api id*/
const int32_t ROBOT_SPORT_API_ID_DAMP               = 1001;
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND       = 1002;
const int32_t ROBOT_SPORT_API_ID_STOPMOVE           = 1003;
const int32_t ROBOT_SPORT_API_ID_STANDUP            = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN          = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND      = 1006;
const int32_t ROBOT_SPORT_API_ID_EULER              = 1007;
const int32_t ROBOT_SPORT_API_ID_MOVE               = 1008;
const int32_t ROBOT_SPORT_API_ID_SWITCHGAIT         = 1009;
const int32_t ROBOT_SPORT_API_ID_BODYHEIGHT         = 1010;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL         = 1015;
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW   = 1016;
const int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT     = 1017;
const int32_t ROBOT_SPORT_API_ID_MOVETOPOS          = 1018;
const int32_t ROBOT_SPORT_API_ID_SWITCHMOVEMODE     = 1019;
const int32_t ROBOT_SPORT_API_ID_VISIONWALK         = 1020;
const int32_t ROBOT_SPORT_API_ID_HANDSTAND          = 1021;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_SET   = 1022;
const int32_t ROBOT_SPORT_API_ID_FREEWALK           = 1023;
const int32_t ROBOT_SPORT_API_ID_CLASSICWALK        = 1024;
const int32_t ROBOT_SPORT_API_ID_FASTWALK            = 1025;

}
}
}

#endif //__LTR_ROBOT_M1_SPORT_API_HPP__

