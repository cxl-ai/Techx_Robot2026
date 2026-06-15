#ifndef __LTR_ROBOT_M1_MOTION_SWITCHER_CLIENT_HPP__
#define __LTR_ROBOT_M1_MOTION_SWITCHER_CLIENT_HPP__

#include <ltr/robot/client/client.hpp>

namespace ltr
{
namespace robot
{
namespace m1
{
/*
 * @brief MotionSwitcherClient - M1 motion mode switcher client
 */
class MotionSwitcherClient : public Client
{
public:
    explicit MotionSwitcherClient();
    ~MotionSwitcherClient();

    void Init();

    int32_t CheckMode(std::string& form, std::string& name);
    int32_t SelectMode(const std::string& nameOrAlias);
    int32_t ReleaseMode();
    int32_t SetSilent(bool silent);
    int32_t GetSilent(bool& silent);
};

}
}
}

#endif//__LTR_ROBOT_M1_MOTION_SWITCHER_CLIENT_HPP__

