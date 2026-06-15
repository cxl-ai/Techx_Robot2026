#ifndef __LTR_ROBOT_M1_ROBOT_STATE_CLIENT_HPP__
#define __LTR_ROBOT_M1_ROBOT_STATE_CLIENT_HPP__

#include <ltr/robot/client/client.hpp>
#include <string>

namespace ltr
{
namespace robot
{
namespace m1
{
/*
 * RobotStateClient - M1 series robot state query client
 */
class RobotStateClient : public Client
{
public:
    explicit RobotStateClient(bool enableLease = false);
    ~RobotStateClient();

    void Init();

    // Query robot state
    int32_t GetRobotState(std::string& state);
    
    // Query motor states
    int32_t GetMotorStates(std::string& states);
    
    // Query IMU state
    int32_t GetIMUState(std::string& imuState);
    
    // Query battery state
    int32_t GetBatteryState(std::string& batteryState);
};

}
}
}

#endif//__LTR_ROBOT_M1_ROBOT_STATE_CLIENT_HPP__

