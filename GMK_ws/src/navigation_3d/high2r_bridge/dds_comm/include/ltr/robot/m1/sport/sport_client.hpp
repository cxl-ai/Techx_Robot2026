#ifndef __LTR_ROBOT_M1_SPORT_CLIENT_HPP__
#define __LTR_ROBOT_M1_SPORT_CLIENT_HPP__

#include <ltr/robot/client/client.hpp>
#include <vector>

namespace ltr
{
namespace robot
{
namespace m1
{
/*
 * PathPoint - Path point structure for trajectory following
 */
struct stPathPoint
{
    float timeFromStart;
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float vyaw;
};

typedef struct stPathPoint PathPoint;

/*
 * SportClient - M1 series sport control client
 */
class SportClient : public Client
{
public:
    explicit SportClient(bool enableLease = false);
    ~SportClient();

    void Init();
    
    // Basic motion commands
    int32_t Damp();
    
    int32_t BalanceStand();

    int32_t StopMove();

    int32_t StandUp();
    
    int32_t StandDown();
    
    int32_t RecoveryStand();
    
    // Movement commands
    int32_t Move(float vx, float vy, float vyaw);
    
    int32_t SwitchGait(int d);

    int32_t BodyHeight(float height);
    
    int32_t SpeedLevel(int level);
    
    int32_t TrajectoryFollow(std::vector<ltr::robot::m1::PathPoint> &path);

    int32_t ContinuousGait(bool flag);

    int32_t MoveToPos(float x, float y, float yaw);
    
    int32_t SwitchMoveMode(bool flag);
    
    int32_t VisionWalk(bool flag);

    int32_t HandStand(bool flag);

    int32_t AutoRecoverySet(bool flag);

    int32_t FreeWalk();

    int32_t ClassicWalk(bool flag);

    int32_t FastWalk(bool flag);

    int32_t Euler(float roll, float pitch, float yaw);
};

}
}
}

#endif//__LTR_ROBOT_M1_SPORT_CLIENT_HPP__

