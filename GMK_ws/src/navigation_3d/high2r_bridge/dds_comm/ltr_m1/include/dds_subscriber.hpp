#pragma once

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <ifaddrs.h>
#include <net/if.h>
#include <string>
#include <cstring>
#include <cstdio>
#include <ltr/robot/channel/channel_subscriber.hpp>
#include <ltr/robot/channel/channel_factory.hpp>
#include "../../../hardwares/sbus/include/rt_sbus.h"
#include "../../../hardwares/sbus/include/rt_G20_controller.h"
#include "SportModeCmd_.hpp"
#include "VelCmd_.hpp"
#include "../../../../utilities/types/std_cout_colors.h"
#include "../../../../config/Config.h"
#include "../../../../utilities/inc/LoadData.h"

using namespace ltr::robot;

#define TOPIC_SPORTMODECMD "rt/ltr/sportmode_cmd"
#define TOPIC_VELCMD "rt/ltr/vel_cmd"

class DDS_Subscriber
{
public:
    explicit DDS_Subscriber(g20_controller::SBUSController *p_remoteController)
    : p_remoteController(p_remoteController)
    {
        Init();
    }

    ~DDS_Subscriber()
    {}

    void Init();
    bool isInterfaceUpAndRunning(const std::string& ifname);
    
    typedef enum HIGHLEVEL_MODE {
        PASSIVE = 0,
        DAMPING,
        SITDOWN,
        STANDUP,
        NORMAL_WALK,
        LION_DANCE,
        LION_DANCE_OFF,
    } HIGHLEVEL_MODE_t;
    std::string NETWORK_INTERFACE;  // Network interface name (can be any interface, not necessarily WiFi)

private:
    void SportModeCmdHandler(const void* message);
    void VelCmdHandler(const void* message);

private:
    ltr::msg::dds_::SportModeCmd_ SportMode_Cmd{};
    ltr::msg::dds_::VelCmd_ VelCmd{};
    /*subscriber*/
    ChannelSubscriberPtr<ltr::msg::dds_::SportModeCmd_> sportmodecmd_subscriber;
    ChannelSubscriberPtr<ltr::msg::dds_::VelCmd_> velcmd_subscriber;
    g20_controller::SBUSController *p_remoteController = nullptr;
    static constexpr float LIN_VX_MAX_ABS = 2.0f;
    static constexpr float LIN_VY_MAX_ABS = 0.6f;
    static constexpr float ANG_WZ_MAX_ABS = 0.8f;
};