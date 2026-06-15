#include "../include/dds_subscriber.hpp"
#include <ltr/robot/channel/channel_factory.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <sstream>
#include <thread>
#include <chrono>

bool DDS_Subscriber::isInterfaceUpAndRunning(const std::string& ifname) {
    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) {
        return false;
    }

    bool ok = false;
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_name) continue;
        if (std::strcmp(ifa->ifa_name, ifname.c_str()) != 0) continue;

        unsigned int flags = ifa->ifa_flags;
        if ((flags & IFF_UP) && (flags & IFF_RUNNING)) {
            ok = true;
            break;
        }
    }

    freeifaddrs(ifaddr);
    return ok;
}


void DDS_Subscriber::Init()
{   
    const std::string file_name = Config::path_2_config_directory + "config/config.ini";
    boost::property_tree::ptree pt;
    boost::property_tree::read_ini(file_name, pt);
    // Read network interface name (can be any network interface, not necessarily WiFi)
    NETWORK_INTERFACE = pt.get<std::string>("Network.NETWORK_INTERFACE", "lo");
    
    if (isInterfaceUpAndRunning(NETWORK_INTERFACE))
    {
        std::cout << GREEN << "[DDS Comm] Network Interface: " << NETWORK_INTERFACE << " is up and running" << RESET << std::endl;
        ChannelFactory::Instance()->Init(0, NETWORK_INTERFACE);
    }
    else
    {   
        std::cout << RED << "[DDS Comm] Network Interface: " << NETWORK_INTERFACE << " is disconnected!" << RESET << std::endl;
        ChannelFactory::Instance()->Init(0);
    }
    /*create subscriber*/
    std::cout << "[DDS Comm] Creating SportModeCmd subscriber for topic: " << TOPIC_SPORTMODECMD << std::flush << std::endl;
    
    // Test callback binding
    auto test_callback = std::bind(&DDS_Subscriber::SportModeCmdHandler, this, std::placeholders::_1);
    std::cout << "[DDS Comm] Callback bound successfully" << std::flush << std::endl;
    
    sportmodecmd_subscriber.reset(new ChannelSubscriber<ltr::msg::dds_::SportModeCmd_>(TOPIC_SPORTMODECMD));
    std::cout << "[DDS Comm] ChannelSubscriber created" << std::flush << std::endl;
    
    sportmodecmd_subscriber->InitChannel(test_callback, 0);  // queuelen=0 means no queue, direct callback
    std::cout << "[DDS Comm] SportModeCmd subscriber initialized successfully" << std::flush << std::endl;
    std::cout << "[DDS Comm] Waiting for DDS discovery (3 seconds)..." << std::flush << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "[DDS Comm] DDS discovery wait completed" << std::flush << std::endl;

    std::cout << "[DDS Comm] Creating VelCmd subscriber for topic: " << TOPIC_VELCMD << std::flush << std::endl;
    velcmd_subscriber.reset(new ChannelSubscriber<ltr::msg::dds_::VelCmd_>(TOPIC_VELCMD));
    velcmd_subscriber->InitChannel(std::bind(&DDS_Subscriber::VelCmdHandler, this, std::placeholders::_1), 0);  // queuelen=0 means no queue, direct callback
    std::cout << "[DDS Comm] VelCmd subscriber initialized successfully" << std::flush << std::endl;
}

void DDS_Subscriber::SportModeCmdHandler(const void* message)
{
    std::cout << "[DEBUG] SportModeCmdHandler called!" << std::endl;
    if (!message) {
        std::cout << "[ERROR] SportModeCmdHandler received null message!" << std::endl;
        return;
    }
    SportMode_Cmd = *(ltr::msg::dds_::SportModeCmd_*)message;
    std::cout << "device_id: " << static_cast<int>(SportMode_Cmd.device_id()) << std::endl;
    std::cout << "mode: " << static_cast<int>(SportMode_Cmd.mode()) << std::endl;

    switch (SportMode_Cmd.device_id())
    {
        case g20_controller::GAMEPAD :
            if(p_remoteController->rc_control_.ctrl_device != g20_controller::GAMEPAD)
            {
                std::cout << "Control device switch to GAMEPAD!" << std::endl;
                p_remoteController->rc_control_.ctrl_device = g20_controller::GAMEPAD;
            }
            break;
        case g20_controller::HIGHLEVELSDK :
            if(p_remoteController->rc_control_.ctrl_device != g20_controller::HIGHLEVELSDK)
            {
                std::cout << "Control device switch to HIGHLEVEL_SDK!" << std::endl;
                p_remoteController->rc_control_.ctrl_device = g20_controller::HIGHLEVELSDK;
            }
            break;
        default:
            break;
    }

    if(p_remoteController->rc_control_.ctrl_device == g20_controller::HIGHLEVELSDK) // highlevel sdk control
    {
        std::lock_guard lk(p_remoteController->rc_mtx_);
        switch (SportMode_Cmd.mode())
        {
            case DDS_Subscriber::PASSIVE:
                p_remoteController->rc_control_.mode = g20_controller::PASSIVE;
                break;
            case DDS_Subscriber::DAMPING:
                p_remoteController->rc_control_.mode = g20_controller::DAMPING;
                break;
            case DDS_Subscriber::SITDOWN:
                p_remoteController->rc_control_.mode = g20_controller::SITDOWN;
                break;
            case DDS_Subscriber::STANDUP:
                p_remoteController->rc_control_.mode = g20_controller::RECOVER_STAND;
                break;
            case DDS_Subscriber::NORMAL_WALK:
                p_remoteController->rc_control_.mode = g20_controller::RL_LOCO_CLASSIC;
                break;
            default:
                break;
        }
    }
}

void DDS_Subscriber::VelCmdHandler(const void* message)
{
    std::cout << "[DEBUG] VelCmdHandler called!" << std::endl;
    if (!message) {
        std::cout << "[ERROR] VelCmdHandler received null message!" << std::endl;
        return;
    }
    std::lock_guard lk(p_remoteController->rc_mtx_);
    
    // Debug: Check conditions
    std::cout << "[DEBUG] VelCmdHandler - ctrl_device=" << static_cast<int>(p_remoteController->rc_control_.ctrl_device) 
              << " (HIGHLEVELSDK=" << static_cast<int>(g20_controller::HIGHLEVELSDK) << ")" << std::endl;
    std::cout << "[DEBUG] VelCmdHandler - SportMode_Cmd.mode()=" << static_cast<int>(SportMode_Cmd.mode()) 
              << " (NORMAL_WALK=" << static_cast<int>(NORMAL_WALK) << ")" << std::endl;
    
    if(p_remoteController->rc_control_.ctrl_device==g20_controller::HIGHLEVELSDK && SportMode_Cmd.mode() == NORMAL_WALK)
    {
        VelCmd = *(ltr::msg::dds_::VelCmd_*)message;
        std::cout << "lin_vx: " << VelCmd.lin_vx() << std::endl;
        std::cout << "lin_vy: " << VelCmd.lin_vy() << std::endl;
        std::cout << "ang_wz: " << VelCmd.ang_wz() << std::endl;
    
        float lin_vx = std::clamp(VelCmd.lin_vx(), -LIN_VX_MAX_ABS, LIN_VX_MAX_ABS);
        float lin_vy = std::clamp(VelCmd.lin_vy(), -LIN_VY_MAX_ABS, LIN_VY_MAX_ABS);
        float ang_wz = std::clamp(VelCmd.ang_wz(), -ANG_WZ_MAX_ABS, ANG_WZ_MAX_ABS);
        
        p_remoteController->rc_control_.v_des[0] = lin_vx;
        p_remoteController->rc_control_.v_des[1] = lin_vy;
        p_remoteController->rc_control_.v_des[2] = ang_wz;
    }
    else
    {
        std::cout << "[DEBUG] VelCmdHandler - Conditions not met, skipping VelCmd processing" << std::endl;
        std::cout << "[DEBUG] VelCmdHandler - Need: ctrl_device==HIGHLEVELSDK(" << static_cast<int>(g20_controller::HIGHLEVELSDK) 
                  << ") AND mode==NORMAL_WALK(" << static_cast<int>(NORMAL_WALK) << ")" << std::endl;
    }
}