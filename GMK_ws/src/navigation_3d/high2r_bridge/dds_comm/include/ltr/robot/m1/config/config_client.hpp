#ifndef __LTR_ROBOT_M1_CONFIG_CLIENT_HPP__
#define __LTR_ROBOT_M1_CONFIG_CLIENT_HPP__

#include <ltr/robot/client/client.hpp>
#include <ltr/robot/m1/config/config_api.hpp>
#include <string>

namespace ltr
{
namespace robot
{
namespace m1
{
/*
 * ConfigClient - M1 series configuration management client
 */
class ConfigClient : public Client
{
public:
    explicit ConfigClient(bool enableLease = false);
    ~ConfigClient();

    void Init();

    // Get configuration
    int32_t GetConfig(const std::string& key, std::string& value);
    
    // Set configuration
    int32_t SetConfig(const std::string& key, const std::string& value);
    
    // Get all configurations
    int32_t GetAllConfig(std::string& config);
    
    // Delete configuration
    int32_t DelConfig(const std::string& key);
};

}
}
}

#endif//__LTR_ROBOT_M1_CONFIG_CLIENT_HPP__

