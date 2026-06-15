#ifndef __LTR_ROBOT_SDK_INERNAL_API_HPP__
#define __LTR_ROBOT_SDK_INERNAL_API_HPP__

#include <ltr/common/decl.hpp>
#include <ltr/common/json/json.hpp>
#include <ltr/common/json/jsonize.hpp>
#include <string>
#include <sstream>

namespace ltr
{
namespace robot
{
/*
 * @brief  max internal api id
 * @value: 100
 */
const int32_t ROBOT_INTERNAL_API_ID_MAX             = 100;

///////////////////////////////////////////////////////////////
/*
 * @brief  invailed api id
 * @value: -1
 */
const int32_t ROBOT_API_ID_NONE                     = -1;

/*
 * @brief  Get api version from server.
 * @value: 1
 */
const int32_t ROBOT_API_ID_INTERNAL_API_VERSION     = 1;

/*
 * @brief  Noop.
 * @value: 2
 */
const int32_t ROBOT_API_ID_INTERNAL_API_NOOP        = 2;

/*
 * @brief  Apply lease from server.
 * @value: 101
 */
const int32_t ROBOT_API_ID_LEASE_APPLY              = 101;
/*
 * @brief  Renewal lease term from server.
 * @value: 102
 */
const int32_t ROBOT_API_ID_LEASE_RENEWAL            = 102;

///////////////////////////////////////////////////////////////

/*
 * @brief  robot lease term default.
 * @value: default 1000000 us (1 second)
 */
const int64_t ROBOT_LEASE_TERM                      = 1000000;

/*
 * macro: IS_INTERNAL_API
 */
#define IS_INTERNAL_API(apiId) ((apiId) <= ROBOT_INTERNAL_API_ID_MAX)

///////////////////////////////////////////////////////////////

/*
 * @brief  Input parameter type for ROBOT_API_ID_LEASE_APPLY
 * @class: ApplyLeaseParameter
 */
class ApplyLeaseParameter : public common::Jsonize
{
public:
    ApplyLeaseParameter()
    {}

    ~ApplyLeaseParameter()
    {}

    void fromJson(common::JsonMap& json) override
    {
        auto it = json.find("name");
        if (it != json.end())
        {
            common::FromJson(it->second, name);
        }
    }

    void toJson(common::JsonMap& json) const override
    {
        json["name"] = common::Any(name);
    }

public:
    std::string name;
};

/*
 * @brief  Output data type for ROBOT_API_ID_LEASE_APPLY
 * @class: ApplyLeaseData
 */
class ApplyLeaseData : public common::Jsonize
{
public:
    ApplyLeaseData() : id(0), term(0)
    {}

    ~ApplyLeaseData()
    {}

    void fromJson(common::JsonMap& json) override
    {
        // Use direct access like unitree_sdk2, which is simpler and more reliable
        if (json.find("id") != json.end())
        {
            common::FromJson(json["id"], id);
        }
        if (json.find("term") != json.end())
        {
            common::FromJson(json["term"], term);
        }
    }

    void toJson(common::JsonMap& json) const override
    {
        json["id"] = common::Any(id);
        json["term"] = common::Any(term);
    }

public:
    int64_t id;
    int64_t term;
};

}
}
#endif//__LTR_ROBOT_SDK_INERNAL_API_HPP__

