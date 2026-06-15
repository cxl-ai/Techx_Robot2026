#ifndef __LTR_ROBOT_SDK_SERIALIZE_HPP__
#define __LTR_ROBOT_SDK_SERIALIZE_HPP__

#include <ltr/common/json/jsonize.hpp>
#include <ltr/common/exception.hpp>

namespace ltr
{
namespace robot
{
template<typename T>
inline bool Serialize(const T& instance, std::string& serialziedData)
{
    try
    {
        serialziedData = common::ToJsonString(instance);
    }
    catch(const common::CommonException& e)
    {
        return false;
    }
    catch(...)
    {
        return false;
    }

    return true;
}

template<typename T>
inline bool Deserialize(const std::string& serialziedData, T& instance)
{
    try
    {
        instance = common::FromJsonString<T>(serialziedData);
    }
    catch(const common::CommonException& e)
    {
        return false;
    }
    catch(...)
    {
        return false;
    }

    return true;
}

}
}
#endif//__LTR_ROBOT_SDK_SERIALIZE_HPP__

