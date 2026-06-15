#ifndef __LTR_DDS_TRAITS_HPP__
#define __LTR_DDS_TRAITS_HPP__

#include <string>
#include <typeinfo>

namespace ltr
{
namespace common
{
// Helper function to get type name for logging
template<typename T>
std::string DdsGetTypeName()
{
    return std::string(typeid(T).name());
}

}
}

#endif//__LTR_DDS_TRAITS_HPP__

