#ifndef __LTR_JSON_HPP__
#define __LTR_JSON_HPP__

#include <ltr/common/any.hpp>
#include <map>
#include <vector>
#include <string>

namespace ltr
{
namespace common
{

typedef std::map<std::string, Any> JsonMap;
typedef std::vector<Any> JsonArray;

Any FromJsonString(const std::string& s);

std::string ToJsonString(const Any& a, bool pretty = false);

static inline bool IsNull(const Any& a)
{
    return a.Empty();
}

static inline bool IsJsonArray(const Any& a)
{
    return a.GetTypeInfo() == typeid(JsonArray);
}

static inline bool IsJsonMap(const Any& a)
{
    return a.GetTypeInfo() == typeid(JsonMap);
}

static inline bool IsJsonObject(const Any& a)
{
    return IsJsonMap(a);
}

}
}

#endif//__LTR_JSON_HPP__

