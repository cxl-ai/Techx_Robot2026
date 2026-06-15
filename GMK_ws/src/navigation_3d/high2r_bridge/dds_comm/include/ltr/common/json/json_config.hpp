#ifndef __LTR_JSON_CONFIG_HPP__
#define __LTR_JSON_CONFIG_HPP__

#include <ltr/common/json/json.hpp>
#include <ltr/common/any.hpp>

#define LTR_JSON_CONF_KEY_PARAMETER  "Parameter"

namespace ltr
{
namespace common
{
class JsonConfig
{
public:
    JsonConfig();
    virtual ~JsonConfig();

    JsonConfig(const std::string& configFileName);

    virtual void Parse(const std::string& configFileName);
    virtual void ParseContent(const std::string& content);

    //top-level field
    bool Has(const std::string& name) const;

    //top-level field
    const Any& Get(const std::string& name) const;

    //top-level field
    template<typename T>
    const T& Get(const std::string& name) const
    {
        return AnyCast<T>(Get(name));
    }

    //top-level field
    template<typename T>
    T GetNumber(const std::string& name) const
    {
        // Simplified: assume T is numeric and can be cast
        const Any& a = Get(name);
        if (a.Empty())
        {
            return T(0);
        }
        // For now, return the value directly if type matches
        return AnyCast<T>(a);
    }

    //top-level field
    template<typename T>
    T Get(const std::string& name, const T& defValue) const
    {
        const Any& a = Get(name);

        if (a.Empty())
        {
            return defValue;
        }

        return AnyCast<T>(a);
    }

    //top-level field
    template<typename T>
    T GetNumber(const std::string& name, const T& defValue) const
    {
        const Any& a = Get(name);

        if (a.Empty())
        {
            return defValue;
        }

        return AnyCast<T>(a);
    }

    //top-level field: Parameter
    bool HasParameter(const std::string& name) const;

    //top-level field: Parameter
    const JsonMap& GetParameter() const;

    //field from top-level field: Parameter
    const Any& GetParameter(const std::string& name) const;

    //field from top-level field: Parameter
    template<typename T>
    const T& GetParameter(const std::string& name) const
    {
        return AnyCast<T>(GetParameter(name));
    }

    //field from top-level field: Parameter
    template<typename T>
    T GetNumberParameter(const std::string& name) const
    {
        const Any& a = GetParameter(name);
        if (a.Empty())
        {
            return T(0);
        }
        return AnyCast<T>(a);
    }

    //field from top-level field: Parameter
    template<typename T>
    T GetParameter(const std::string& name, const T& defValue) const
    {
        const Any& a = GetParameter(name);

        if (a.Empty())
        {
            return defValue;
        }

        return AnyCast<T>(a);
    }

    //field from top-level field: Parameter
    template<typename T>
    T GetNumberParameter(const std::string& name, const T& defValue) const
    {
        const Any& a = GetParameter(name);

        if (a.Empty())
        {
            return defValue;
        }

        return AnyCast<T>(a);
    }

private:
    void ParseInner(const std::string& content);

private:
    JsonMap mJsonMap;
    JsonMap mParameter;
};

}
}
#endif//__LTR_JSON_CONFIG_HPP__

