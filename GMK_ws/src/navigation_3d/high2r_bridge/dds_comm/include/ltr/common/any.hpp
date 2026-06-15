#ifndef __LTR_ANY_HPP__
#define __LTR_ANY_HPP__

#include <ltr/common/exception.hpp>
#include <ltr/common/decl.hpp>
#include <typeinfo>
#include <string>

namespace ltr
{
namespace common
{
class Any
{
public:
    Any()
        : mContent(0)
    {}

    template<typename ValueType>
    Any(const ValueType& value)
        : mContent(new Holder<ValueType>(value))
    {}

    Any(const char* s)
        : Any(std::string(s))
    {}

    Any(const char* s, size_t len)
        : Any(std::string(s, len))
    {}

    Any(const Any& other)
        : mContent(other.mContent ? other.mContent->Clone() : 0)
    {}

    ~Any()
    {
        delete mContent;
        mContent = 0;
    }

    Any& Swap(Any& other)
    {
        std::swap(mContent, other.mContent);
        return *this;
    }

    bool Empty() const
    {
        return mContent == 0;
    }

    const std::type_info& GetTypeInfo() const
    {
        return mContent ? mContent->GetTypeInfo() : typeid(void);
    }

    template<typename ValueType>
    Any& operator=(const ValueType& other)
    {
        Any(other).Swap(*this);
        return *this;
    }

    Any& operator=(Any other)
    {
        other.Swap(*this);
        return *this;
    }

public:
    class PlaceHolder
    {
    public:
        virtual ~PlaceHolder()
        {}

    public:
        virtual const std::type_info& GetTypeInfo() const = 0;
        virtual PlaceHolder* Clone() const = 0;
    };

    template<typename ValueType>
    class Holder : public PlaceHolder
    {
    public:
        explicit Holder(const ValueType& value)
            : mValue(value)
        {}

        virtual const std::type_info& GetTypeInfo() const
        {
            return typeid(ValueType);
        }

        virtual PlaceHolder* Clone() const
        {
            return new Holder(mValue);
        }

    public:
        ValueType mValue;
    };

public:
    PlaceHolder* mContent;
};

static const Any LTR_EMPTY_ANY = Any();

template<typename ValueType>
const ValueType& AnyCast(const Any* operand)
{
    const std::type_info& t1 = typeid(ValueType);
    const std::type_info& t2 = operand->GetTypeInfo();

    if (t1 == t2)
    {
        return ((Any::Holder<ValueType>*)(operand->mContent))->mValue;
    }

    LTR_THROW(BadCastException, std::string("AnyCast error. target type is ")
        + t1.name() + ", but source type is " + t2.name());
}

template<typename ValueType>
const ValueType& AnyCast(const Any& operand)
{
    return AnyCast<ValueType>(&operand);
}

}
}
#endif//__LTR_ANY_HPP__

