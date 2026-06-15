#ifndef __LTR_STRING_TOOL_HPP__
#define __LTR_STRING_TOOL_HPP__

#include <ltr/common/decl.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace ltr
{
namespace common
{
const char LTR_TRIM_DELIM_STR[] = {0x09,0x0A,0x0B,0x0C,0x0D,0x20,0x00};

std::string ToString(const char* s);
std::string ToString(const std::string& s);

template <typename T>
std::string ToString(const T& t)
{
    return std::to_string(t);
}

void StringTo(const std::string& s, int32_t& val);
void StringTo(const std::string& s, uint32_t& val);
void StringTo(const std::string& s, int64_t& val);
void StringTo(const std::string& s, uint64_t& val);
void StringTo(const std::string& s, float& val);
void StringTo(const std::string& s, double& val);
void StringTo(const std::string& s, long double& val);

template<typename T>
T StringTo(const std::string& s)
{
    T t;
    StringTo(s, t);
    return t;
}

std::string& ToUpper(std::string& s);
std::string& ToLower(std::string& s);

int32_t Compare(const std::string& s1, const std::string& s2,
    bool caseSensitive = true);
int32_t Compare(const std::string& s1, size_t pos, size_t len,
    const std::string& s2, bool caseSensitive = true);

std::string& TrimLeft(std::string& s, const std::string& delim = std::string(LTR_TRIM_DELIM_STR));
std::string& TrimRight(std::string& s, const std::string& delim = std::string(LTR_TRIM_DELIM_STR));
std::string& Trim(std::string& s, const std::string& delim = std::string(LTR_TRIM_DELIM_STR));

void Split(const std::string& s, std::vector<std::string>& parts,
    const std::string& delim);

bool StartWith(const std::string& s, const std::string& start,
    bool caseSensitive = true);

bool EndWith(const std::string& s, const std::string& end,
    bool caseSensitive = true);

std::string Replace(const std::string& s, const std::string& partten,
    const std::string& target);
}
}
#endif//__LTR_STRING_TOOL_HPP__

