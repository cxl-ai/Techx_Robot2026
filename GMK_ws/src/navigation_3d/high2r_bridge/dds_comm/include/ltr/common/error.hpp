#ifndef __LTR_ERROR_HPP__
#define __LTR_ERROR_HPP__

#include <ltr/common/decl.hpp>

namespace ltr
{
// Declare error codes
#define LTR_DECL_ERR(name, code, desc)   \
    const int32_t name = code; const std::string name##_DESC = desc;

#define LTR_DESC_ERR(name) name##_DESC

LTR_DECL_ERR(LTR_OK,              0,      "Success")
LTR_DECL_ERR(LTR_ERR_COMMON,      1001,   "common error")
LTR_DECL_ERR(LTR_ERR_BADCAST,     1002,   "Bad cast error")
LTR_DECL_ERR(LTR_ERR_FUTURE,      1003,   "Future error")
LTR_DECL_ERR(LTR_ERR_FUTURE_FAULT,1004,   "Future fault error")
LTR_DECL_ERR(LTR_ERR_JSON,        1005,   "Json data error")
LTR_DECL_ERR(LTR_ERR_SYSTEM,      1006,   "System error")
LTR_DECL_ERR(LTR_ERR_FILE,        1007,   "File operation error")
LTR_DECL_ERR(LTR_ERR_SOCKET,      1008,   "Socket operaton error")
LTR_DECL_ERR(LTR_ERR_IO,          1009,   "IO operaton error")
LTR_DECL_ERR(LTR_ERR_LOCK,        1010,   "Lock operation error")
LTR_DECL_ERR(LTR_ERR_NETWORK,     1011,   "Network error")
LTR_DECL_ERR(LTR_ERR_TIMEOUT,     1012,   "Timeout error")
LTR_DECL_ERR(LTR_ERR_UNKNOWN,     -1,     "Unknown error")

}
#endif//__LTR_ERROR_HPP__

