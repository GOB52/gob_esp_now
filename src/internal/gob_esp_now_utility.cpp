/*!
  @file gob_esp_now_definition.cpp
  @brief Common definitions for libraries

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include "gob_esp_now_utility.hpp"
#include <cstdarg>

namespace goblib { namespace esp_now {

String formatString(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    size_t sz = vsnprintf(nullptr, 0U, fmt, args); // calculate length
    va_end(args);
    
    char buf[sz + 1];
    va_start(args, fmt); // Reinitiaize args (args cannot reuse because indefinite value after vsnprintf)
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    // String don't has constructor(const char*, const size_t);
    buf[sz] = '\0';
    return String(buf);
}
//
}}
