/*!
  @file gob_esp_now_utility.hpp
  @brief Utilities for gob_esp_now

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#ifndef GOBLIB_ESP_NOW_UTILITY_HPP
#define GOBLIB_ESP_NOW_UTILITY_HPP

#include <cstdint>
#include <type_traits>
#include <FreeRTOS/freeRTOS.h>
#include <FreeRTOS/semphr.h>
#if !defined(NDEBUG)
#include <esp_timer.h>
#endif
#include <Wstring.h>

namespace goblib { namespace esp_now {

constexpr char LIB_TAG[] = "GEN";

// Porting std::to_underlying (C++23)
template<typename E> constexpr inline typename std::underlying_type<E>::type to_underlying(const E& e) noexcept
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

// Restore values based on u64 (assuming u8 is earlier)
inline uint64_t restore_u64_earlier(const uint64_t u64, const uint8_t u8)
{
    return ((u64 - ((uint8_t)(u64 & 0xFF) < u8) * 0x100) & ~static_cast<uint64_t>(0xff)) | u8;
}

// Restore values based on u64 (assuming u8 is later)
inline uint64_t restore_u64_later(const uint64_t u64, const uint8_t u8)
{
    return ((u64 + ((uint8_t)(u64 & 0xFF) > u8) * 0x100) & ~static_cast<uint64_t>(0xff)) | u8;
}

// lock_guard
// Lock in scope for recursive mutex
struct lock_guard
{
    explicit lock_guard (SemaphoreHandle_t& s) : _sem(&s) { xSemaphoreTakeRecursive(*_sem, portMAX_DELAY); }
    ~lock_guard()                                         { xSemaphoreGiveRecursive(*_sem); }
    lock_guard() = delete;
    lock_guard(const lock_guard&) = delete;
    lock_guard(lock_guard&&) = delete;
    lock_guard& operator=(const lock_guard&) = delete;
    lock_guard& operator=(lock_guard&&) = delete;
  private:
    SemaphoreHandle_t* _sem{};
};

String formatString(const char* fmt, ...);

#if !defined(NDEBUG)
// Simple Profiler for debug
class profile
{
  public:
    explicit profile(int64_t& out) : _save(out) { _start_at = esp_timer_get_time(); }
    ~profile() {_save = esp_timer_get_time() - _start_at; }
  private:
    int64_t _start_at{};
    int64_t& _save;
};
#endif
//
}}
#endif
