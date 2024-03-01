/*!
  @file gob_rudp.hpp
  @brief RUDP related definition

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#ifndef GOBLIB_RUDP_HPP
#define GOBLIB_RUDP_HPP

#include <cstdint>
#include <type_traits>

namespace goblib { namespace esp_now {
/*!
  @struct RUDP
  @brief RUDP-like block
  @details It does not meet all of the RUDP standards.
  It is a simplified implementation.
*/
struct RUDP
{
    ///@cond
    static constexpr uint8_t _SYN = 0x80; // Begin session
    static constexpr uint8_t _ACK = 0x40; // Acknowledge
    static constexpr uint8_t _EAK = 0x20; // Selective ACK
    static constexpr uint8_t _RST = 0x10; // End session
    static constexpr uint8_t _NUL = 0x08; // Heartbeat
    static constexpr uint8_t _CHK = 0x04; // Calculate checksum include payload?
    static constexpr uint8_t _TCS = 0x02; // Demand for resumption
    ///@endcond
    
    /*!
      @enum Flag
      @brief Flags of RUDP
     */
    enum class Flag : uint8_t
     {
         NONE    = 0,           //!< @brief Unreliable
         SYN     = _SYN,        //!< @brief Begin session
         SYN_ACK = _SYN | _ACK, //!< @brief Begin session with ACK
         ACK     = _ACK,        //!< @brief Acknowledge (with payload if exists)
         //ACK_CHK = _ACK | _CHK,
         //EAK     = _ACK | _EAK,
         RST     = _RST,        //!< @brief End session
         RST_ACK = _ACK | _RST, //!< @brief End session with ACK
         NUL     = _NUL | _ACK, //!< @brief Heartbeat
         //TCS = _TCS,
         //TCS_ACK = _TCS | _ACK,
     };
    using flag_t = std::underlying_type<Flag>::type; //!< @brief Flag type
    static_assert(sizeof(flag_t) == sizeof(uint8_t), "Illegal size");
    
    flag_t  flag{};     //!< @brief flag ==0:Unreliable !=0:RUDP flags
    uint8_t sequence{}; //!< @brief Lower 8bit of sequence
    uint8_t ack{};      //!< @brief Lower 8bit of ack
    uint8_t _sum{};     //!< @brief Check sum (Not supported yet...)

}  __attribute__((__packed__));

//
}}  
#endif
