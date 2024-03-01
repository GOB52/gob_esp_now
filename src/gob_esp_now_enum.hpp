/*!
  @file gob_esp_now_enum.hpp
  @brief Enumeration definition

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#ifndef GOB_ESP_NOW_ENUM_HPP
#define GOB_ESP_NOW_ENUM_HPP

#include <cstdint>

namespace goblib { namespace esp_now {

/*!
  @enum Role
  @brief Role of communicator
  @note ESP-NOW does not have the concept of Master and Slave. This is a role distinction for convenience.
*/
enum class Role : uint8_t
{
    None,      //!< @brief No role
    Primary,   //!< @brief Primary. Also known as master, controller(ESP-NOW Terminology)
    Secondary, //!< @brief Secondary. Also known as slave
    Hybrid,    //!< @brief Primary and secondary
};

/*!
  @enum Notify
  @brief Notify type
  notify and argument  Correspondence Chart for notify(), onNotify()
  | Notify | Argument |
  | --- | --- |
  | Disconnect | const MACAddress* |
  | ConnectionLost | const MACAddress* |
  | Shookhands | const MACAddress* |  
*/
enum class Notify : uint8_t
{
    Disconnect,     //!< @brief Actively disconnected
    ConnectionLost, //!< @brief Connection lost
    Shookhands,     //!< @brief Completed the 3-way handshake
};
//
}}
#endif
