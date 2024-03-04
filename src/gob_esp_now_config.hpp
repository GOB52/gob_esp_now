/*!
  @file gob_esp_now_config.hpp
  @brief Configuration definition

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#ifndef GOB_ESP_NOW_CONFIG_HPP
#define GOB_ESP_NOW_CONFIG_HPP

#include <cstdint>
//#include <esp_now.h> // for ESP_NOW_MAX_DATA_LEN

namespace goblib { namespace esp_now {

/*!
  @struct config_t
  @brief Communicator configuration (include RUDP configuration)
  @warning Not all RUDP mechanisms are supported
  @sa https://datatracker.ietf.org/doc/html/draft-ietf-sigtran-reliable-udp-00
*/
struct config_t
{
    ///@name Communicator
    ///@{
    uint8_t update_priority{1};       //!< @brief Priority of the update task. Zero means If zero, the user explicitly calls it.
    uint8_t update_core{1};           //!< @brief CPU core on which the update task is executed
    uint8_t receive_priority{2};      //!< @brief Priority of the receive task
    uint8_t receive_core{1};          //!< @brief CPU core on which the receive task is executed
    uint8_t receive_queue_size{2};    //!< @brief Size of FreeRTOS Queue
    uint16_t task_stack_size{1024*3}; //!< @brief Size of the stack for task
    ///@}

    ///@name RUDP
    ///@{
    //uint8_t session{};     //!< @brief  Session id
    //uint8_t outstanding{}; //!< @brief  The maximum number of segments that should be sent without getting an acknowledgment. (as window size)

    //! @brief The maximum number of octets that can be received by the peer (Fixed ESP_NOW_MAX_DATA_LEN in this library)
    //uint8_t maximumSegmentSize{ESP_NOW_MAX_DATA_LEN};
    //! @brief The timeout value for retransmission of unacknowledged packets
    uint16_t retransmissionTimeout{320};

    //! @brief The timeout value for sending an acknowledgment segment if another segment is not sent
    uint16_t cumulativeAckTimeout{100};
    //! @brief The timeout value for sending a null segment if a data segment has not been sent
    uint16_t nullSegmentTimeout{10*1000};

    //! @brief This timeout value indicate the amount of time the state information will be saved for a connection before an auto reset occurs.        
    uint16_t transferStateTimeout{0};
    //! @brief The maximum number of times consecutive retransmission(s)
    uint8_t maxRetrans{8};
    //! @brief The maximum number of acknowledgments that will be accumulated before sending an acknowledgment
    uint8_t maxCumAck{4};
    //! @brief he maximum number of out of sequence packets that will be accumulated before an EACK segment is sent
    uint8_t maxOutOfSeq{0};
    //! @brief The maximum number of consecutive auto reset that will performed before a connection is reset
    uint8_t maxAutoReset{0};
    ///@}
}  __attribute__((__packed__));

//
}}
#endif
