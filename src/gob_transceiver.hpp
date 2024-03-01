/*!
  @file gob_transceiver.hpp
  @brief Transceiver(TRX) base class.
  @details The transceiver belongs to the communicator and sends and receives data to and from other transceivers with the same ID on other devices.

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#ifndef GOBLIB_TRANSCEIVER_HPP
#define GOBLIB_TRANSCEIVER_HPP

#include "gob_rudp.hpp"
#include "gob_mac_address.hpp"
#include "internal/gob_esp_now_utility.hpp"
#include "internal/gob_esp_now_vmap.hpp"

namespace goblib { namespace esp_now {

class Communicator;

/*!
  @struct TransceiverHeader
  @brief Transceiver data header
*/
struct TransceiverHeader
{
    uint8_t tid{};  //!< @brief Identifier
    uint8_t size{}; //!< @brief Data size including this header
    RUDP    rudp{}; //!< @brief RUDP header
    // Payload continues if exists

    ///@name Properties
    ///@{
    inline bool isRUDP() const       { return (bool)rudp.flag; } //!< @brief is RUDP data?
        inline bool isSYN() const        { return (rudp.flag & to_underlying(RUDP::Flag::SYN)); } //!< @brief is SYN ?
        inline bool onlySYN() const      { return (rudp.flag == to_underlying(RUDP::Flag::SYN)); } //!< @brief only SYN?
    //inline bool isSYN() const        { return (rudp.flag == to_underlying(RUDP::Flag::SYN)); } //!< @brief is SYN ?

    inline bool isRST() const        { return (rudp.flag & to_underlying(RUDP::Flag::RST)); } //!< @brief is RST?
    inline bool isACK() const        { return (rudp.flag & to_underlying(RUDP::Flag::ACK)); } //!< @brief is ACK?
    inline bool onlyACK() const      { return (rudp.flag == to_underlying(RUDP::Flag::ACK)); } //!< @brief only ACK?
    inline bool isNUL() const        { return (rudp.flag == to_underlying(RUDP::Flag::NUL)); } //!< @brief is NUL?
    inline bool hasPayload() const   { return size > sizeof(*this); } //!< @brief Has payload?
    inline uint8_t payloadSize() const { return hasPayload() ? size - sizeof(*this) : 0; } //!< @brief payload size if exists
    inline uint8_t* payload() const  { return hasPayload() ? ((uint8_t*)this) + sizeof(*this) : nullptr; } //!< @brief Gets the payload pointer if exists.
    inline bool needReturnACK() const { return (isACK() && payload()) || isNUL() || (isRST() && isACK()); }
    ///@}
}  __attribute__((__packed__));


/*!
   @class Transceiver
   @brief Post and receive data each.
   @note Unique data exchange is done in derived classes. See also library examples.
*/
class Transceiver
{
  public:
    /*!
      @param tid Unique value for each transceiver (Other than 0)
      @note id also serves as priority (in ascending order)
      @warning tid other than 0
     */
    explicit Transceiver(const uint8_t tid);
    virtual ~Transceiver();

    ///@cond
    Transceiver(const Transceiver&) = delete;
    Transceiver& operator=(const Transceiver&) = delete;
    ///@endcond

    ///@name Properties
    ///@{
    inline uint8_t identifier() const { return _tid; } //!< @brief Gets the identifier
    ///@}

    ///@name Delivered up to this sequence number?
    ///@{
    //! @brief Has this sequence number been delivered to this address?
    inline bool delivered(const uint64_t seq, const MACAddress& addr)
    {
        return seq <= _peerInfo[addr].recvAck;
    }
    ///!@brief Has this sequence number been delivered to all peers?
    bool delivered(const uint64_t seq);
    ///@}
    
    //! @brief Reset sequence,ack...
    void reset();
    //! @brief Clear information about the specified address
    void clear(const MACAddress& addr);
    
    ///@name Data transmission (Reliable)
    ///@note Has mechanisms for transmission sequencing and arrival assurance
    ///@{
    /*!
      @brief Post data
      @param peer_addr Destination address. Post to all peer if nullptr
      @param data Payload data if exists
      @param length Length of the payload data if exists
      @retval !=0 Sent sequence
      @retval ==0 Failed to post
      @note In the case of Post, data are combined.
      @note To reduce the number of transmissions when sending multiple data or transceivers.
     */
    uint64_t postReliable(const uint8_t* peer_addr, const void* data, const uint8_t length);
    //! @brief Post to all peer
    inline uint64_t postReliable(                   const void* data, const uint8_t length) { return postReliable(nullptr, data, length); }
    //! @brief Post to destination
    template<typename T> inline uint64_t postReliable(const MACAddress& addr, const T& data) { return postReliable(addr.peer_addr(), &data ,sizeof(data)); }
    //! @brief Post to all peer
    template<typename T> inline uint64_t postReliable(                        const T& data) { return postReliable(nullptr,     &data ,sizeof(data)); }

#if 0    
    /*!
      @brief Send data
      @param peer_addr Destination address. Post to all peer if nullptr
      @param data Payload data if exists
      @param length Length of the payload data if exists
      @retval !=0 Sent sequence
      @retval ==0 Failed to post
      @warning ESP-NOW does not allow frequent calls to be sent without a send callback being received,
      @warning as such calls will result in an error.In that case, the library will return failure.
      @warning <em>Unlike post, if it fails, it is not automatically resubmitted by update,
      @warning so you have to manage resubmission yourself.</em>
     */
    uint64_t sendReliable(const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);
    //! @brief Send to all peer
    inline uint64_t sendReliable(                   const void* data = nullptr, const uint8_t length = 0) { return sendReliable(nullptr, data, length); }
    //! @brief Send to destination
    template<typename T> inline uint64_t sendReliable(const MACAddress& addr, const T& data) { return sendReliable(addr.data(), &data ,sizeof(data)); }
    //! @brief Send to all peer
    template<typename T> inline uint64_t sendReliable(                        const T& data) { return sendReliable(nullptr,     &data ,sizeof(data)); }
#endif
    ///@}
    
    ///@name Data transmission (Unreliable)
    ///@note Parameters are the same as post/sendReliable
    ///@{
    bool postUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length);
    inline bool postUnreliable(                   const void* data, const uint8_t length)  { return postUnreliable(nullptr, data, length); }
    template<typename T> inline bool postUnreliable(const MACAddress& addr, const T& data) { return postUnreliable(addr.data(), &data ,sizeof(data)); }
    template<typename T> inline bool postUnreliable(                        const T& data) { return postUnreliable(nullptr,     &data ,sizeof(data)); }
    bool sendUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length);
    inline bool sendUnreliable(                   const void* data, const uint8_t length)  { return sendUnreliable(nullptr, data, length); }
#if 0
    template<typename T> inline bool sendUnreliable(const MACAddress& addr, const T& data) { return sendUnreliable(addr.data(), &data ,sizeof(data)); }
    template<typename T> inline bool sendUnreliable(                        const T& data) { return sendUnreliable(nullptr,     &data ,sizeof(data)); }
#endif
    ///@}    
   
    /*!
      @brief Call any function with lock
      @param Func Any functor
      @param args Arguments for functor
      @details Example
      @code{.cpp}
      // Example in class functions
      class YourTransceiver : public Transceiver
      {
        public:
          struct Payload { int value; };
          explicit YourTransceiver(const uint8_t tid) : Transceiver(tid) {}
          int value() const { return _value; }
    
        protected:
          virtual void onReceive(const MACAddress& addr, const TransceiverHeader* data) override
          {
              with_lock([this](const TransceiverHeader* th) { this->_value = ((Payload*)th->payload())->value; }, data);
          }
          volatile int _value{};
      };

      // Example of external use of the class
      YourTransceiver yt(1);
      void foo()
      {
          int arg{3};
          auto ret = yt.with_lock([](const int v)
          {
              return v * yt.value();
          }, arg);
      }
      @endcode
     */
    template<typename Func, typename... Args> auto with_lock(Func func, Args&&... args)
            -> decltype(func(std::forward<Args>(args)...))
    {
        lock_guard lock(_sem);
        return func(std::forward<Args>(args)...);
    }

    template<typename Func, typename... Args> auto with_lock(Func func, Args&&... args) const
            -> decltype(func(std::forward<Args>(args)...))
    {
        lock_guard lock(_sem);
        return func(std::forward<Args>(args)...);
    }

    
#if !defined(NDEBUG) || defined(DOXYGEN_PROCESS)
    ///@name Debugging features
    ///@warning Conditions of use, <em><strong>NDEBUG must NOT be DEFINED.</strong></em>
    ///@{
    virtual String debugInfo() const; //!< @brief Gets the information string
    ///@}
#endif    

  protected:
    /*!
      @brief update transceiver
      @note Call in Communicator::update
      @warning Note that the communicator is locked
     */
    virtual void update(const unsigned long ms) {}

    /*!
      @brief Receiving callback
      @param addr Sender MAC address
      @param data Pointer of the payload data
      @param length Length of data
      @note Called only exists the payload
      @warning The receiving callback function also runs from the Wi-Fi task (Core 0),
      @warning <em><strong>So, do not do lengthy operations in the callback function.</strong></em>
    */
    virtual void onReceive(const MACAddress& addr, const void* data, const uint8_t length) {}
    
    void build_peer_map();
    bool post_rudp(uint64_t& seq, const uint8_t* peer_addr, const RUDP::Flag flag, const void* data = nullptr, const uint8_t length = 0);
    inline bool post_ack(const uint8_t* peer_addr) { uint64_t seq{}; return post_rudp(seq, peer_addr, RUDP::Flag::ACK) != 0; }
    inline bool post_ack(const MACAddress& addr)   { return post_ack(static_cast<const uint8_t*>(addr)); }
    inline bool post_nul(const uint8_t* peer_addr) { uint64_t seq{}; return post_rudp(seq, peer_addr, RUDP::Flag::NUL) != 0; }
    inline bool post_nul(const MACAddress& addr)   { return post_nul(static_cast<const uint8_t*>(addr)); }
    
    bool make_data(uint64_t& seq, uint8_t* buf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);

    inline bool delivered(const uint8_t seq, const MACAddress& addr)
    {
        return delivered(restore_u64_earlier(_peerInfo[addr].sequence, seq), addr);
    }
    
    ///@cond
    struct PeerInfo
    {
        // ACK with payload
        uint64_t sequence{};  // S
        uint64_t recvSeq{};   // R (Sequence number received from the peer)
        uint64_t recvAck{};   // R (ACKed number received from the peer)
        uint64_t sentAck{};   // S Last sent ACKed number
        // no payload
        uint64_t ackSequence{}; // S
        uint64_t recvAckSeq{};  // R
        //
        unsigned long recvTime{}; // R
        bool needReturnACK{};     // R
    } __attribute__((__packed__));
    ///@endcond

    // If derived, call the parent on_receive first!
    // Normally, an override of onReceive should do the trick.
    virtual void on_receive(const MACAddress& addr, const TransceiverHeader* th);

  private:
    Transceiver();
    void _update(const unsigned long ms, const uint8_t cat, const uint8_t mca);


  protected:
    map_t<MACAddress, PeerInfo> _peerInfo;
    
  private:
    mutable SemaphoreHandle_t _sem{};
    uint8_t _tid{};    // Transceiver unique identifier (0 reserved for SystemTRX)
    friend class Communicator;
};
//
}}
#endif
