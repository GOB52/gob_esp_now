/*!
  @file gob_esp_now.hpp
  @brief ESP-NOW wrapper, helper and utilities.

  @mainpage gob_esp_now
  This library is wrapped with ESP-NOW.  
  C++11 or later.
  
  @author GOB https://twitter.com/gob_52_gob

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#ifndef GOBLIB_ESP_NOW_HPP
#define GOBLIB_ESP_NOW_HPP

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>
#include <type_traits>
#include <FreeRTOS/freeRTOS.h>
#include <FreeRTOS/semphr.h>
#include <esp_now.h>
#include <WString.h>
#include "gob_mac_address.hpp"
#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
# pragma messgae "Using std::map"
# include <map>
#else
#include "internal/gob_esp_now_vmap.hpp"
#endif

/*!
  @namespace goblib
  @brief Top level namespace of mine
 */
namespace goblib {

/*
  @namespace esp_now
  @brief For ESP-NOW
 */
namespace esp_now {

constexpr char LIB_TAG[] = "gen"; //!< @brief Tag for logging

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

/*!
  @struct lock_guard
  @brief Scoped semaphore locking
 */
struct lock_guard
{
    explicit lock_guard (SemaphoreHandle_t& s) : _sem(&s) { xSemaphoreTake(*_sem, portMAX_DELAY); }
    ~lock_guard()                                         { xSemaphoreGive(*_sem); }
    lock_guard() = delete;
    lock_guard(const lock_guard&) = delete;
    lock_guard(lock_guard&&) = delete;
    lock_guard& operator=(const lock_guard&) = delete;
    lock_guard& operator=(lock_guard&&) = delete;
  private:
    SemaphoreHandle_t* _sem{};
};

/*!
  @struct CommunicatorHeader
  @brief Communicator data header
*/
struct CommunicatorHeader
{
    static constexpr uint16_t SIGNETURE = 0x5200; //!< @brief Packet signeture(8bit) + Header version(8bit)
    uint16_t signeture{SIGNETURE}; //!< @brief Signeture of the Communicator's data
    uint8_t  app_id{};             //!< @brief Application-specific ID
    uint8_t  count{};              //!< @brief Number of transceiver data
    uint8_t  size{sizeof(*this)};  //!< @brief Size of packet [ |CH|TH...|TH......|TH.| ]
    // Transceiver data continues for count times.
}  __attribute__((__packed__));

/*!
  @struct RUDP
  @brief RUDP-like block
  @details It does not meet all of the RUDP standards.
  It is a simplified implementation.
*/
struct RUDP
{
    ///@cond 0
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
    
    flag_t  flag{};     //!< @brief flag ==0:Unreliable !=0:Reliable flags
    uint8_t sequence{}; //!< @brief Lower 8bit of sequence
    uint8_t ack{};      //!< @brief Lower 8bit of ack
    uint8_t _sum{};     //!< @breif Check sum (Not supported)

}  __attribute__((__packed__));

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
    inline bool isReliable() const   { return rudp.flag; } //!< @brief is reliable data?
    inline bool isUnreliable() const { return !rudp.flag; } //!< @brief is unreliable data?
    inline bool isSYN() const        { return (rudp.flag == to_underlying(RUDP::Flag::SYN)); } //!< @brief is SYN?
    inline bool isSYN_ACK() const    { return (rudp.flag == to_underlying(RUDP::Flag::SYN_ACK)); } //!< @brief is SYN?
    inline bool isRST() const        { return (rudp.flag & to_underlying(RUDP::Flag::RST)); } //!< @brief is RST?
    inline bool isACK() const        { return (rudp.flag & to_underlying(RUDP::Flag::ACK)); } //!< @brief is ACK?
    inline bool onlyACK() const      { return (rudp.flag == to_underlying(RUDP::Flag::ACK)); } //!< @brief only ACK?
    inline bool isNUL() const        { return (rudp.flag == to_underlying(RUDP::Flag::NUL)); } //!< @brief is NUL?
    inline bool hasPayload() const   { return size > sizeof(*this); } //!< @brief Has payload?
    inline uint8_t payloadSize() const { return hasPayload() ? size - sizeof(*this) : 0; } //!< @brief payload size if exists
    ///@}
    inline uint8_t* payload() const  { return hasPayload() ? ((uint8_t*)this) + sizeof(*this) : nullptr; } //!< @brief Gets the payload pointer if exists.
}  __attribute__((__packed__));


/*!
  @enum Notify
  @brief Notify type
  notify and argument  Correspondence Chart for notify(), onNotify()
  | Notify | Argument |
  | --- | --- |
  | Disconnect | const MACAddress* |
  | ConnectionLost | const MACAddress* |  
*/
enum class Notify : uint8_t
{
    Disconnect,     //!< @brief Actively disconnected
    ConnectionLost, //!< @brief Connection lost
};

class Transceiver;

/*!
  @class Communicator
  @brief Manage transceivers
  @note Singleton
 */
class Communicator
{
  public:
    using notify_function = void(*)(const Notify notify, const void* arg);

    ///@name Default config values
    ///@{
    static constexpr uint16_t DEFAULT_RETRANSMISSION_TIMEOUT = (320);
    static constexpr uint16_t DEFAULT_CUMULATIVE_ACK_TIMEOUT = (80);
    static constexpr uint16_t DEFAULT_NULL_TIMEOUT = (1000*10);
    static constexpr uint16_t DEFAULT_TRANSFER_STATE_TIMEOUT = 0;
    static constexpr uint8_t DEFAULT_MAX_RETRANS = 8;
    static constexpr uint8_t DEFAULT_MAX_CUM_ACK = 4;
    static constexpr uint8_t DEFAULT_MAX_OUT_OF_SEQ = 0;
    static constexpr uint8_t DEFAULT_MAX_AUTO_RESET = 0;
    ///@}

    /*!
      @struct config_t
      @brief Compatible with SYN parameters
      @warning Note that not all are supported
     */
    struct config_t
    {
        //        uint8_t session{};     // Session id
        //        uint8_t outstanding{}; // 返答を待たずに送れるセグメント数(as window size)
        //
        //        uint8_t maximumSegmentSize{ESP_NOW_MAX_DATA_LEN}; //!< @brief The maximum number of octets that can be received by the peer
        //! @brief The timeout value for retransmission of unacknowledged packets
        uint16_t retransmissionTimeout{DEFAULT_RETRANSMISSION_TIMEOUT};
        //! @brief The timeout value for sending an acknowledgment segment if another segment is not sent
        uint16_t cumulativeAckTimeout{DEFAULT_CUMULATIVE_ACK_TIMEOUT};
        //! @brief The timeout value for sending a null segment if a data segment has not been sent
        uint16_t nullSegmentTimeout{DEFAULT_NULL_TIMEOUT};
        //! @brief This timeout value indicate the amount of time the state information will be  saved for a connection before an auto reset occurs.        
        uint16_t transferStateTimeout{DEFAULT_TRANSFER_STATE_TIMEOUT};
        //! @brief The maximum number of times consecutive retransmission(s)
        uint8_t maxRetrans{DEFAULT_MAX_RETRANS};
        //! @brief The maximum number of acknowledgments that will be accumulated before sending an acknowledgment
        uint8_t maxCumAck{DEFAULT_MAX_CUM_ACK};
        //! @brief he maximum number of out of sequence packets that will be accumulated before an EACK segment is sent
        uint8_t maxOutOfSeq{DEFAULT_MAX_OUT_OF_SEQ};
        //! @brief The maximum number of consecutive auto reset that will performed before a connection is reset
        uint8_t maxAutoReset{DEFAULT_MAX_AUTO_RESET};
    }  __attribute__((__packed__));

    /*!
      @enum Role
      @brief Role of communicator
     */
    enum class Role : uint8_t
    {
        NoRole,    //!< @brief No role
        Primary,   //!< @brief Primary. Also known as master, controller(ESP-NOW Terminology)
        Secondary, //!< @brief Secondary. Also known as slave
    };
    
    struct State
    {
        enum Status : uint8_t { None, Succeed, Failed };
        Status state{}; // status
        uint8_t retry{}; // retry counter
        unsigned long sentTime{}; // sent time
        unsigned long recvTime{}; // recv time
        inline void reset() { state = None; }

    } __attribute__((__packed__));

#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
    using queue_map_t = std::map<MACAddress /* Target (NullAddress is all peers) */, std::vector<uint8_t>>;
    using state_map_t = std::map<MACAddress, State>;
#else
    using queue_map_t = vmap<MACAddress, std::vector<uint8_t>>;
    using state_map_t = vmap<MACAddress, State>;
#endif
    
    static Communicator& instance();
    
    ///@cond 0
    Communicator(const Communicator&) = delete;
    Communicator& operator=(const Communicator&) = delete;
    /// @endcond

    ///@name Properties
    ///@{
    inline const MACAddress& address() const  { return _addr; } //!< @brief Gets the self address
    inline unsigned long lastSentTime() const { return _lastSentTime; } //!< @brief Gets the last sent time
    inline Role role() const        { return _role; } //!< @brief Gets the role type
    inline bool isPrimary() const   { return _role == Role::Primary; } //!< @brief Is role primary?
    inline bool isSecondary() const { return _role == Role::Secondary; } //<! @brief Is role secondary?
    inline bool isNoRole() const   { return _role == Role::NoRole; } //!< @brief No role?
    inline config_t config() const { return _config; } //!< @brief Gets the configuretion
    ///@}

    void setRole(const Role r) { _role = r; }
    
    /*!
      @brief Begin communication
      @param app_id Unique value for each application
    */
    inline bool begin(const uint8_t app_id) { return begin(app_id, config_t{}); }
    /*!
      @brief Begin communication
      @param app_id Unique value for each application
      @param cfg Configuretion
    */
    bool begin(const uint8_t app_id, const config_t& cfg);
    void end();
    /*!
      @brief Update comminicator
      @note Transmission of posted data is done here
      @note Registered transceivers are also updated here
     */
    void update();

    ///@name Transceiver
    ///@{
    Transceiver* transceiver(const uint8_t tid); //!< @brief Gets the transceiver
    bool registerTransceiver(Transceiver* t); //!< @brief Register the transceiver
    bool unregisterTransceiver(Transceiver* t); //!< @brief Unregister the transceiver
    size_t numOfTransceivers() const { return _transceivers.size(); } //!< @brief Gets the number of registered transceivers
    ///@}

    ///@name Peer
    ///@warning Peer status is shared with ESP-NOW
    ///@{
    bool registerPeer(const MACAddress& addr, const uint8_t channel = 0, const bool encrypt = false, const uint8_t* lmk = nullptr); //!< @brief Register the peer
    void unregisterPeer(const MACAddress& addr); //!< @brief Unregister the peer

    /*!
      @brief Clear all peer
      @warning Cannot clear multicast address. Explicitly call unregisterPeer if you want to erase it.
    */
    void clearPeer();
    uint8_t numOfPeer(); //!< @brief Number of registered peers
    bool existsPeer(const MACAddress& addr); //!< @brief Exists peer?
    ///@}

    /*!
      @brief Post data
      @param peer_addr MAC address (send all unicast peer if nullptr)
      @param data data pointer
      @param length Length of data
      @note Only stores data, actual transmission is done by update()
     */
    bool post(const uint8_t* peer_addr, const void* data, const uint8_t length);
    //! @brief Post data with lock
    bool postWithLock(const uint8_t* peer_addr, const void* data, const uint8_t length)
    {
        lock_guard _(_sem);
        return post(peer_addr, data, length);
    }
    /*!
      @brief Send transceiver data
      @param peer_addr MAC address (send all unicast peer if nullptr)
      @param data data pointer
      @param length Length of data
     */
    bool send(const uint8_t* peer_addr, const void* data, const uint8_t length);
    //! @brief Send data with lock
    bool sendWithLock(const uint8_t* peer_addr, const void* data, const uint8_t length)
    {
        lock_guard _(_sem);
        return send(peer_addr, data, length);
    }

    /*!
      @brief Call any function with lock
      @param Func Any functor
      @param args Arguments for functor
      @warning The same binary semaphore is used inside the communicator, so beware of deadlocks.
    */
    template<typename Func, typename... Args> auto with_lock(Func func, Args&&... args) const
            -> decltype(func(std::forward<Args>(args)...))
    {
        lock_guard lock(_sem);
        return func(std::forward<Args>(args)...);
    }

    /// @name Notification
    /// @{
    //! @brief Register function
    inline void registerNotifyFunction(notify_function f) { lock_guard _(_sem); _notifyFunction = f; }
    //! @brief Unregister fucntion
    inline void unregisterNotifyFunction() { lock_guard _(_sem); _notifyFunction = nullptr; }
    /*!
      @brief Notification
      @note Call your notify function if registered
     */
    void notify(const Notify n, const void* arg = nullptr);
    ///@}
    
#if !defined(NDEBUG) || defined(DOXYGEN_PROCESS)
    ///@name Debugging features
    ///@warning Conditions of use, <em><strong>NDEBUG must NOT be DEFINED.</strong></em>
    ///@{
    String debugInfo() const; //!< @brief Gets the information string with lock
    virtual String debug_info() const; //!< @brief Gets the information string
    bool isEnableDebug() const     { return _debugEnable; } //!< @brief Are debugging features enabled?
    void enableDebug(const bool b) { _debugEnable = b;    } //!< @brief Enable/Disable debugging features
    ///@}

    ///@name Debugging features for Simulate communication failures
    ///@{
    float debugSendLoss() const    { return _debugSendLoss; } //!< @brief Gets the sending losee
    float debugRecieveLoss() const { return _debugRecvLoss; } //!< @brief Gets the receiving losee
    void setDebugSendLoss(const float rate)    { _debugSendLoss = rate; } //!< @brief Set sending loss
    void setDebugReceiveLoss(const float rate) { _debugRecvLoss = rate; } //!< @brief Set receiving loss
    void setDebugLoss(const float rate)        { _debugSendLoss = _debugRecvLoss = rate; } //!< @brief Set sending and receiving loss percentage
    ///@}
#endif
    
  protected:
    Communicator();
    
    ///@name Callback
    ///@note Called from WiFi-task.
    /// @sa https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
    ///@{
    static void callback_onSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    void onSent(const MACAddress& addr, const esp_now_send_status_t status);
    static void callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length);
    void onReceive(const MACAddress& addr, const uint8_t* data, const uint8_t length);
    ///@}

    bool send_esp_now(const uint8_t* peer_addr, /* DON'T const!! Calls td::move in funciton */std::vector<uint8_t>& packet);
    //    void append_to_sent(const uint8_t* peer_addr, std::vector<uint8_t>& packet);    
    bool remove_acked(const MACAddress& addr, std::vector<uint8_t>& packet);

    void reset_sent_state(const MACAddress& addr);

  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore

    uint8_t _app_id{};// Application-specific ID
    bool _began{};
    volatile bool _canSend{true};
    Role _role{Role::NoRole};

    MACAddress _addr{}; // Self address
    config_t _config{};

    Transceiver* _sysTransceiver{}; // System transceiver
    std::vector<Transceiver*> _transceivers; // User transceiver

    unsigned long _lastSentTime{};
    MACAddress _lastSentAddr{BROADCAST};
    queue_map_t _queue;
    state_map_t _state;

    notify_function _notifyFunction{};

    
    
#if !defined(NDEBUG)
    bool _debugEnable{};
    float _debugSendLoss{}, _debugRecvLoss{};
#endif
};

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

    ///@cond 0
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
    template<typename T> inline uint64_t postReliable(const MACAddress& addr, const T& data) { return postReliable(addr.data(), &data ,sizeof(data)); }
    //! @brief Post to all peer
    template<typename T> inline uint64_t postReliable(                        const T& data) { return postReliable(nullptr,     &data ,sizeof(data)); }

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
    template<typename T> inline bool sendUnreliable(const MACAddress& addr, const T& data) { return sendUnreliable(addr.data(), &data ,sizeof(data)); }
    template<typename T> inline bool sendUnreliable(                        const T& data) { return sendUnreliable(nullptr,     &data ,sizeof(data)); }
    ///@}    
   
    /*!
      @brief Call any function with lock
      @param Func Any functor
      @param args Arguments for functor
      @warning The same binary semaphore is used inside the transceiver, so beware of deadlocks.
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
      @warning The receiving callback function also runs from the Wi-Fi task,
      @warning <em><strong>So, do not do lengthy operations in the callback function.</strong></em>
    */
    virtual void onReceive(const MACAddress& addr, const void* data, const uint8_t length) {}
    //! @brief Notification callback
    virtual void onNotify(const Notify notify, const void* arg) {}
    
    void build_peer_map();
    // WARN:Locked in this >>
    bool post_rudp(const uint8_t* peer_addr, const RUDP::Flag flag, const void* data = nullptr, const uint8_t length = 0);
    inline bool post_ack(const uint8_t* peer_addr) { return post_rudp(peer_addr, RUDP::Flag::ACK); }
    inline bool post_ack(const MACAddress& addr)   { return post_ack(static_cast<const uint8_t*>(addr)); }
    inline bool post_nul(const uint8_t* peer_addr) { return post_rudp(peer_addr, RUDP::Flag::NUL); }
    inline bool post_nul(const MACAddress& addr)   { return post_nul(static_cast<const uint8_t*>(addr)); }
    // <<
    
    uint64_t make_data(uint8_t* buf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);

    inline bool delivered(const uint8_t seq, const MACAddress& addr)
    {
        return delivered(restore_u64_earlier(_peerInfo[addr].sequence, seq), addr);
    }
    
    struct PeerInfo
    {
        // ACK with payload
        uint64_t sequence{};  // S
        uint64_t recvSeq{};   // R (Sequence number received from the peer)
        uint64_t recvAck{};   // R (ACK number received from the peer)
        uint64_t sentAck{};   // Last ACK sent
        // no payload
        uint64_t ackSequence{}; // S
        uint64_t recvAckSeq{}; // R
        //
        unsigned long recvTime{}; // R
        bool needReturnACK{}; // R
    } __attribute__((__packed__));
#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
    using info_map_t = std::map<MACAddress, PeerInfo>;
#else
    using info_map_t = vmap<MACAddress, PeerInfo>;
#endif
    
  private:
    Transceiver(); // System transceiver for communicator
    void _update(const unsigned long ms, const Communicator::config_t& cfg);
    void on_receive(const MACAddress& addr, const TransceiverHeader* th);
    
  private:
    const uint8_t _tid{};    // Transceiver unique identifier (0 reserved)
    info_map_t _peerInfo;    // peer information for RUDP (ACK with payload)
    
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore
    friend class Communicator;
};
//
}}
#endif
