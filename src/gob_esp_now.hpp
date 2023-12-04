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
    return (u64 & ~static_cast<uint64_t>(0xff)) | u8;
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
    static constexpr uint16_t SIGNETURE = 0x454e; //!< @brief Packet signeture "GE"
    static constexpr uint8_t VERSION = 0x00;      //!< @brief Header version

    uint16_t signeture{SIGNETURE}; //!< @brief Signeture of the Communicator's data
    uint8_t  version{VERSION};     //!< @brief Version of the header
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
    //uint8_t _sum{};

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
    inline bool isReliable() const { return rudp.flag; } //!< @brief is reliable data?
    inline bool isUnreliable() const { return !rudp.flag; } //!< @brief is unreliable data?
    inline bool isSYN() const { return (rudp.flag & to_underlying(RUDP::Flag::SYN)); } //!< @brief is SYN?
    inline bool isRST() const { return (rudp.flag & to_underlying(RUDP::Flag::RST)); } //!< @brief is RST?
    inline bool isACK() const { return (rudp.flag & to_underlying(RUDP::Flag::ACK)); } //!< @brief is ACK?
    inline bool hasPayload() const { return size > sizeof(*this); } //!< @brief Has payload?
    ///@}
    inline uint8_t* payload() const { return ((uint8_t*)this) + sizeof(*this); } //!< @brief Gets the payload pointer
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
    ///@name Default config values
    ///@{
    static constexpr uint16_t DEFAULT_RETRANSMISSION_TIMEOUT = (200);
    static constexpr uint16_t DEFAULT_CUMULATIVE_ACK_TIMEOUT = (100);
    static constexpr uint16_t DEFAULT_NULL_TIMEOUT = (1000*5);
    static constexpr uint16_t DEFAULT_TRANSFER_STATE_TIMEOUT = (1000*10);
    static constexpr uint8_t DEFAULT_MAX_RETRANS = 16;
    static constexpr uint8_t DEFAULT_MAX_CUM_ACK = 3;
    static constexpr uint8_t DEFAULT_MAX_OUT_OF_SEQ = 5;
    static constexpr uint8_t DEFAULT_MAX_AUTO_RESET = 1;
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
        uint16_t retransmissionTimeout;
        //! @brief The timeout value for sending an acknowledgment segment if another segment is not sent
        uint16_t cumulativeAckTimeout;
        //! @brief The timeout value for sending a null segment if a data segment has not been sent
        uint16_t nullSegmentTimeout;
        //! @brief This timeout value indicate the amount of time the state information will be  saved for a connection before an auto reset occurs.        
        uint16_t transferStateTimeout;
        //! @brief The maximum number of times consecutive retransmission(s)
        uint8_t maxRetrans;
        //! @brief The maximum number of acknowledgments that will be accumulated before sending an acknowledgment
        uint8_t maxCumAck;
        //! @brief he maximum number of out of sequence packets that will be accumulated before an EACK segment is sent
        uint8_t maxOutOfSeq;
        //! @brief The maximum number of consecutive auto reset that will performed before a connection is reset
        uint8_t maxAutoReset;

        static config_t defaultValue()
        {
            config_t cfg =
            {
                DEFAULT_RETRANSMISSION_TIMEOUT,
                DEFAULT_CUMULATIVE_ACK_TIMEOUT,
                DEFAULT_NULL_TIMEOUT,
                DEFAULT_TRANSFER_STATE_TIMEOUT,
                DEFAULT_MAX_RETRANS,
                DEFAULT_MAX_CUM_ACK,
                DEFAULT_MAX_OUT_OF_SEQ,
                DEFAULT_MAX_AUTO_RESET
            };
            return cfg;
        }
    }  __attribute__((__packed__));

    static Communicator& instance();
    
    ///@cond 0
    Communicator(const Communicator&) = delete;
    Communicator& operator=(const Communicator&) = delete;
    /// @endcond


    ///@name Properties
    ///@{
    const MACAddress& address() const { return _addr; } //!< @brief Gets the self address
    // Gets the threshold for loss of connection (ms)
    //    unsigned long lossOfConnectionTime() const { return _locTime; } 
    // Set threshold for loss of connection (ms)
    //    void setLossOfConnectionTime(const unsigned long ms) { _locTime = ms; }

    unsigned long lastSentTime() const { return _lastSentTime; } //!< @brief Gets the last sent time
    ///@}
    
    /*!
      @brief Begin communication
      @param app_id Unique value for each application
    */
    bool begin(const uint8_t app_id, const config_t* cfg = nullptr);
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
    bool registerTransceiver(Transceiver* t); //!< @brief Register
    bool unregisterTransceiver(Transceiver* t); //!< @brief Unregister
    size_t numOfTransceivers() const { return _transceivers.size(); } //!< @brief Gets the number of registered transceivers
    ///@}

    ///@name Peer
    ///@warning Peer status is shared with ESP-NOW
    ///@{
    bool registerPeer(const MACAddress& addr, const uint8_t channel = 0, const bool encrypt = false, const uint8_t* lmk = nullptr); //!< @brief Register peer
    void unregisterPeer(const MACAddress& addr); //!< @brief Unregister peer

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
    bool post_with_lock(const uint8_t* peer_addr, const void* data, const uint8_t length)
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
    bool send_with_lock(const uint8_t* peer_addr, const void* data, const uint8_t length)
    {
        lock_guard _(_sem);
        return send(peer_addr, data, length);
    }

    /*!
      @brief Notify transceivers
      @warning This function locks the transceiver internally, so beware of deadlocks
     */
    void notify(const Notify notify, const void* arg = nullptr);

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
    
#if !defined(NDEBUG) || defined(DOXYGEN_PROCESS)
    ///@name Debugging features
    ///@warning Conditions of use, <em><strong>NDEBUG must NOT be DEFINED.</strong></em>
    ///@{
    virtual String debugInfo() const; //!< @brief Gets the information string
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

    //static constexpr unsigned long LOSS_OF_CONNECTION_TIME =  4000; //!< @brief Threshold for loss of connection (ms)
    static constexpr unsigned long DEFAULT_RESEND_INTERVAL = 1000 * 3; //!< @brief Resend interval (ms)
    
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
    
  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore

    uint8_t _app_id{};// Application-specific ID
    bool _began{};
    volatile bool _canSend{true};

    MACAddress _addr{}; // Self address
    config_t _config{};

    std::vector<Transceiver*> _transceivers;

    unsigned long _lastSentTime{};

    // 0:none 1:succeed 2:failed
    struct SendState
    {
        enum State : uint8_t { None, Succeed, Failed};
        State state{};
        uint8_t retry{};
        unsigned long sentTime{};
    } __attribute__((__packed__));

#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
    using sent_map_t = std::map<MACAddress, SendState>;
    using queue_map_t = std::map<MACAddress, std::vector<uint8_t>>;
#else
    using queue_map_t = vmap<MACAddress, std::vector<uint8_t>>;
    using sent_map_t = vmap<MACAddress, SendState>;
#endif
    queue_map_t _queue;
    //    queue_map_t _sentQueue;
    sent_map_t _sentState;
    
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
      @param tid Unique value for each transceiver
      @note id also serves as priority (in ascending order)
     */
    explicit Transceiver(const uint8_t tid);
    virtual ~Transceiver();

    ///@cond 0
    Transceiver(const Transceiver&) = delete;
    Transceiver& operator=(const Transceiver&) = delete;
    ///@endcond

    ///@name Properties
    ///@{
    int8_t identifier() const { return _tid; } //!< @brief Gets the identifier
    inline uint64_t sequence() const { return _sequence; } //!< @brief Gets the my sequence No.
    inline uint64_t sequence(const MACAddress& addr) const { return (_peerInfo.count(addr) == 1) ? _peerInfo.at(addr).sequence : 0ULL; } //!< @brief Gets the received sequence No,
    inline uint64_t ack(const MACAddress& addr) const { return (_peerInfo.count(addr) == 1) ? _peerInfo.at(addr).ack : 0ULL; } //!< @brief Gets the received ACK No,
    inline unsigned long ackTime(const MACAddress& addr) const { return (_peerInfo.count(addr) == 1) ? _peerInfo.at(addr).time : 0; } //!< @brief Gets the latest time of received ACK
    //! @brief Was the specified sequence received by all peers?
    inline bool peerReceived(const uint64_t seq)
    {
        return std::all_of(_peerInfo.begin(), _peerInfo.end(), [&seq](decltype(_peerInfo)::const_reference a)
        {
            return seq <= a.second.ack || !(a.first) || a.first.isMulticast(); // Null MAC and multicast are considered true
        });
    }
    //! @brief Was the specified sequence received by all peers?
    inline bool peerReceived(const uint8_t seq) { return peerReceived(restore_u64_earlier(_sequence, seq)); }

    //! @brief Was the specified sequence received at the specified peer?
    inline bool peerReceived(const uint64_t seq, const MACAddress& addr) { return seq <= _peerInfo[addr].ack; }
    //! @brief Was the specified sequence received at the specified peer?
    inline bool peerReceived(const uint8_t seq, const MACAddress& addr) { return peerReceived(restore_u64_earlier(_sequence, seq), addr); }
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
      @note In the case of Post, data are combined.
      @note To reduce the number of transmissions when sending multiple data or transceivers.
     */
    bool postReliable(const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);
    //! @brief Post to all peer
    inline bool postReliable(                   const void* data = nullptr, const uint8_t length = 0) { return postReliable(nullptr, data, length); }
    //! @brief Post to destination
    template<typename T> inline bool postReliable(const MACAddress& addr, const T& data) { return postReliable(addr.data(), &data ,sizeof(data)); }
    //! @brief Post to all peer
    template<typename T> inline bool postReliable(                        const T& data) { return postReliable(nullptr,     &data ,sizeof(data)); }

    /*!
      @brief Send data
      @param peer_addr Destination address. Post to all peer if nullptr
      @param data Payload data if exists
      @param length Length of the payload data if exists
      @warning ESP-NOW does not allow frequent calls to be sent without a send callback being received,
      @warning as such calls will result in an error.In that case, the library will return failure.
      @warning <em>Unlike post, if it fails, it is not automatically resubmitted by update,
      @warning so you have to manage resubmission yourself.</em>
     */
    bool sendReliable(const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);
    //! @brief Send to all peer
    inline bool sendReliable(                   const void* data = nullptr, const uint8_t length = 0) { return sendReliable(nullptr, data, length); }
    //! @brief Send to destination
    template<typename T> inline bool sendReliable(const MACAddress& addr, const T& data) { return sendReliable(addr.data(), &data ,sizeof(data)); }
    //! @brief Send to all peer
    template<typename T> inline bool sendReliable(                        const T& data) { return sendReliable(nullptr,     &data ,sizeof(data)); }
    ///@}

    ///@name Data transmission (Unreliable)
    ///@note Parameters are the same as post/sendReliable
    ///@{
    bool postUnreliable(const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);    
    inline bool postUnreliable(                   const void* data = nullptr, const uint8_t length = 0) { return postUnreliable(nullptr, data, length); }
    template<typename T> inline bool postUnreliable(const MACAddress& addr, const T& data) { return postUnreliable(addr.data(), &data ,sizeof(data)); }
    template<typename T> inline bool postUnreliable(                        const T& data) { return postUnreliable(nullptr,     &data ,sizeof(data)); }
    bool sendUnreliable(const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);
    inline bool sendUnreliable(                   const void* data = nullptr, const uint8_t length = 0) { return sendUnreliable(nullptr, data, length); }
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
      @brief Receiving callbacks
      @warning The receiving callback function also runs from the Wi-Fi task,
      @warning <em><strong>So, do not do lengthy operations in the callback function.</strong></em>
    */
    virtual void onReceive(const MACAddress& /*addr*/, const TransceiverHeader* /*data*/) {}
    //! @brief Notification callbacks
    virtual void onNotify(const Notify /*notify*/, const void* /*arg*/) { /* nop */ }

    void build_peer_map();
    bool post_ack(const uint8_t* peer_addr);
    inline bool post_ack(const MACAddress& addr) { return post_ack((bool)addr ? addr.data() : nullptr); }
    uint64_t make_data(uint8_t* buf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);

    /*!
      @struct Recv
      @brief received information
     */
    struct Recv
    {
        uint64_t sequence;  // Sent by peer sequenceNo (It will be set to rudp.ack when send)
        uint64_t ack;       // Sequence received by the peer
        unsigned long time; // Time ACK received
    };

#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
    using info_map_t = std::map<MACAddress, Recv>;
#else
    using info_map_t = vmap<MACAddress, Recv>;
#endif
    const info_map_t& peerInfo() const { return _peerInfo; }
    
  private:
    void _update(const unsigned long ms, const unsigned long lastSentTime, const uint16_t cumulativeAckTimeout, const uint8_t maxCumAck);
    void on_receive(const MACAddress& addr, const TransceiverHeader* th);
    void on_notify(const Notify notify, const void* arg);
    
  private:
    const uint8_t _tid{}; // Transceiver unique identifier
    unsigned long _sentTime{}, _emptyAckSendInterval{5000};
    uint64_t _sequence{}; // Send sequence
    info_map_t _peerInfo; // peerr information for RUDP
    
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore
    friend class Communicator;
};
//
}}
#endif
