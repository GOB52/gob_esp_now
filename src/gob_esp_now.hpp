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

/*!
  @namespace esp_now
  @brief For ESP-NOW
 */
namespace esp_now {

constexpr char LIB_TAG[] = "gen"; //!< @brief Tag for logging

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
    
    // Flag-specific data
    struct Payload_SYN
    {
        uint8_t session{};     // Session id
        uint8_t outstanding{}; // 返答を待たずに遅れるセグメント数
        uint8_t retransmissionTimeout{}; // 送ったデータに対して指定時間返答がなかったら再送信
        uint8_t cumulativeAckTimeout{};  // セグメントを受け取ってから、この時間送信すべきものがなければACKのみ返す
        uint8_t nullSegmentTimeout{};    // NUL 送信間隔
        uint8_t transferStateTimeout{};  // 通信不能になってからこの時間のうちに TCS が来たら再開
        uint8_t maxRetrans{};            // 最大再送回数(超えたら通信不能)
        uint8_t maxCumAck{}; // 受け取ったがACKしていないセグメント数(超えたらACKのみ返す)
        uint8_t maxOutOfSeq{}; // 来ていない手前のセグメントがあって、ACK だせないセグメントがこの数を超えたら EAK 送信
        uint8_t maxAutoReset{}; // 通信不能になってから自動で 3WH 再試行していい回数
    }  __attribute__((__packed__));
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

    inline uint8_t* payload() const { return ((uint8_t*)this) + sizeof(*this); } //!< @brief Gets the payload pointer
    inline bool hasPayload() const { return size > sizeof(*this); } //!< @brief Has payload?
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
    static Communicator& instance();
    
    ///@cond 0
    Communicator(const Communicator&) = delete;
    Communicator& operator=(const Communicator&) = delete;
    /// @endcond

    const MACAddress& address() const { return _addr; } //!< @brief Gets the self address
    //! Gets the threshold for loss of connection (ms)
    unsigned long lossOfConnectionTime() const { return _locTime; } 
    //! Set threshold for loss of connection (ms)
    void setLossOfConnectionTime(const unsigned long ms) { _locTime = ms; }
    
    /*!
      @brief Begin communication
      @param app_id Unique value for each application
    */
    bool begin(const uint8_t app_id);
    void end();
    /*!
      @brief Update comminicator
      @note Transmission of posted data is done here
      @note Registered transceivers are also updated here
     */
    void update();

    ///@name Transceiver
    ///@{
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

    static constexpr unsigned long LOSS_OF_CONNECTION_TIME =  4000; //!< @brief Threshold for loss of connection (ms)

    ///@name Callback
    ///@note Called from WiFi-task.
    /// @sa https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
    ///@{
    static void callback_onSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    void onSent(const MACAddress& addr, const esp_now_send_status_t status);
    static void callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length);
    void onReceive(const MACAddress& addr, const uint8_t* data, const uint8_t length);
    ///@}

    bool send_esp_now(const uint8_t* peer_addr, /* DON'T const!! Calls td::move in funciton */std::vector<uint8_t>& vec);
    void move_to_last(const uint8_t* peer_addr, std::vector<uint8_t>& vec);    

  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore

    uint8_t _app_id{};// Application-specific ID
    bool _began{};
    volatile bool _canSend{true};
    volatile bool _retry{};
    unsigned long _sentTime{}, _locTime{LOSS_OF_CONNECTION_TIME};

    MACAddress _addr{}; // Self address
    std::vector<Transceiver*> _transceivers;

    MACAddress _lastAddr{};
    std::vector<uint8_t> _lastData{};

#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
    using queue_map_t = std::map<MACAddress, std::vector<uint8_t>>;
#else
    using queue_map_t = vmap<MACAddress, std::vector<uint8_t>>;
#endif
    queue_map_t _queue;

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
    inline uint64_t sequence(const MACAddress& addr) const { return (_recvSeq.count(addr) == 1) ? _recvSeq.at(addr) : 0ULL; } //!< @brief Gets the received sequence No,
    inline uint64_t ack(const MACAddress& addr) const { return (_recvAck.count(addr) == 1) ? _recvAck.at(addr) : 0ULL; } //!< @brief Gets the received ACK No,
    //! @brief Was the specified sequence received by all peers?
    bool received(const uint64_t seq)
    {
        return std::all_of(_recvAck.begin(), _recvAck.end(), [&seq](decltype(_recvAck)::const_reference a)
        {
            return seq <= a.second || !(a.first) || a.first.isMulticast(); // Null MAC and multicast are considered true
        });
    }
    //! @brief Was the specified sequence received at the specified peer?
    bool received(const uint64_t seq, const MACAddress& addr)
    {
        return seq <= _recvAck[addr];
    }
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
          int arg;
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
    void build_peer_map();
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

    uint64_t make_data(uint8_t* buf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);
    void on_receive(const MACAddress& addr, const TransceiverHeader* data);
    void on_notify(const Notify notify, const void* arg);
    

#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
    using seq_map_t = std::map<MACAddress, uint64_t>;
#else
    using seq_map_t = vmap<MACAddress, uint64_t>;
#endif
    const seq_map_t& sequences() const { return _recvSeq; }
    const seq_map_t& acks() const { return _recvAck; }
    
  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore
    const uint8_t _tid{}; // Transceiver unique identifier
    uint64_t _sequence{}; // send sequence
    seq_map_t _recvSeq; // received sequence no
    seq_map_t _recvAck; // received ack no
    
    friend class Communicator;
};
//
}}
#endif
