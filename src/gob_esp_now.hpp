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
#include <map>
#include <FreeRTOS/freeRTOS.h>
#include <FreeRTOS/semphr.h>
#include <esp_now.h>
#include <WString.h>
#include "gob_mac_address.hpp"

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

constexpr const char LIB_TAG[] = "GEN"; //!< @brief Tag for logging

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

    uint16_t signeture{SIGNETURE}; //!< @brief  Signeture of the Communicator's data
    uint8_t  version{VERSION};     //!< @brief  Version of the header
    uint8_t  app_id{};             //!< @brief  Application-specific ID
    uint8_t  count{};              //!< @brief  Number of transceiver data
    uint8_t  reserved{};
    // Transceiver data continues for count times.
}  __attribute__((__packed__));


/*!
  @struct RUDP
  @brief RUDP block
*/
struct RUDP
{
    ///@cond 0
    static constexpr uint8_t _SYN = 0x80; // Begin session
    static constexpr uint8_t _ACK = 0x40; // Acknowledge
    static constexpr uint8_t _EAK = 0x20; // Selective ACK
    static constexpr uint8_t _RST = 0x10; // End session
    static constexpr uint8_t _NUL = 0x08; // Heartbeat
    static constexpr uint8_t _CHK = 0x04; // Calculate CRC include payload if ON;
    static constexpr uint8_t _TCS = 0x02; // Demand for resumption
    ///@endcond
    
    /*!
      @enum Flag
      @brief Flags of RUDP
     */
    enum class Flag : uint8_t
     {
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
    // SYN
    struct Syn
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
*/
enum Notify : uint8_t
{
    Disconnect,  //!< @brief Peer disconnection @note arg MACAddress*
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
    unsigned long lossOfConnectionTime() const { return _lossOfConnectionTime; } 
    //! Set threshold for loss of connection (ms)
    void setLossOfConnectionTime(const unsigned long ms) { _lossOfConnectionTime = ms; }
    
    /*!
      @brief Begin communication
      @param app_id Unique value for each application
    */
    bool begin(const uint8_t app_id);
    void end();
    /*! @brief Update comminication
      @note Transmission of posted data is done here
     */
    void update();

    ///@name Transceiver
    ///@{
    bool registerTransceiver(Transceiver* t); //!< @brief Register
    bool unregisterTransceiver(Transceiver* t); //!< @brief Unregister
    size_t numOfTransceivers() const { return _transceivers.size(); } //!< @brief Gets the number of registered transceivers
    ///@}

    /// @name Peer
    /// @{
    static bool registerPeer(const MACAddress& addr, const uint8_t channel = 0, const bool encrypt = false, const uint8_t* lmk = nullptr); //!< @brief Register peer
    static void unregisterPeer(const MACAddress& addr); //!< @brief Unregister peer
    static void clearPeer();                            //!< @brief Clear all peer
    static int  numOfPeer();                            //!< @brief Number of registered peers
    static bool existsPeer(const MACAddress& addr);     //!< @brief Exists peer?
    /// @}

    /*!
      @brief Post data
      @param peer_addr MAC address (send all unicast peer if nullptr)
      @param data data pointer
      @param length Length of data
      @note Only stores data, actual transmission is done by update()
     */
    bool post(const uint8_t* peer_addr, const void* data, const uint8_t length);
    /*!
      @brief Send transceiver data
      @param peer_addr MAC address (send all unicast peer if nullptr)
      @param data data pointer
      @param length Length of data
     */
    bool send(const uint8_t* peer_addr, const void* data, const uint8_t length);
    
#if !defined(NDEBUG) || defined(DOXYGEN_PROCESS)
    ///@name Debugging features
    ///@note Create a pseudo send/receive loss condition
    ///@warning Conditions of use, <em><strong>NDEBUG must NOT be DEFINED.</strong></em>
    ///@{
    String debugInfo() const;                               //!< @brief Gets the information string
    bool isEnableDebug() const     { return _debugEnable; } //!< @brief Are debugging features enabled?
    void enableDebug(const bool b) { _debugEnable = b;    } //!< @brief Enable/Disable debugging features
    float debugSendLoss() const    { return _debugSendLoss; } //!< @brief Gets the sending losee
    float debugRecieveLoss() const { return _debugRecvLoss; } //!< @brief Gets the receiving losee
    void setDebugSendLoss(const float rate)    { _debugSendLoss = rate; } //!< @brief Set sending loss
    void setDebugReceiveLoss(const float rate) { _debugRecvLoss = rate; } //!< @brief Set receiving loss
    void setDebugLoss(const float rate)        { _debugSendLoss = _debugRecvLoss = rate; } //!< @brief Set sending and receiving loss percentage
    //Caused once in a specified number of times
    ///@}
#endif
    
  protected:
    Communicator();

    static constexpr unsigned long LOSS_OF_CONNECTION_TIME =  4000; //!< @brief Threshold for loss of connection (ms)

    ///@name Callback
    ///@note Called from WiFi-task.
    ///@note [Send] Too short interval between sending two ESP-NOW data may lead to disorder of sending callback function.
    /// So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending has returned.
    ///@note [Recv] The receiving callback function also runs from the Wi-Fi task. So, do not do lengthy operations in the callback function.
    /// Instead, post the necessary data to a queue and handle it from a lower priority task.
    /// @sa https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#send-esp-now-data
    ///@{
    static void callback_onSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    void onSent(const MACAddress& addr, const esp_now_send_status_t status);
    static void callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length);
    void onReceive(const MACAddress& addr, const uint8_t* data, const uint8_t length);
    ///@}

    bool send_esp_now(const uint8_t* peer_addr, /* DON'T const!! Calls td::move in funciton */std::vector<uint8_t>& vec);
    
  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore

    uint8_t _app_id{};// Application-specific ID
    bool _began{};
    volatile bool _canSend{};
    volatile bool _retry{};
    unsigned long _sendTime{}, _lossOfConnectionTime{LOSS_OF_CONNECTION_TIME};

    MACAddress _addr{}; // Self address
    std::vector<Transceiver*> _transceivers;

    MACAddress _lastAddr{};
    std::vector<uint8_t> _lastData{};
    std::map<MACAddress, std::vector<uint8_t>> _queue;

#if !defined(NDEBUG)
    bool _debugEnable{};
    float _debugSendLoss{}, _debugRecvLoss{};
#endif
};

/*
TODO:
  Transceiver::update  必要
* RUDP 受け取って、送るものがない場合に一定期間で ACK のみ返したいので
* HeartbeatTransceiver 作って一定タイミングで勝手にハートビートしたい
_ack[addr] は seq が飛んだ場合の対処する


*/

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

    Transceiver(const Transceiver&) = delete;
    Transceiver& operator=(const Transceiver&) = delete;

    ///@name Properties
    ///@{
    int8_t identifier() const { return _tid; } //!< @brief Gets the identifier
    inline uint64_t sequence() const { lock_guard _(_sem);  return _sequence; } //!< @brief Gets the sequence No.
    inline uint64_t ack(const MACAddress& addr) const { lock_guard _(_sem); return (_ack.count(addr) == 1) ? _ack.at(addr) : 0ULL; } //!< @brief Gets the ack No
    ///@}

    ///@name Data transmission
    ///@{
    /*!
      @brief Post data
      @param peer_addr Destination address. Post to all peer if nullptr
      @param data Payload data if exists
      @param length Length of the payload data if exists
      @note In the case of Post, data are combined.
      @note To reduce the number of transmissions when sending multiple data or transceivers.
      @warnig 
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

    /*!
      @brief Receive data
      @note Called from WiFi task, so be careful of exclusivity control, etc.
      @sa with_lock
    */
    virtual void onReceive(const MACAddress& /*addr*/, const TransceiverHeader* /*data*/) {}
    //! @brief Notifications from Communicator
    virtual void onNotify(const Notify /*notify*/, void* arg = nullptr) { /* nop */ }
    
    /*!
      @brief Call any function with lock
      @param Func Any functor
      @param args Arguments for functor
      @details Example
      @code{.cpp}
      class YourTransceiver : public Transceiver
      {
          struct Payload { int value; };
          explicit YourTransceiver(const uint8_t tid) : Transceiver(tid) {}         
          virtual void onReceive(const MACAddress& addr, const TransceiverHeader* data) override
          {
              with_lock([&]() { _value = ((PayLoad*)data->payload())->value; });
          }
          volatile int _value{};
      };
      YourTransceiver yt(1);
      YourTransceiver::Payload pl = { 52; }
      yt.send(pl);
      @endcode
     */
    template<typename Func, typename... Args> auto with_lock(Func func, Args&&... args) const
            -> decltype(func(std::forward<Args>(args)...))
    {
        lock_guard lock(_sem);
        return func(std::forward<Args>(args)...);
    }

#if !defined(NDEBUG) || defined(DOXYGEN_PROCESS)
    ///@name Debugging features
    ///@note Create a pseudo send/receive loss condition
    ///@warning Conditions of use, <em><strong>NDEBUG must NOT be DEFINED.</strong></em>
    ///@{
    String debugInfo() const;                               //!< @brief Gets the information string
    ///@}
#endif    

  protected:
    virtual void update() {}

    uint8_t* make_data(uint8_t* buf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data, const uint8_t length);
    void on_receive(const MACAddress& addr, const TransceiverHeader* data);
    
  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore
    uint8_t _tid{}; // Transceiver unique identifier
    uint64_t _sequence{}; // send sequence
    std::map<MACAddress, uint64_t> _ack; // received sequence

    friend class Communicator;
};

#if 0
class HeartbeatTransciver :  public Transceiver
{
  protected:
    virtual void update() override;
    virtual void onNotify(const Notify notify, void* arg) override;
};

class HandshakeTransceiver : public Transceiver
{
  public:
    explicit HandshakeTransceiver(const uint8_t tid, const uint8_t maxPeer = 1) : Transceiver(tid), _maxPeer(maxPeer) { assert(maxPeer < ESP_NOW_MAX_TOTAL_PEER_NUM); }

    inline bool isPrimary()    const { return _id == 0; }
    inline bool isSecondary()  const { return _id > 0;  }
    inline bool isIndecided()  const { return _id < 0;  }

    inline int8_t identifier() const { return _id;      }
#if 0
    const MACAddress primaryAddress() const
    {
        return isPrimary() ? Communicator::instance().address()
                : (isSecondary() ? _peer.front() : MACAddress());
    }
    size_t numOfSecondary() { return isSecondary() ? _peer.size() : 0; }
    const std::vector<MACAddress>* secondaryAddress() const { return isPrimary() ? &_peer : nullptr; }
#endif
    
    enum Command : uint8_t
    {
        DeclarePrimary,
        ApplySecondary,
        AcceptSecondary,
        AckAccept,
    };
    struct Payload
    {
        Command command{};
        uint16_t session{};
        int8_t id{};
    };
    bool declarePrimary();
    
  protected:
    bool postCommand(const MACAddress& addr);
    virtual void onReceive(const MACAddress& addr, const uint8_t* data, int length);
    
  private:
    static constexpr int8_t INVALID_ID = -1;
    int8_t _id{INVALID_ID}; // 0 as primary, 1~ as secondary negative: inundecided
    uint8_t _maxPeer{};
    uint16_t _session{};
    uint32_t _validSecondary{}; // bits (
    std::vector<MACAddress> _peer{}; // [0]:primary [1]:others
};
#endif

//
}}
#endif
