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
#include <cstring> // std::memcpy
#include <algorithm> // std::move
#include <memory>
#include <vector>
#include <map>
#include <FreeRTOS/freeRTOS.h>
#include <FreeRTOS/semphr.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <WString.h>

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
//esp_fill_random

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
  @class MACAddress
  @brief MAC address class
 */
class MACAddress
{
  public:
    /// @name Constructor
    /// @warning Be careful with endians
    /// @warning 0x0000123456789abc => bc:9a:78:56:34:12 (little endian)
    /// @{
    constexpr MACAddress() {}
    constexpr explicit MACAddress(const uint64_t addr) : _addr64(addr) {} 
    constexpr MACAddress(const uint8_t u0, const uint8_t u1, const uint8_t u2, const uint8_t u3, const uint8_t u4, const uint8_t u5) : _addr{u0, u1, u2, u3, u4, u5} {}
    template <class InputIter> MACAddress(InputIter first, InputIter last)
    {
        assert(last - first == ESP_NOW_ETH_ALEN && "Illegal size");
        uint8_t* ptr{_addr};
        while(first != last) { *ptr++ = *first++; }
    }
    explicit MACAddress(std::initializer_list<uint8_t> il) : MACAddress(il.begin(), il.end()) {}
    explicit MACAddress(const uint8_t* addr) { assert(addr && "nullptr"); std::memcpy(_addr, addr, sizeof(_addr)); }
    explicit MACAddress(const char* str) { assert(str && "nullptr"); parse(str); }
    explicit MACAddress(const esp_mac_type_t mt) { get(mt); }
    MACAddress(const MACAddress& x) { _addr64 = x._addr64; }
    MACAddress(MACAddress&& x)      { _addr64 = x._addr64; x._addr64 = 0; }
    /// @}

    ///@name Assignment
    ///@{
    MACAddress& operator=(const MACAddress& x)
    {
        if(this != &x) { _addr64 = x._addr64; }
        return *this;
    }
    MACAddress& operator=(MACAddress&& x)
    {
        if(this != &x) { _addr64 = x._addr64; x._addr64 = 0; }
        return *this;
    }
    ///@}

    explicit inline constexpr operator uint64_t() const noexcept { return _addr64; } //!< @brief Cast to uint64_t
    
    /// @name Properties
    /// @{
    inline constexpr uint32_t OUI() const { return ((uint32_t)_addr[0] << 16) | ((uint32_t)_addr[1] << 8) | ((uint32_t)_addr[2]); }  //!< @brief Gets the Organisationally Unique Identifier
    inline constexpr uint32_t NIC() const { return ((uint32_t)_addr[3] << 16) | ((uint32_t)_addr[4] << 8) | ((uint32_t)_addr[5]); }  //!< @brief Gets the Network Interface Controller
    inline constexpr bool isUnicast()   const { return !isMulticast();     } //!< @brief Unicast address?
    inline constexpr bool isMulticast() const { return _addr[0] & 0x01;    } //!< @brief Multicast address?
    inline constexpr bool isBroadcast() const { return _addr64 == 0xFFFFFFFFFFFF; } //!< @brief Broadcast address?
    inline constexpr bool isLocal()     const { return _addr[0] & 0x02;    } //!< @brief Locally administered?
    inline constexpr bool isUniversal() const { return !isLocal();         } //!< @brief Universally administered?
    /// @}
    
    /// @name Element access 
    /// @{
    /*! @brief access specified element */
    inline const uint8_t& operator[](size_t i) const&  { assert(i < ESP_NOW_ETH_ALEN && "Invalid index"); return _addr[i]; }
    //! @brief access specified element 
    inline uint8_t&       operator[](size_t i) &       { assert(i < ESP_NOW_ETH_ALEN && "Invalid index"); return _addr[i]; }
    //! @brief access specified element 
    inline uint8_t        operator[](size_t i) const&& { assert(i < ESP_NOW_ETH_ALEN && "Invalid index"); return std::move(_addr[i]); }
    //! @brief direct access to the underlying array 
    inline constexpr const uint8_t* data() const { return _addr; }
    //! @brief direct access to the underlying array 
    inline uint8_t* data() { return const_cast<uint8_t*>(data()); }
    /// @}

    /*!
      @brief Obtain an address of the specified type
      @param mtype Type of address to be acquired
      @retval true Success
      @retval false Failed
      @sa https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html
    */
    bool get(const esp_mac_type_t mtype);
    bool parse(const char* str); //!< @brief Obtains an instance from a text string such as "12:34:56:78:aB:Cd"
    String toString() const; //!< @brief Outputs as a String, such as "fe:dc:ba:98:76:54"

    /// @cond 0
    friend bool operator==(const MACAddress& a, const MACAddress& b);
    friend bool operator!=(const MACAddress& a, const MACAddress& b);
    friend bool operator< (const MACAddress& a, const MACAddress& b);
    /// @endcond

  private:
    union
    {
        uint64_t _addr64{}; // Using higher 48 bits
        uint8_t _addr[ESP_NOW_ETH_ALEN];
    };
};

inline bool operator==(const MACAddress& a, const MACAddress& b) { return a._addr64 == b._addr64; }
inline bool operator!=(const MACAddress& a, const MACAddress& b) { return !(a == b); }
inline bool operator< (const MACAddress& a, const MACAddress& b)
{
    return std::tie(a._addr[0], a._addr[1], a._addr[2], a._addr[3], a._addr[4], a._addr[5]) <
            std::tie(b._addr[0], b._addr[1], b._addr[2], b._addr[3], b._addr[4], b._addr[5]);
}
inline bool operator> (const MACAddress& a, const MACAddress& b) { return b < a;    }
inline bool operator<=(const MACAddress& a, const MACAddress& b) { return !(a > b); }
inline bool operator>=(const MACAddress& a, const MACAddress& b) { return !(a < b); }

constexpr MACAddress BROADCAST = MACAddress(0xFFFFFFFFFFFF); //!< @brief Broadcast address

#if 0
/*!
  @struct RUDPHeader
  @brief RUDP-like header
 */
struct RUDPHeader
{
    /*!
      @enum Flag
      @brief Flag bits of RUDP type
      @warning Not everything is implemented according to RUDP specifications.
      @warning It is only a RUDP imitation.
     */
    enum Flag : uint8_t
    {
        SYN = 0x80, //!< @brief Begin session
        ACK = 0x40, //!< @brief Acknowledge
        EAK = 0x20, //!< @brief Selective ACK
        RST = 0x10, //!< @brief End session
        NUL = 0x08, //!< @brief Heartbeat
        CHK = 0x04, //!< @brief Calculate CRC include payload if on. Otherwise calclulate only header.
        TCS = 0x02, //!< @brief Demand for resumption
    };
    using flag_t = std::underlying_type<Flag>::type;

    //static constexpr uint32_t HEADER = 0X304e4547; // "GEN0" [G]ob_[E]sp_[N]ow header [0]
    //    static constexpr uint32_t HEADER = 0X314e4547; // "GEN0" [G]ob_[E]sp_[N]ow header [1] as unreliable
    //uint32_t header{HEADER}; //!< @brief Fixed header
    flag_t flag{};             //!< @brief Type flag bits.
    uint8_t channel{};       //!< @brief Channel No.
    uint16_t session{};      //!< @brief Session ID
    uint16_t sequence{};     //!< @brief Lower 16bits of sequence No.
    uint16_t ack{};          //!< @brief Lower 16bits of ACK No.
    //_crc{} with salt
    uint8_t length{};        //!< @brief Size of the payload
    uint8_t payload[1];      //!< @brief Head of the payload

} __attribute__((__packed__));

// Payload...
struct Payload_SYN
{
    uint16_t session{};  // session id
    uint16_t sequence{}; // sequence initial value.
};
#endif

/*!
  @enum Notify
  @brief Notify type
*/
enum Notify : uint8_t
{
    Disconnect,  //!< @brief Peer disconnection
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
      @brief Post transceiver data
      @param peer_addr MAC address (send all unicast peer if nullptr)
      @param data Transceiver header and subsequent payload
      @param length Length of tdata
      @note Only stores data, actual transmission is done by update()
     */
    bool post(const uint8_t* peer_addr, const void* data, const uint8_t length);

#if !defined(NDEBUG) || defined(DOXYGEN_PROCESS)
    ///@name Debugging features
    ///@note Create a pseudo send/receive loss condition
    ///@{
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
    
  protected:
    static constexpr uint16_t SIGNETURE = 0x454e; //!< @brief Packet signeture "GE"
    static constexpr uint8_t VERSION = 0x00;      //!< @brief Header version
    
    /*!
      @struct Header
      @brief Communicator data header
     */
    struct Header
    {
        uint16_t signeture{SIGNETURE}; //!< @brief  Signeture of the Communicator's data
        uint8_t  version{VERSION};     //!< @brief  Version of the header
        uint8_t  app_id{};             //!< @brief  Application-specific ID
        uint8_t  count{};              //!< @brief  Number of transceiver data
        uint8_t  reserved{};
        // Transceiver data continues for count times.
    };//6

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

    bool send(const MACAddress& addr, std::vector<uint8_t>& vec);
    
  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore

    uint8_t _app_id{};// Application-specific ID
    bool _began{};
    volatile bool _canSend{};
    volatile uint8_t _retry{};

    MACAddress _addr{}; // Self address
    std::vector<Transceiver*> _transceivers;

    MACAddress _lastDest{};
    std::vector<uint8_t> _lastData{};
    std::map<MACAddress, std::vector<uint8_t>> _queue;

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
      @struct Header
      @brief Common Transceiver header
     */
    struct Header
    {
        uint8_t  tid{};      //!< @brief Transerver id
        uint8_t  size{};     //!< @brief  Data size including this header
        uint8_t  sequence{}; //!< @brief  Lower 8bit of sequence
        uint8_t  ack{};      //!< @brief  Lower 8bit of ack
        // Payload continues if exists
        inline uint8_t* payload() const { return ((uint8_t*)this) + sizeof(*this); } //!< @brief Gets the payload pointer
    };//4

    /*! @param tid Unique value for each transceiver */
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

    ///@name Post
    ///@{
    /*!
      @brief Post data
      @param peer_addr Destination address. Post to all peer if nullptr
      @param data Payload data if exists
      @param length Length of the payload data if exists
     */
    bool post(const uint8_t* peer_addr, const void* data = nullptr, const uint8_t length = 0);
    //! @brief Post to all peer
    inline bool post(                   const void* data = nullptr, const uint8_t length = 0) { return post(nullptr, data, length); }
    //! @brief Post to destination
    template<typename T> inline bool post(const MACAddress& addr, const T& data) { return post(addr.data(), &data ,sizeof(data)); }
    //! @brief Post to all peer
    template<typename T> inline bool post(                        const T& data) { return post(nullptr,     &data ,sizeof(data)); }
    ///@}

    /*!
      @brief Call any function with lock
      @param Func Any functor
      @param args Arguments for functor
      @code
      class YourTransceiver : public Transceiver
      {
          //...Omission
          int yourThreadSafeFunction(const int mul)
          {
              return with_lock([&](const int m) { return _need_thread_safe_value * m; }, mul);
          }
          volatile int need_thread_safe_value{};
      };
      @endcode
     */
    template<typename Func, typename... Args> auto with_lock(Func func, Args&&... args) const
            -> decltype(func(std::forward<Args>(args)...))
    {
        lock_guard lock(_sem);
        return func(std::forward<Args>(args)...);
    }

    virtual void onNotify(const Notify, void* arg = nullptr){}
    /*! @warning Parent class function call required if overrided */
    virtual void onReceive(const MACAddress& addr, const Header* data);

  private:
    mutable SemaphoreHandle_t _sem{}; // Binary semaphore
    uint8_t _tid{}; // Transceiver unique identifier
    uint64_t _sequence{}; // send sequence
    std::map<MACAddress, uint64_t> _ack; // received sequence
};

#if 0
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
