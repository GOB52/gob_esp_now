/*!
  @file gob_esp_now.hpp
  @brief ESP-NOW wrapper, helper and utilities

  @mainpage gob_esp_now
  ESP-NOW communication wrapped with RUDP-like send/receive
  C++11 or later.
  
  @author GOB https://twitter.com/gob_52_gob

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#ifndef GOBLIB_ESP_NOW_HPP
#define GOBLIB_ESP_NOW_HPP

#include <esp_now.h>
#include "internal/gob_esp_now_definition.hpp"
#include "gob_mac_address.hpp"
#include "gob_rudp.hpp"
#include "gob_transceiver.hpp"

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

/*!
  @enum Role
  @brief Role of communicator
*/
enum class Role : uint8_t
{
    None,      //!< @brief No role
    Primary,   //!< @brief Primary. Also known as master, controller(ESP-NOW Terminology)
    Secondary, //!< @brief Secondary. Also known as slave
};

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


class Transceiver;
class SystemTRX;

/*!
  @class Communicator
  @brief Manage transceivers
  @note Singleton
 */
class Communicator
{
  public:
    using notify_function = void(*)(const Notify notify, const void* arg);

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
    inline bool isNoRole() const   { return _role == Role::None; } //!< @brief No role?
    inline RUDP::config_t config() const { return _config; } //!< @brief Gets the RUDP configuretion
    ///@}

    //! @brief Set role
    void setRole(const Role r) { _role = r; }
    
    /*!
      @brief Begin communication
      @param app_id Unique value for each application
    */
    inline bool begin(const uint8_t app_id) { return begin(app_id, RUDP::config_t{}); }
    /*!
      @brief Begin communication
      @param app_id Unique value for each application
      @param cfg Configuretion
    */
    bool begin(const uint8_t app_id, const RUDP::config_t& cfg);
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
#if 0
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
#endif
    /*!
      @brief Call any function with lock
      @param Func Any functor
      @param args Arguments for functor
    */
    template<typename Func, typename... Args> auto with_lock(Func func, Args&&... args) const
            -> decltype(func(std::forward<Args>(args)...))
    {
        lock_guard lock(_sem);
        return func(std::forward<Args>(args)...);
    }

    /// @name Notification
    /// @{
    //! @brief Register callback on otify
    inline void registerNotifyCallback(notify_function f) { lock_guard _(_sem); _notifyFunction = f; }
    //! @brief Unregister callback
    inline void unregisterNotifyCallback() { lock_guard _(_sem); _notifyFunction = nullptr; }
    /*!
      @brief Notification
      @note Call your notify function if registered
     */
    void notify(const Notify n, const void* arg = nullptr);
    ///@}

    ///@name Handshake
    ///@{
    bool acceptRequest(); //!< @brief Accept SYN requests (broadcasting)
    bool postSYN(const MACAddress& addr); //!< @brief Post SYN request
    ///@}
    
#if !defined(NDEBUG) || defined(DOXYGEN_PROCESS)
    ///@name Debugging features
    ///@warning Conditions of use, <em><strong>NDEBUG must NOT be DEFINED.</strong></em>
    ///@{
    String debugInfo() const; //!< @brief Gets the information string with lock
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
    struct State
    {
        enum Status : uint8_t { None, Succeed, Failed };
        Status state{}; // status
        uint8_t retry{}; // retry counter
        unsigned long sentTime{}; // sent time
        unsigned long recvTime{}; // recv time
        inline void reset() { state = None; }

    } __attribute__((__packed__));

    Communicator();
    
    ///@name Callback
    ///@note Called from WiFi-task (Core 0)
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
    mutable SemaphoreHandle_t _sem{};
    QueueHandle_t _permitToSend{};
    
    uint8_t _app_id{};// Application-specific ID
    bool _began{};
    Role _role{Role::None};

    MACAddress _addr{}; // Self address
    RUDP::config_t _config{};

    SystemTRX* _sysTRX{}; // System transceiver
    std::vector<Transceiver*> _transceivers; // User transceivers

    unsigned long _lastSentTime{};
    MACAddress _lastSentAddr{};
    std::vector<uint8_t> _lastSentQueue{};

    map_t<MACAddress, std::vector<uint8_t>> _queue;
    map_t<MACAddress, State> _state;

    notify_function _notifyFunction{};

    // Time between esp_send and callback_onSent
    uint64_t _sentCount{};
    uint64_t _time{};
    unsigned long _minTime{99999999};
    unsigned long _maxTime{};
    
#if !defined(NDEBUG)
    bool _debugEnable{};
    float _debugSendLoss{}, _debugRecvLoss{};
#endif
};
//
}}
#endif
