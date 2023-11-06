/*!
  @file gob_esp_now.hpp
  @brief ESP-NOW wrapper and utilities.

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
#include <vector>
#include <esp_now.h>
#include <esp_mac.h>
#include <WString.h>

namespace goblib
{
namespace esp_now
{

/*!
  @class MACAddress
  @brief MAC address class
 */
class MACAddress
{
  public:
    /// @name Constructor
    /// @{
    constexpr MACAddress() {}
    constexpr MACAddress(const uint8_t u0, const uint8_t u1, const uint8_t u2, const uint8_t u3, const uint8_t u4, const uint8_t u5) : _addr{u0, u1, u2, u3, u4, u5} {}
    template <class InputIter> MACAddress(InputIter first, InputIter last)
    {
        assert(last - first == ESP_NOW_ETH_ALEN && "Illegal size");
        uint8_t* ptr{_addr};
        while(first != last) { *ptr++ = *first++; }
    }
    MACAddress(std::initializer_list<uint8_t> il) : MACAddress(il.begin(), il.end()) {}
    explicit MACAddress(const uint8_t* addr) { std::memcpy(_addr, addr, sizeof(_addr)); }
    explicit MACAddress(const char* str) { parse(str); }
    MACAddress(const MACAddress& x) { std::memcpy(_addr, x._addr, sizeof(_addr)); }
    MACAddress(MACAddress&& x)      { std::memcpy(_addr, x._addr, sizeof(_addr)); }
    /// @}

    ///@name Assignment
    ///@{
    MACAddress& operator=(const MACAddress& x)
    {
        if(this != &x) { std::memcpy(_addr, x._addr, sizeof(_addr)); }
        return *this;
    }
    MACAddress& operator=(MACAddress&& x)
    {
        if(this != &x) { std::memcpy(_addr, x._addr, sizeof(_addr)); }
        return *this;
    }
    ///@}
        
    friend bool operator==(const MACAddress& a, const MACAddress& b);
    friend bool operator!=(const MACAddress& a, const MACAddress& b);

    /// @name Properties
    /// @{
    inline uint32_t OUI() const { return ((uint32_t)_addr[0] << 16) | ((uint32_t)_addr[1] << 8) | ((uint32_t)_addr[2]); }  //!< @brief Gets the Organisationally Unique Identifier
    inline uint32_t NIC() const { return ((uint32_t)_addr[3] << 16) | ((uint32_t)_addr[4] << 8) | ((uint32_t)_addr[5]); }  //!< @brief Gets the Network Interface Controller
    inline bool isUnicast()   const { return !isMulticast();     } //!< @brief Unicast address?
    inline bool isMulticast() const { return _addr[0] & 0x01;    } //!< @brief Multicast address?
    inline bool isBroadcast() const { return *this == BROADCAST; } //!< @brief Broadcast address?
    inline bool isLocal()     const { return _addr[0] & 0x02;    } //!< @brief Locally administered?
    inline bool isUniversal() const { return !isLocal();         } //!< @brief Universally administered?
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
    inline const uint8_t* data() const { return _addr; }
    //! @brief direct access to the underlying array 
    inline uint8_t* data() { return const_cast<uint8_t*>(data()); }
    /// @}

    //! @brief Obtain an address of the specified type
    bool get(const esp_mac_type_t mtype); 
    bool parse(const char* str); //!< @brief Obtains an instance from a text string such as "12:34:56:78:aB:Cd"
    String toString() const; //!< @brief Outputs as a String, such as "fe:dc:ba:98:76:54"

    static const MACAddress BROADCAST; //!< @brief Broadcast address

  private:
    uint8_t _addr[ESP_NOW_ETH_ALEN]{};
};

/// @name Equality Compare
/// @{
inline bool operator==(const MACAddress& a, const MACAddress& b)
{
    return std::memcmp(a._addr, b._addr, sizeof(a._addr)) == 0;
}
inline bool operator!=(const MACAddress& a, const MACAddress& b) { return !(a == b); }
/// @}


/*!
  @class Communicator
  @brief Communicator class wrapping ESP-NOW
 */
class Communicator
{
  public:
    /*!
      @enum Alignment
      @brief Communicator alignment.
     */
    enum class Alignment : uint8_t
    {
        None,      //!< @brief Unsettled state
        Primary,   //!< @brief as Primary (Contoller as ESP-NOW)
        Secondary, //!< @brief as Secondary (Slave as ESP-NOW)
    };

    /// @name Constructor
    /// @{
    Communicator() { _addr.get(ESP_MAC_WIFI_STA); }
    virtual ~Communicator() { end(); }

    /// @name Properties
    /// @{
    /*! @brief Gets the alignment */
    inline Alignment alignment() const       { return _align; }
    inline bool isPrimary() const            { return alignment() == Alignment::Primary; } //!< @brief Am I primary?
    inline bool isSecondary() const          { return alignment() == Alignment::Secondary; } //!< Am I secondary?
    inline const MACAddress& address() const { return _addr; } //!< @brief Gets the MAC address
    /// @}

    //! @brief Assign information from other communicator.
    void assign(const Communicator& c);
    //! @brief Begin communicator
    virtual bool begin(); 
    //! @brief End communicator
    virtual void end();
    static size_t numOfCommunicator() { return _comm.size(); }
    
    /// @name Send
    /// @{

    ///@brief Send data to a specific address.
    bool send(const MACAddress& addr, const void* data, const uint8_t length);
    //! @brief Send data to a specific address.
    template <typename T> bool send(const MACAddress& addr, const T& data) { return send(addr, &data, sizeof(data)); }
    //! @brief Send data to addresses in the peer list.
    bool send(const void* data, const uint8_t length);
    //! @brief Send data to addresses in the peer list.
    template <typename T> bool send(const T& data) { return send(&data, sizeof(data)); }
    /// @}
    
    /// @name Peer
    /// @brief Peer registration status is global
    /// @{
    static bool registerPeer(const MACAddress& addr, const uint8_t channel = 0, const bool encrypt = false, const uint8_t* lmk = nullptr); //!< @brief Register peer
    static void unregisterPeer(const MACAddress& addr); //!< @brief Unregister peer
    static void clearPeer();                            //!< @brief Clear all peer
    static int  numOfPeer();                            //!< @brief Number of registered peers
    static bool existsPeer(const MACAddress& addr);     //!< @brief Exists peer?
    /// @}

#if !defined(NDEBUG)
    /// @name For debug
    /// @{
    static void setLostRate(int32_t percent) { _lostPercent = percent; }
  private:
    static int32_t _lostPercent;
    /// @}
#endif
    
  protected:
    virtual void onReceive(const MACAddress& addr, const uint8_t* data, int length) = 0;
    virtual void onSent(const MACAddress& addr, esp_now_send_status_t status) = 0;

    static void callback_onSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length);
    
    Alignment _align{Alignment::None};
    uint8_t _channel{};
    MACAddress _addr{}; // self address

  private:
    static std::vector<Communicator*> _comm;
};

/*!
  @class ConnectingCommunicator
  @brief Class to establish a connection at the primary or secondary
 */
class ConnectingCommunicator : public Communicator
{
  public:
    ConnectingCommunicator() : Communicator() {}
    virtual ~ConnectingCommunicator() {}

    const MACAddress& primaryAddress() const { return isPrimary() ? address() : _primary; }
    const std::vector<MACAddress>& secondaryAddress() const { return _secandary; }
    uint8_t numOfSecondary() const { return _secandary.size(); }
    bool validIdentifier() const { return _id != INVALID_ID; }
    uint8_t identifier() const { return _id; }
    
    virtual bool begin() override;
    virtual void end() override;
    
    //! @brief Declares itself as primary
    bool declarePrimary();
    
  protected:
    virtual void onReceive(const MACAddress& addr, const uint8_t* data, int length) override;
    virtual void onSent(const MACAddress& addr, esp_now_send_status_t status) override;

    MACAddress _primary{};
    std::vector<MACAddress> _secandary{};
    static constexpr uint8_t INVALID_ID = 0xff;
    uint8_t _id{INVALID_ID};
};

/*!
  @class SynchronousCommunicator
  @brief Claas to synchronous.
*/
class SynchronousCommunicator: public Communicator
{
  public:
    SynchronousCommunicator() : Communicator() {}

    /// @name Properties
    /// @{
    time_t syncTime() const { return _syncTime; } //!< @brief estimated time of synchronization
    bool hasExtra() const { return !_extra.empty(); }
    const void* extra() const { return _extra.data(); }
    /// @}

    /*! @brief Send scheduled synchronization time
      @warning Only the primary can run */
    bool sendSyncTime(const time_t t, const void* extra = nullptr, const uint8_t length = 0);
    //! @brief Has the scheduled synchronization time been reached?
    bool reachSyncTime() const;
    
  protected:    
    virtual void onReceive(const MACAddress& addr, const uint8_t* data, int length) override;
    virtual void onSent(const MACAddress& addr, esp_now_send_status_t status) override;

  private:
    time_t _syncTime{};
    std::vector<uint8_t> _extra;
};

/*!
  @struct RUDPHeader
  @note Like the RUDP
  @sa https://en.wikipedia.org/wiki/Reliable_User_Datagram_Protocol
 */
struct RUDPHEader
{
    /*! @enum Flag
     */
    enum Flag : uint8_t
    {
        Data, //!< @brief Packet is any data.
        Ack   //!< @brief Packet is ACK.
    };

    uint32_t magic{};    //!< @brief Application-specific magic No.
    Flag flag{};         //!< @brief Flag
    uint8_t channel{};   //!< @brief channel No.
    uint8_t reserved[2];
    uint32_t sequence{}; //!< Sequence No.
    uint32_t ack{};      //!< Received sequence No.
}; // packed

class RUDPCommunicator : public Communicator
{
  public:
    RUDPCommunicator() : Communicator() {}

  protected:    
    virtual void onReceive(const MACAddress& addr, const uint8_t* data, int length) override;
    virtual void onSent(const MACAddress& addr, esp_now_send_status_t status) override;

  private:
    uint64_t _sequence{}, _sequenceAck{};
    uint64_t _ack[8];

    

};

//
}}
#endif
