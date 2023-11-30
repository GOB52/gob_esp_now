/*!
  @file gob_mac_address.hpp
  @brief MAC address
*/
#ifndef GOBLIB_MAC_ADDRESS_HPP
#define GOBLIB_MAC_ADDRESS_HPP

#include <cstdint>
#include <initializer_list>
#include <cstring> // std::memcpy
#include <algorithm> // std::move
#include <tuple> // std::tie
#include <esp_now.h>
#include <esp_mac.h>
#include <WString.h>

namespace goblib { namespace esp_now {

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
    explicit MACAddress(const uint8_t* addr) { if(addr) { std::memcpy(_addr, addr, sizeof(_addr)); } }
    explicit MACAddress(const char* str) { if(str) { parse(str); } }
    explicit MACAddress(const esp_mac_type_t mt) { get(mt); }
    MACAddress(const MACAddress& x) { _addr64 = x._addr64; }
    MACAddress(MACAddress&& x) noexcept { _addr64 = x._addr64; x._addr64 = 0; }
    /// @}

    ///@name Assignment
    ///@{
    MACAddress& operator=(const MACAddress& x)
    {
        if(this != &x) { _addr64 = x._addr64; }
        return *this;
    }
    MACAddress& operator=(MACAddress&& x) noexcept
    {
        if(this != &x) { _addr64 = x._addr64; x._addr64 = 0; }
        return *this;
    }
    ///@}

    ///@name Cast
    ///@{
    explicit inline constexpr operator uint64_t() const noexcept { return _addr64; } //!< @brief Cast to uint64_t
    /*!
      @brief Cast to bool
      @retval false Null address (all zero)
     */
    explicit inline constexpr operator bool() const noexcept { return _addr64 != 0ULL; }
    inline constexpr bool operator!() const noexcept { return !static_cast<bool>(*this); } //!< @brief Null address?
    ///@}
    
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
    /*!
      @brief Outputs as a String, such as "fe:dc:ba:98:76:54"
      @param mask Mask upper address if true
     */
    String toString(const bool mask = false) const;

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
//
}}
#endif
