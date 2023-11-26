/*!
  @file gob_mac_address.cpp
  @brief MAC address
*/
#include "gob_mac_address.hpp"

namespace goblib { namespace esp_now {

// -----------------------------------------------------------------------------
// class MACAddress
bool MACAddress::get(const esp_mac_type_t mtype)
{
    _addr64 = 0;
    return esp_read_mac(_addr, mtype) == ESP_OK;
}

bool MACAddress::parse(const char* str)
{
    assert(str && "nullptr");
    _addr64 = 0;
    int tmp[ESP_NOW_ETH_ALEN]{};
    auto res = sscanf(str ? str : "", "%x:%x:%x:%x:%x:%x", &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]);
    if(res == ESP_NOW_ETH_ALEN && std::all_of(tmp, tmp + ESP_NOW_ETH_ALEN, [](const int v) { return v < 256; }))
    {
        *this = MACAddress(tmp, tmp + ESP_NOW_ETH_ALEN);
        return true;
    }
    return false;
}

String MACAddress::toString(const bool mask) const
{
    char buf[128]{};
    if(mask) { snprintf(buf, sizeof(buf), "xx:xx:xx:%02x:%02x:%02x",  _addr[3], _addr[4], _addr[5]); }
    else     { snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",  _addr[0], _addr[1], _addr[2], _addr[3], _addr[4], _addr[5]); }
    buf[sizeof(buf)-1] = '\0';
    return String(buf);
}

//
}}
