/*!
  @file gob_esp_now.cpp
  @brief ESP-NOW wrapper and utilities.

  @mainpage gob_ESP-NOW
  This library is wrapped with ESP-NOW.  
  C++11 or later.
  
  @author GOB https://twitter.com/gob_52_gob

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include <gob_esp_now.hpp>
#include <esp_log.h>
#include <ctime>
#include <cstdio>
#if !defined(NDEBUG)
#include <esp_random.h>
#endif

// Logging
#define LIB_LOGE(fmt, ...) ESP_LOGE(myTag, "[%s:%d]" #fmt, __FILE__, __LINE__, ##__VA_ARGS__)
#define LIB_LOGW(fmt, ...) ESP_LOGW(myTag, "[%s:%d]" #fmt, __FILE__, __LINE__, ##__VA_ARGS__)
#define LIB_LOGI(fmt, ...) ESP_LOGI(myTag, "[%s:%d]" #fmt, __FILE__, __LINE__, ##__VA_ARGS__)
#define LIB_LOGD(fmt, ...) ESP_LOGD(myTag, "[%s:%d]" #fmt, __FILE__, __LINE__, ##__VA_ARGS__)
#define LIB_LOGV(fmt, ...) ESP_LOGV(myTag, "[%s:%d]" #fmt, __FILE__, __LINE__, ##__VA_ARGS__)

namespace
{
PROGMEM const char myTag[] = "gob_esp_now";

#if !defined(NDEBUG)
PROGMEM const char err_ok[]        = "Succeed";

PROGMEM const char err_unknown[]   = "Unknown";
PROGMEM const char err_not_init[]  = "Not initialized."; 
PROGMEM const char err_arg[]       = "Invalid argument";
PROGMEM const char err_no_mem[]    = "Out of memory";
PROGMEM const char err_full[]      = "Peer list is full";
PROGMEM const char err_not_found[] = "Peer is not found";
PROGMEM const char err_internal[]  = "Internal error";
PROGMEM const char err_exist[]     = "Peer has existed";
PROGMEM const char err_if[]        = "Interface error";

PROGMEM const char* errTable[] =
{
    err_unknown,
    err_not_init,
    err_arg,
    err_no_mem,
    err_full,
    err_not_found,
    err_internal,
    err_exist,
    err_if
};
const char* err2cstr(esp_err_t e)
{
    return (e == ESP_OK) ? err_ok : errTable[e - ESP_ERR_ESPNOW_BASE];
}
#else
const char* err2cstr(esp_err_t) { return ""; }
#endif

struct Packet
{
    enum Type : uint8_t
    {
        None,
        DeclarePrimary,
        DeclareSecondary,
        YourIdentifier,
        SyncTime,
        //
        UserType,

    };
    static constexpr uint32_t HEADER = 0X304e4547; // "GEN0" [G]ob_[E]sp_[N]ow ver [0]

    uint32_t header{HEADER};
    uint8_t type{};
    uint8_t reserved[3]{};
    
    explicit Packet(const Type t) : type(t) {}
    explicit operator bool() const noexcept { return header == HEADER; }
    bool     operator !()    const noexcept { return !static_cast<bool>(*this); }
};

struct IdentifierPacket : public Packet
{
    IdentifierPacket(const uint8_t id) : Packet(Type::YourIdentifier), identifier(id) {}
    uint8_t identifier{};
};
    
struct SyncTimePacket : public Packet
{
    SyncTimePacket(const time_t t) : Packet(Type::SyncTime), syncTime(t) {}
    time_t syncTime{};
    uint8_t extra[32]{};
    uint8_t length{};
};
//
}

namespace goblib { namespace esp_now {

// -----------------------------------------------------------------------------
// class MACAddress
const MACAddress MACAddress::BROADCAST(0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

/*!
  @param mtype Type of address to be acquired
  @retval true Success
  @retval false Failed
 */
bool MACAddress::get(const esp_mac_type_t mtype)
{
    return esp_read_mac(_addr, mtype) == ESP_OK;
}

bool MACAddress::parse(const char* str)
{
    int tmp[ESP_NOW_ETH_ALEN]{};
    auto res = sscanf(str ? str : "", "%x:%x:%x:%x:%x:%x", &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]);
    if(res == 6 && std::all_of(tmp, tmp + ESP_NOW_ETH_ALEN, [](const int v) { return v < 256; }))
    {
        *this = MACAddress(tmp, tmp + ESP_NOW_ETH_ALEN);
        return true;
    }
    return false;
}

String MACAddress::toString() const
{
    char buf[128]{};
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             _addr[0], _addr[1], _addr[2], _addr[3], _addr[4], _addr[5]);
    buf[sizeof(buf)-1] = '\0';
    return String(buf);
}

// -----------------------------------------------------------------------------
// class Communicator
std::vector<Communicator*> Communicator::_comm{};
#if !defined(NDEBUG)
int32_t Communicator::_lostPercent;
#endif

void Communicator::assign(const Communicator& c)
{
    _align = c.alignment();
    _addr = c.address();
}

/*!
  @note If ESP-NOW is not initialized, initialize it.
*/
bool Communicator::begin()
{
    // Already?
    auto it = std::find(_comm.begin(), _comm.end(), this);
    if(it != _comm.end()) { return true; }

    // ESP-NOW initialize not yet?
    esp_now_peer_num_t num{};
    if(esp_now_get_peer_num(&num) == ESP_ERR_ESPNOW_NOT_INIT)
    {
        auto r = esp_now_init();
        if(r != ESP_OK)
        {
            LIB_LOGE("Failed to init:%d [%s]", r, err2cstr(r));
            return false;
        }
    }

    // Set callback if first begin
    if(_comm.empty())
    {
        LIB_LOGV("regsiter callback");
        if((esp_now_register_send_cb(callback_onSent) != ESP_OK)
           || (esp_now_register_recv_cb(callback_onReceive) != ESP_OK) )
        {
            LIB_LOGE("Failed to register");
            esp_now_unregister_send_cb();
            esp_now_unregister_recv_cb();
            return false;
        }
    }

    _comm.push_back(this);
    return true;
}

void Communicator::end()
{
    auto it = std::remove(_comm.begin(), _comm.end(), this);
    if(it != _comm.end()) { _comm.erase(it, _comm.end()); }

    if(_comm.empty())
    {
        LIB_LOGV("unregsiter callback");
        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
    }
    _align = Alignment::None;
}
                
/*! @note send data to the peer whose MAC address matches peer_addr in the peer list */
bool Communicator::send(const MACAddress& addr, const void* data, const uint8_t length)
{
    if(length > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Too large data"); return false; }
    if(!existsPeer(addr)) { LIB_LOGE("%s was not registered", addr.toString().c_str()); return false; }

    //#if !defined(NDEBUG)
    //    if(esp_random() % 100 < _lostPercent) { return true; }
    //#endif
    
    // If peer_addr is not NULL, send data to the peer whose MAC address matches peer_addr
    auto ret = esp_now_send(addr.data(), (const uint8_t*)data, length);
    if(ret != ESP_OK) { LIB_LOGE("Failed to send:%d [%s]", ret, err2cstr(ret)); }
    return ret == ESP_OK;
}

/*! @note send data to all of the peers that are added to the peer list */
bool Communicator::send(const void* data, const uint8_t length)
{
    if(length > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Too large data"); return false; }
    //#if !defined(NDEBUG)
    //    if(esp_random() % 100 < _lostPercent) { return true; }
    //#endif

    // If peer_addr is NULL, send data to all of the peers that are added to the peer list
    auto ret = esp_now_send(nullptr, (const uint8_t*)data, length);
    if(ret != ESP_OK) { LIB_LOGE("Failed to send:%d [%s]", ret, err2cstr(ret)); }
    return ret == ESP_OK;
}

bool Communicator::registerPeer(const MACAddress& addr, const uint8_t channel, const bool encrypt, const uint8_t* lmk)
{
    if(!esp_now_is_peer_exist(addr.data()))
    {
        LIB_LOGV("register %s", addr.toString().c_str());

        esp_now_peer_info_t info{};
        std::memcpy(info.peer_addr, addr.data(), sizeof(info.peer_addr));
        info.channel = channel;
        info.encrypt = encrypt;
        if(encrypt) { std::memcpy(info.lmk, lmk, sizeof(info.lmk)); }

        auto ret = esp_now_add_peer(&info);
        if(ret != ESP_OK) { LIB_LOGE("Failed to add:%d [%s]", ret, err2cstr(ret)); }
        return ret == ESP_OK;
    }
    return true;
}

void Communicator::unregisterPeer(const MACAddress& addr)
{
    LIB_LOGV("unregister %s", addr.toString().c_str());
    auto ret = esp_now_del_peer(addr.data());
    if(ret != ESP_OK) { LIB_LOGE("Failed to del:%d [%s]", ret, err2cstr(ret)); }
}

/*! @warning Cannot clear multicast address. Explicitly call unregisterPeer if you want to erase it. */
void Communicator::clearPeer()
{
    esp_now_peer_info_t info{};
    esp_now_del_peer(MACAddress::BROADCAST.data());
    while(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        esp_now_del_peer(info.peer_addr);
    }
}

int Communicator::numOfPeer()
{
    esp_now_peer_num_t num{};
    return (esp_now_get_peer_num(&num) == ESP_OK) ? num.total_num : 0;
}

bool Communicator::existsPeer(const MACAddress& addr)
{
    esp_now_peer_info_t info{};
    return esp_now_get_peer(addr.data(), &info) == ESP_OK;
}
                   
void Communicator::callback_onSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    MACAddress addr(mac_addr);
    LIB_LOGV("static sent to %s", addr.toString().c_str());
    for(auto& c : _comm) { c->onSent(addr, status); }
}

void Communicator::callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length)
{
    MACAddress addr(mac_addr);
    const Packet* p = (const Packet*)data;

    LIB_LOGV("static receive from %s : [%s] %d", addr.toString().c_str(), (bool)*p ? "PKT" : "ICD", p->type);

    if(*p)
    {
    //#if !defined(NDEBUG)
    //    if(esp_random() % 100 < _lostPercent) { return; }}
    //#endif
        for(auto& c : _comm) { c->onReceive(addr, data,length); }
    }
    else
    {
        LIB_LOGW("Incorrect data?");
    }
}

// -----------------------------------------------------------------------------
// class Communicator
/*! @warning Regsiter broadcast address */
bool ConnectingCommunicator::begin()
{
    if(Communicator::begin())
    {
        return registerPeer(MACAddress::BROADCAST);
    }
    return false;
}

/*! @warning Unregsiter broadcast address */
void ConnectingCommunicator::end()
{
    Communicator::end();
    unregisterPeer(MACAddress::BROADCAST);
}

bool ConnectingCommunicator::declarePrimary()
{
    if(alignment() != Alignment::None) { return false; }

    Packet pkt(Packet::Type::DeclarePrimary);
    if(send(MACAddress::BROADCAST, pkt))
    {
        _align = Alignment::Primary;
        _id = 0; // Primary is always zero.
        return true;
    }
    return false;
}

void ConnectingCommunicator::onReceive(const MACAddress& addr, const uint8_t* data, int length)
{
    const Packet* p = (const Packet*)data;
    const IdentifierPacket* ip = (const IdentifierPacket*)data;

    switch(alignment())
    {
    case Alignment::Primary:
        // Notification from secondary and registered it not yet.
        if(p->type == Packet::Type::DeclareSecondary && !existsPeer(addr))
        {
            if(registerPeer(addr))
            {
                _secandary.push_back(addr);
                LIB_LOGV("%s is my secondary", addr.toString().c_str());

                IdentifierPacket pkt(_secandary.size());
                LIB_LOGV("Send id:%u to %s", pkt.identifier, addr.toString().c_str());
                if(!send(addr, pkt))
                {
                    LIB_LOGE("Failed to send id");
                }
            }
            else
            {
                LIB_LOGE("Failed to add secondary");
            }
        }
        break;
    case Alignment::None:
        // Notification from primary?:
        if(p->type == Packet::Type::DeclarePrimary)
        {
            LIB_LOGV("From primary %s.", addr.toString().c_str());
            _align = Alignment::Secondary;
            if(registerPeer(addr))
            {
                _primary = addr;
                // Notify the primary that I am a secondary.
                Packet pkt(Packet::Type::DeclareSecondary);
                if(!send(addr, pkt))
                {
                    LIB_LOGE("Failed to send ack");
                }
            }
            else
            {
                LIB_LOGE("Failed to add primary");
            }
        }
        break;
    case Alignment::Secondary:
        if(ip->type == Packet::Type::YourIdentifier)
        {
            _id = ip->identifier;
            LIB_LOGV("Receive ID:%u", _id);
        }
        break;
    }
}

void ConnectingCommunicator::onSent(const MACAddress& addr, esp_now_send_status_t status)
{
    LIB_LOGV("sent:%d", status);
}

// -----------------------------------------------------------------------------
// class SynchronousCommunicator

bool SynchronousCommunicator::sendSyncTime(const time_t t, const void* extra, const uint8_t length)
{
    if(!isPrimary()) { return false; }
    
    SyncTimePacket packet(t);
    _syncTime = t;
    if(numOfPeer() == 0) { return true; } // Standalone

    if(extra)
    {
        auto len = length;
        if(len > sizeof(packet.extra)) { len = sizeof(packet.extra); LIB_LOGW("The overflow will be cut"); }
        std::memcpy(packet.extra, extra, len);
        packet.length = len;
    }

    LIB_LOGV("syncTime:%ld extra:%u", t, length);
    return send(packet);
}

bool SynchronousCommunicator::reachSyncTime() const
{
    return _syncTime && std::time(nullptr) >= _syncTime;
}

void SynchronousCommunicator::onReceive(const MACAddress& addr, const uint8_t* data, int length)
{
    const SyncTimePacket* p = (const SyncTimePacket*)data;

    if(p->type == Packet::Type::SyncTime)
    {
        LIB_LOGV("[R]syncTime:%ld", p->syncTime);
        _syncTime = p->syncTime;
        if(p->length)
        {
            _extra.resize(p->length);
            std::memcpy(_extra.data(), p->extra, p->length);
        }
    }
}

void SynchronousCommunicator::onSent(const MACAddress& addr, esp_now_send_status_t status)
{
    LIB_LOGV("sent:%d", status);
}

//
}}
