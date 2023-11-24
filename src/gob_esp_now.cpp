/*!
  @file gob_esp_now.cpp
  @brief ESP-NOW wrapper, helper and utilities.
*/
#include <gob_esp_now.hpp>
#include <esp_log.h>
#include <ctime>
#include <cstdio>
//#include <FreeRTOS/FreeRTOS.h>
//#include <freeRTOS/semphr.h>
#if !defined(NDEBUG)
#include <esp_random.h>
#endif

namespace
{
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
//
}

// Logging
#define LIB_LOGE(fmt, ...) ESP_LOGE(LIB_TAG, "[%s:%d] %s " #fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LIB_LOGW(fmt, ...) ESP_LOGW(LIB_TAG, "[%s:%d] %s " #fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LIB_LOGI(fmt, ...) ESP_LOGI(LIB_TAG, "[%s:%d] %s " #fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LIB_LOGD(fmt, ...) ESP_LOGD(LIB_TAG, "[%s:%d] %s " #fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LIB_LOGV(fmt, ...) ESP_LOGV(LIB_TAG, "[%s:%d] %s " #fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__)

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
Communicator& Communicator::instance()
{
    // NOTE: In C++11 or later, initialization of static variables is guaranteed to be thread safe.
    static Communicator instance;
    return instance;
}

Communicator:: Communicator()
{
    _sem = xSemaphoreCreateBinary();
    xSemaphoreGive(_sem);
    _addr.get(ESP_MAC_WIFI_STA);
}

bool Communicator::begin(const uint8_t app_id)
{
    lock_guard lock(_sem);
    if(_began) { return true; }

    // ESP-NOW initialize not yet?
    esp_now_peer_num_t num{};
    if(esp_now_get_peer_num(&num) == ESP_ERR_ESPNOW_NOT_INIT)
    {
        LIB_LOGV("Initialize ESP-NOW");
        auto r = esp_now_init();
        if(r != ESP_OK)
        {
            LIB_LOGE("Failed to init:%d [%s]", r, err2cstr(r));
            return false;
        }
    }
    if((esp_now_register_send_cb(callback_onSent) != ESP_OK)
       || (esp_now_register_recv_cb(callback_onReceive) != ESP_OK) )
    {
        LIB_LOGE("Failed to register cb");
        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
        return false;
    }

    _began = true;
    _app_id = app_id;
    _transceivers.clear();
    _lastData.clear();
    _queue.clear();
    _canSend = true;
    _retry = 0;

    return true;
}


void Communicator::end()
{
    lock_guard lock(_sem);
    if(_began)
    {
        _began = false;
        _transceivers.clear();
        _lastData.clear();
        _queue.clear();
        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
    }
}

void Communicator::update()
{
    lock_guard lock(_sem);

    if(!_began || !_canSend) { return; }

    // TODO:回数によって切断する?
    if(_retry)
    {
        LIB_LOGE("Send retry");
        send(_lastDest, _lastData);
        return;
    }

    for(auto& it : _queue) // first:MACAddress second:vector
    {
        if(!it.second.empty())
        {
            send(it.first, it.second);
            break;
        }
    }
}

// Must be call in lock.
bool Communicator::send(const MACAddress& addr, std::vector<uint8_t>& vec)
{
    auto ret = esp_now_send(addr.data(), vec.data(), vec.size());
    if(ret != ESP_OK) { LIB_LOGE("Failed to send %d:%s", ret, err2cstr(ret)); return false; }

    ////
    const uint8_t* p = vec.data();;
    auto cnt = ((const Header*)p)->count;
    p += sizeof(Header);
    const Transceiver::Header* th = (const Transceiver::Header*)p;
    p += sizeof(Transceiver::Header);
    LIB_LOGD("[S]:%u %u sz:%u [%02x]", cnt, th->sequence, th->size, *p);

    _lastDest = addr;
    _lastData = std::move(vec);
    assert(vec.empty() && "Not moved");

    _canSend = !(ret == ESP_OK);
    return (ret == ESP_OK);
}

bool Communicator::post(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    lock_guard lock(_sem);

    MACAddress addr = peer_addr ? MACAddress(peer_addr) : MACAddress();
    auto& md = _queue[addr]; // Create empty mapped_type if not exiets.
    if(md.capacity() < ESP_NOW_MAX_DATA_LEN) { md.reserve(ESP_NOW_MAX_DATA_LEN); } // Expand memory of the vector.

    // Modify header
    if(md.empty())
    {
        Header h;
        h.app_id = _app_id;
        h.count = 0;
        md.insert(md.end(), (uint8_t*)&h, (uint8_t*)&h + sizeof(h));
    }

    // Overflow
    if(length + md.size() > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Overflow"); return false; }

    // Append data
    Header* header = (Header*)md.data();
    ++header->count;
    auto osz = md.size();
    md.insert(md.end(), (uint8_t*)data, (uint8_t*)data + length);
    assert(md.size() == osz + length && "Failed to insert");
    LIB_LOGD("[CP]%u/%zu", header->count, md.size());

    return true;
}

bool Communicator::registerTransceiver(Transceiver* t)
{
    lock_guard lock(_sem);
    // Already registered?
    auto it = std::find(_transceivers.begin(), _transceivers.end(), t);
    if(it != _transceivers.end()) { return true; }

    _transceivers.push_back(t);
    return true;
}

bool Communicator::unregisterTransceiver(Transceiver* t)
{
    lock_guard lock(_sem);
    auto it = std::remove(_transceivers.begin(), _transceivers.end(), t);
    if(it == _transceivers.end()) { LIB_LOGE("Not found"); return false; }
    _transceivers.erase(it, _transceivers.end());
    return true;
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
    esp_now_del_peer(BROADCAST.data());
    while(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        esp_now_del_peer(info.peer_addr);
    }
}

/*! @note return total peer */
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

/*!
  @fn callback_onSent
*/
void Communicator::callback_onSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    MACAddress addr(mac_addr);
    //LIB_LOGV("static sent to %s %d", addr.toString().c_str(), status);
    instance().onSent(addr, status);
}

void Communicator::onSent(const MACAddress& addr, const esp_now_send_status_t status)
{
    lock_guard lock(_sem);
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        _canSend = true;
        _retry = 0;
        //_queue[addr].clear();
        _lastData.clear();
    }
    else
    {
        LIB_LOGE("Failed to sent");
        ++_retry;
    }
}

void Communicator::callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length)
{
    MACAddress addr(mac_addr);
    LIB_LOGV("static receive from %s:%d", addr.toString().c_str(), length);
    instance().onReceive(addr, data, length);
    
}

void Communicator::onReceive(const MACAddress& addr, const uint8_t* data, const uint8_t length)
{
    const Header* h = (const Header*)data;
    if(h->signeture != SIGNETURE || h->app_id != _app_id) { LIB_LOGE("Illegal data"); return; }

    ////
    data += sizeof(Header);
    auto cnt = h->count;
    LIB_LOGD("Rtc:%u", cnt);
    
    while(cnt--)
    {
        auto th = (const Transceiver::Header*)data;
        lock_guard lock(_sem);
        for(auto& t : _transceivers)
        {
            if(t->identifier() == th->tid)
            {
                t->onReceive(addr, th);
                break;
            }
        }
        data += th->size;
    }
}

// -----------------------------------------------------------------------------
// class Transceiver
Transceiver::Transceiver(const uint8_t tid) : _tid(tid)
{
    _sem = xSemaphoreCreateBinary();
    xSemaphoreGive(_sem);
}

Transceiver::~Transceiver()
{
    vSemaphoreDelete(_sem);
}

bool Transceiver::post(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    lock_guard lock(_sem);

    MACAddress addr = peer_addr ? MACAddress(peer_addr) : MACAddress();
    uint8_t buf[sizeof(Header) + length]{};

    // Header
    Header* h = (Header*)buf;
    h->tid = _tid;
    h->size = sizeof(Header) + length;
    h->sequence = ++_sequence & 0xFF;
    h->ack = _ack[addr];

    // Append payload
    if(data && length) { std::memcpy(buf + sizeof(Header), data, length); }

    return Communicator::instance().post(peer_addr, buf, sizeof(buf));
}

void Transceiver::onReceive(const MACAddress& addr, const Header* data)
{
    lock_guard lock(_sem);
    uint64_t ack_h56 = _ack[addr] & 0xFFFFFFFFFFFFFF00;
    uint8_t  ack_l8 =  _ack[addr] & 0x00000000000000FF;
    if(data->sequence < ack_l8) { ack_h56 += 0x100; } // 8bit overflow?
    _ack[addr] = ack_h56 | data->sequence;
}

#if 0
// -----------------------------------------------------------------------------
// class HandshakeTransceiver
bool HandshakeTransceiver::declarePrimary()
{
    if(!isIndecided()) { LIB_LOGE("The state has already been determined. %d", _id); return false; }

    _validSecondary = 0;
    _peer.clear();
    _id = 0; // To be primary
           
    registerPeer(BROADCAST);

    Payload payload;
    payload.command = Command::DeclarePrimary;
    payload.sesson = esp_random() % 0xFFFF;
    return Communicator::instance().post(BROADCAST, payload);
}

bool HandshakeTransceiver::postCommand(const MACAddress& addr, const Command cmd)
{
    Payload payload;
    pauload.command = cmd;
    return Communicator::instance().post(addr, payload);
}

bool HandshakeTransceiver::acceptSecondary(const MACAddress& addr, const int8_t id)
{
    auto it = std::find(_secondary.begin(), secondary.end(), addr);
    if(it != _secondary.end())
    {
        LIB_LOGE("Already registered %s", addr.toString().c_str());
        return false;
    }

    _secondary.push_back(addr);

    Payload payload;
    pauload.command = Command::AcceptSecondary;
    payload.id = _secondary.size();
    
    return Communicator::instance().post(addr, payload);
}
                   
void HandshakeTransceiver::onReceive(const MACAddress& addr, const Header* h)
{
    assert(data);
    if(length < sizeof(Payload)) { LIB_LOGE("invalid data: %d", length); return; }

    const Payload* hp = (const Payload*)data;
    switch(hp->command)
    {
    case Command::DeclarePrimary: // From primary declared device
        if(isIndecided())
        {
            _session = hp->session;
            postCommand(addr, Command::RequestSecondary);
        }
        else {  LIB_LOGE("Command unacceptable:%u id:%d", hp->command(), _id); }
g        break;
    case Command::ApplySecondary: // From device responding to primary declaration
        if(isPrimary())
        {
            if(addSecondary())
            {
                acceptSecondary(addr, _secondary.size());
            }
            else
            {
                LIB_LOGW("Reject apply secondary from %s", addr.toString().c_str());
            }
        }
        else {  LIB_LOGE("Command unacceptable:%u id:%d", hp->command(), _id); }
        break;
    case Command::AcceptSecondary: // From primary
        if(isIndecided())
        {
            _id = hp->id; // To be secondary
            postCommand(addr, Command::AckAccept);
        }
        else {  LIB_LOGE("Command unacceptable:%u id:%d", hp->command(), _id); }
        break;
    case Command::AckAccept: // From accepted secondary
        if(isPrimary())
        {
            // seq ack check...
            _validSecondary |= (1ULL << hp->id);
        }
        else {  LIB_LOGE("Command unacceptable:%u id:%d", hp->command(), _id); }
        break;
    }
}
#endif
//
}}
