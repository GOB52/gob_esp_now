/*!
  @file gob_esp_now.cpp
  @brief ESP-NOW wrapper, helper and utilities.
*/
#include <gob_esp_now.hpp>
#include <esp_log.h>
#include <esp32-hal.h> // millis
#include <ctime>
#include <cstdio>
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

float randf() { return esp_random() / (float)UINT32_MAX; } // [0.0, 1.0]

template<typename ...Args> String formatString(const char* fmt, Args... args)
{
    size_t sz = snprintf(nullptr, 0U, fmt, args...); // calculate length
    char buf[sz + 1];
    snprintf(buf, sizeof(buf), fmt, args...);
    return String(buf, sz);
}

#else
const char* err2cstr(esp_err_t) { return ""; }
#endif

#if 0
void dump(const void* p, const size_t len)
{
    String s;
    s = formatString("ADDR:%p\n", p);
    for(size_t i = 0; i < len; ++i)
    {
        auto c = len - i < 
        s += formatString("%*02x ", 
    }
}
#endif
    
inline uint64_t modify_uint64(const uint64_t u64, const uint8_t u8)
{
    return ((u64 & 0xFFFFFFFFFFFFFF00) + (u8 < (uint8_t)(u64 & 0xFF) ? 0x100 : 0x00)) | u8;
}

template<typename E> constexpr inline typename std::underlying_type<E>::type to_underlying(const E& e) noexcept
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

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
    _retry = false;

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

    auto ms = millis();
    
    if(!_began || !_canSend) { return; }

    // Retry send last data to last destination
    if(_retry)
    {
        // Connection lost?
        if(_sendTime && (ms - _sendTime) > _lossOfConnectionTime)
        {
            LIB_LOGD("<LoC> %s", _lastAddr.toString().c_str());
            if((bool)_lastAddr)
            {
                std::for_each(_transceivers.begin(), _transceivers.end(), [this](Transceiver* t)
                {
                    t->onNotify(Notify::Disconnect, &(this->_lastAddr));
                });
            }
            _queue[_lastAddr].clear();
            _lastData.clear();
            _sendTime = 0;
            _retry = false;
            return;
        }
        // Resend
        LIB_LOGD("Retry:%s %zu", _lastAddr.toString().c_str(), _lastData.size());
        send_esp_now((bool)_lastAddr ? _lastAddr.data() : nullptr, _lastData);
        return;
    }

    // Send if exists (Order by MACAddresss ASC, Null MACAddress is top)
    for(auto& it : _queue) // first:MACAddress second:vector
    {
        if(!it.second.empty())
        {
            LIB_LOGD("Send");
            send_esp_now((bool)it.first ? it.first.data() : nullptr, it.second);
            break;
        }
    }
}

// Post
bool Communicator::post(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    lock_guard lock(_sem);

    MACAddress addr = peer_addr ? MACAddress(peer_addr) : MACAddress();
    auto& md = _queue[addr]; // Create empty mapped_type if not exiets.
    if(md.capacity() < ESP_NOW_MAX_DATA_LEN) { md.reserve(ESP_NOW_MAX_DATA_LEN); } // Expand memory of the vector.

    // Modify header
    if(md.empty())
    {
        CommunicatorHeader ch;
        ch.app_id = _app_id;
        ch.count = 0;
        md.insert(md.end(), (uint8_t*)&ch, (uint8_t*)&ch + sizeof(ch));
    }

    // Overflow
    // TODO:..... 蓄積と再ポストはどうする?
    if(length + md.size() > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Overflow"); return false; }

    // Append data
    CommunicatorHeader* cheader = (CommunicatorHeader*)md.data();
    ++cheader->count;
    auto osz = md.size();
    md.insert(md.end(), (uint8_t*)data, (uint8_t*)data + length);
    assert(md.size() == osz + length && "Failed to insert");
    LIB_LOGD("CHeadr:%u,%zu", cheader->count, md.size());

    return true;
}

// Send directly
bool Communicator::send(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    lock_guard lock(_sem);

    if(!_began || !_canSend) { return false; }
    if(sizeof(CommunicatorHeader) + length > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Overflow"); return false; }

    std::vector<uint8_t> v;
    v.reserve(ESP_NOW_MAX_DATA_LEN);
    
    CommunicatorHeader ch;
    ch.app_id = _app_id;
    ch.count = 1;
    v.insert(v.end(), (uint8_t*)&ch, (uint8_t*)&ch + sizeof(ch));
    v.insert(v.end(), (uint8_t*)data, (uint8_t*)data + length);

    return send_esp_now(peer_addr, v);
}

// Must be call in lock.
bool Communicator::send_esp_now(const uint8_t* peer_addr, std::vector<uint8_t>& vec)
{
#if 0
    if(!vec.empty())
    {
        auto ch = (const CommunicatorHeader*)vec.data();
        auto th = (const TransceiverHeader*)(vec.data() + sizeof(CommunicatorHeader));
        LIB_LOGD("SEQ:%u", th->rudp.sequence);
    }
#endif
    
#if !defined(NDEBUG)
    if(isEnableDebug() && randf() < _debugSendLoss)
    {
        if(!_sendTime) { _sendTime = millis(); }
        _lastAddr = MACAddress(peer_addr);
        if(&_lastData != &vec) { _lastData = std::move(vec); }
        _retry = true;
        LIB_LOGD("[DF]:FailedS");
        return false;;
    }
#endif
    auto ret = esp_now_send(peer_addr, vec.data(), vec.size());
    if(ret != ESP_OK) { LIB_LOGE("Failed to esp_now_send %d:%s", ret, err2cstr(ret)); return false; }

    if(!_sendTime) { _sendTime = millis(); }
    _canSend = false;
    _lastAddr = MACAddress(peer_addr);
    if(&_lastData != &vec) { _lastData = std::move(vec); }

    return true;
}

bool Communicator::registerTransceiver(Transceiver* t)
{
    lock_guard lock(_sem);
    // Already registered?
    auto it = std::find(_transceivers.begin(), _transceivers.end(), t);
    if(it != _transceivers.end()) { return true; }

    _transceivers.push_back(t);
    std::sort(_transceivers.begin(), _transceivers.end(), [](Transceiver* a, Transceiver* b)
    {
        return a->identifier() < b->identifier();
                          
    });

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
    LIB_LOGV("static sent to %s %d", addr.toString().c_str(), status);
    instance().onSent(addr, status);
}

void Communicator::onSent(const MACAddress& addr, const esp_now_send_status_t status)
{
    lock_guard lock(_sem);
    _canSend = true;

    if(status == ESP_NOW_SEND_SUCCESS)
    {
        LIB_LOGV("ESP_NOW_SEND_SUCCESS %s", addr.toString().c_str());
        _retry = false;
        _lastData.clear();
        _sendTime = 0;
    }
    else
    {
        LIB_LOGE("ESP_NOW_SEND_FAIL %s", addr.toString().c_str());
        // 
        _retry = true;
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
    auto ch = (const CommunicatorHeader*)data;
    // Check 
    if(ch->signeture != CommunicatorHeader::SIGNETURE
       || ch->version != CommunicatorHeader::VERSION
       || ch->app_id != _app_id) { LIB_LOGE("Illegal data"); return; }

    // To the transceivers
    auto cnt = ch->count;
    data += sizeof(CommunicatorHeader);
    while(cnt--)
    {
        auto th = (const TransceiverHeader*)data;
        lock_guard lock(_sem);
        for(auto& t : _transceivers)
        {
            if(t->identifier() == th->tid)
            {
#if !defined(NDEBUG)
                auto f = randf();
                if(isEnableDebug() && f < _debugRecvLoss)
                {
                    LIB_LOGD("[DF]:FailedR tid:%u", t->identifier());
                    continue;
                }
#endif
                // Duplicate incoming data is not processed.
                // // TODO: 正しい seq を受けたら (飛んでいない、古いデータは棄却)
                uint64_t ack = t->ack(addr);
                if(modify_uint64(ack, th->rudp.sequence) > ack)
                {
                    t->on_receive(addr, th);
                    // TODO: 正しい seq を受けたら (飛んでいない)
                    if(th->hasPayload()) { t->onReceive(addr, th); }
                }
                else { LIB_LOGW("Duplicated data has come:%u",th->rudp.sequence); }
                break;
            }
        }
        data += th->size;
    }
}

#if !defined(NDEBUG)
String Communicator::debugInfo() const
{
    lock_guard lock(_sem);
    String s;
    s += formatString("Communicator app_id:%u, %zu transceivers\n", _app_id, _transceivers.size());

    for(auto& t : _transceivers)
    {
        s += String("  ") + t->debugInfo() + '\n';
    }

    s += "peer list:\n";
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            MACAddress addr(info.peer_addr);
            s += String("  ") + addr.toString() + '\n';
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
    s += formatString("last: [%s] %zu\n", _lastAddr.toString().c_str(), _lastData.size());
    s += formatString("queue:%zu\n", _queue.size());
    for(auto& it : _queue)
    {
        s += formatString("  [%s] %zu\n", it.first.toString().c_str(), it.second.size());
    }

    s.trim();
    return s;
}
#endif

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

uint8_t* Transceiver::make_data(uint8_t* obuf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    // Header
    TransceiverHeader* th = (TransceiverHeader*)obuf;
    th->tid = _tid;
    th->size = sizeof(*th) + length;
    {
        lock_guard lock(_sem);
        th->rudp.flag = to_underlying(flag);
        // TODO:RUDPのみ
        th->rudp.sequence = (++_sequence) & 0xFF;
        th->rudp.ack = _ack[MACAddress(peer_addr)];
    }
    if(data && length) { std::memcpy(obuf + sizeof(*th), data, length); } // Append payload
    return obuf;
}
                   
bool Transceiver::postReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
    return Communicator::instance().post(peer_addr, buf, sizeof(buf));
}

bool Transceiver::sendReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
    return Communicator::instance().send(peer_addr, buf, sizeof(buf));
}

void Transceiver::on_receive(const MACAddress& addr, const TransceiverHeader* data)
{
    lock_guard lock(_sem);
    // TODO:RUDP only
    // Update ACK
    auto old = _ack[addr];
    _ack[addr] = modify_uint64(_ack[addr], data->rudp.sequence);
    LIB_LOGW("seq:%u ack %llu => %llu", data->rudp.sequence, old, _ack[addr]);
}

#if !defined(NDEBUG)
String Transceiver::debugInfo() const
{
    lock_guard lock(_sem);
    String s;
    s = formatString("ID:%u SEQ:%llu\n", _tid, _sequence);
    for(auto& it :  _ack)
    {
        s += formatString("  ACK:[%s]:%llu\n", it.first.toString().c_str(), it.second);
    }
    s.trim();
    return s;
}
#endif

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
