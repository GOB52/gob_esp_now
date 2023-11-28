/*!
  @file gob_esp_now.cpp
  @brief ESP-NOW wrapper, helper and utilities.
*/
#include "gob_esp_now.hpp"
#include "internal/gob_esp_now_log.hpp"
#include <esp32-hal.h> // millis
#include <ctime>
#include <cstdio>
#if !defined(NDEBUG)
#include <esp_random.h>
#endif

namespace
{
// For debug
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

void initialize_esp_now()
{
    esp_now_peer_num_t num{};
    if(esp_now_get_peer_num(&num) == ESP_ERR_ESPNOW_NOT_INIT)
    {
        LIB_LOGV("Initialize ESP-NOW");
        auto r = esp_now_init();
        if(r != ESP_OK)
        {
            LIB_LOGE("Failed to init:%d [%s]", r, err2cstr(r));
        }
    }
}

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
    if(_began) { return true; }

    // Initialize ESP-NOW if it has not already been initialized.
    initialize_esp_now();

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
    for(auto& t : _transceivers) { t->with_lock([&t](){ t->build_peer_map(); }); }
    
    return true;
}


void Communicator::end()
{
    if(_began)
    {
        lock_guard _(_sem);
        _began = false;
        _transceivers.clear();
        _lastData.clear();
        _queue.clear();
        _retry = false;
        _sentTime = 0;

        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
    }
}

void Communicator::update()
{
    if(!_began) { return; }

    auto ms = millis();

    lock_guard _(_sem);

    for(auto& t : _transceivers) { t->update(ms); } // update transceivers
    
    if(!_canSend) { return; }

    // Retry send last data to last destination
    if(false && _retry)
    {
#if 0
        // Connection lost?
        if(_sentTime && (ms - _sentTime) > _locTime)
        {
            LIB_LOGD("<LoC> %s", _lastAddr.toString().c_str());
            if((bool)_lastAddr) { notify(Notify::Disconnect, &_lastAddr); }
            _queue[_lastAddr].clear();
            _lastData.clear();
            _sentTime = 0;
            _retry = false;
            return;
        }
#endif
        // Resend
        if(!_lastData.empty())
        {
            LIB_LOGD("Retry:%s %zu", _lastAddr.toString().c_str(), _lastData.size());
            send_esp_now((bool)_lastAddr ? _lastAddr.data() : nullptr, _lastData);
        }
        else { _retry = false; }
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

// TODO:Send のみ非Transceiver あり?
// TransceiverHader* th にすべきでは?

// Post
bool Communicator::post(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }

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
    cheader->size = md.size();
    LIB_LOGD("CHeadr:%u,%zu", cheader->count, md.size());

    return true;
}

// Send directly
bool Communicator::send(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }

    if(!_began || !_canSend) { return false; }
    if(sizeof(CommunicatorHeader) + length > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Overflow"); return false; }

    std::vector<uint8_t> v;
    v.reserve(ESP_NOW_MAX_DATA_LEN);
    
    CommunicatorHeader ch;
    ch.app_id = _app_id;
    ch.count = 1;
    ch.size = sizeof(ch) + length;
    v.insert(v.end(), (uint8_t*)&ch, (uint8_t*)&ch + sizeof(ch));
    v.insert(v.end(), (uint8_t*)data, (uint8_t*)data + length);
    
    return send_esp_now(peer_addr, v);
}

bool Communicator::send_esp_now(const uint8_t* peer_addr, std::vector<uint8_t>& vec)
{
#if 0
    if(!vec.empty())
    {
        //auto ch = (const CommunicatorHeader*)vec.data();
        auto th = (const TransceiverHeader*)(vec.data() + sizeof(CommunicatorHeader));
        LIB_LOGD("RF:%02x. SEQ:%u", th->rudp.flag, th->rudp.sequence);
    }
#endif
    
#if !defined(NDEBUG)
    if(isEnableDebug() && randf() < _debugSendLoss)
    {
        if(!_sentTime) { _sentTime = millis(); }
        move_to_last(peer_addr, vec);
        _retry = true;
        LIB_LOGD("[DF]:FailedS");
        return false;;
    }
#endif
    auto ret = esp_now_send(peer_addr, vec.data(), vec.size());
    if(ret != ESP_OK)
    {
        LIB_LOGE("Failed to esp_now_send %s %d:%s", MACAddress(peer_addr).toString().c_str(), ret, err2cstr(ret));
        if(peer_addr) { vec.clear(); }
        return false;
    }

    if(!_sentTime) { _sentTime = millis(); }
    _canSend = false;
    move_to_last(peer_addr, vec);
    return true;
}

void Communicator::move_to_last(const uint8_t* peer_addr, std::vector<uint8_t>& vec)
{
    // Reject unreliable data
    auto ptr = vec.data();
    auto ch = (CommunicatorHeader*)ptr;
    auto cnt = ch->count;
    ptr += sizeof(CommunicatorHeader);

    //LIB_LOGV("CH0:%u:%u", ch->count, ch->size);
    while(cnt--)
    {
        auto th = (TransceiverHeader*)ptr;
        //LIB_LOGV("RF:%02x %u", th->rudp.flag, th->size);

        if(th->rudp.flag == 0)
        {
            ch->count--;
            ch->size -= th->size;
            auto rs = vec.begin() + (ptr - vec.data());
            auto re = vec.begin() + (ptr + th->size - vec.data());
            auto it = vec.erase(rs, re);
            ptr = vec.data() + std::distance(vec.begin(), it);
        }
        else { ptr += th->size; }
    }
    //LIB_LOGV("CH1:%u:%u", ch->count, ch->size);

    // Move to last if packet has RUDP
    if(ch->count)
    {
        _lastAddr = MACAddress(peer_addr);
        if(&_lastData != &vec) { _lastData = std::move(vec); }
    }
    else { vec.clear(); } // No data requiring retry preparation
}

void Communicator::notify(const Notify n, const void* arg)
{
    for(auto& t : _transceivers)
    {
        t->on_notify(n, arg);
        t->onNotify(n, arg);
    }
}
                   
bool Communicator::registerTransceiver(Transceiver* t)
{
    t->build_peer_map();

    // Already registered?
    auto it = std::find(_transceivers.begin(), _transceivers.end(), t);
    if(it != _transceivers.end()) { LIB_LOGE("Already registered"); return false; }

    // Is the same ID registered?
    it = std::find_if(_transceivers.begin(), _transceivers.end(), [&t](Transceiver* a)
    {
        return t->identifier() == a->identifier();
    });
    if(it != _transceivers.end()) { LIB_LOGE("Same id exists %u", t->identifier()); return false; }
    
    _transceivers.push_back(t);
    std::sort(_transceivers.begin(), _transceivers.end(), [](Transceiver* a, Transceiver* b)
    {
        return a->identifier() < b->identifier();
                          
    });
    return true;
}

bool Communicator::unregisterTransceiver(Transceiver* t)
{
    auto it = std::remove(_transceivers.begin(), _transceivers.end(), t);
    if(it == _transceivers.end()) { LIB_LOGE("Not found"); return false; }
    _transceivers.erase(it, _transceivers.end());
    return true;
}

bool Communicator::registerPeer(const MACAddress& addr, const uint8_t channel, const bool encrypt, const uint8_t* lmk)
{
    // Initialize ESP-NOW if it has not already been initialized.
    initialize_esp_now();
    
    if(addr != BROADCAST && addr.isMulticast())
    {
        LIB_LOGE("Not support multicast address");
        return false;
    }
    
    esp_err_t ret{ESP_OK};
    if(!esp_now_is_peer_exist(addr.data()))
    {
        LIB_LOGV("register %s", addr.toString().c_str());

        esp_now_peer_info_t info{};
        std::memcpy(info.peer_addr, addr.data(), sizeof(info.peer_addr));
        info.channel = channel;
        info.encrypt = encrypt;
        if(encrypt) { std::memcpy(info.lmk, lmk, sizeof(info.lmk)); }

        ret = esp_now_add_peer(&info);
        if(ret == ESP_OK)
        {
            for(auto& t : _transceivers) { t->with_lock([&t](){ t->build_peer_map(); }); }
        }
    }
    if(ret != ESP_OK) { LIB_LOGE("Failed to add:%d [%s]", ret, err2cstr(ret)); }
    return ret == ESP_OK;;
}

void Communicator::unregisterPeer(const MACAddress& addr)
{
    LIB_LOGV("unregister %s", addr.toString().c_str());
    auto ret = esp_now_del_peer(addr.data());
    if(ret != ESP_OK) { LIB_LOGE("Failed to del:%d [%s]", ret, err2cstr(ret)); }
}

void Communicator::clearPeer()
{
    esp_now_peer_info_t info{};
    esp_now_del_peer(BROADCAST.data());
    while(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        esp_now_del_peer(info.peer_addr);
    }
}

uint8_t Communicator::numOfPeer()
{
    esp_now_peer_num_t num{};
    return (esp_now_get_peer_num(&num) == ESP_OK) ? num.total_num : 0;
}

bool Communicator::existsPeer(const MACAddress& addr)
{
    return esp_now_is_peer_exist(addr.data());
}

void Communicator::callback_onSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    MACAddress addr(mac_addr);
    LIB_LOGV("static sent to %s %d", addr.toString().c_str(), status);
    instance().onSent(addr, status);
}

void Communicator::onSent(const MACAddress& addr, const esp_now_send_status_t status)
{
    lock_guard _(_sem);
    _canSend = true;

    if(status == ESP_NOW_SEND_SUCCESS)
    {
        LIB_LOGV("ESP_NOW_SEND_SUCCESS %s", addr.toString().c_str());
        _retry = false;
        _lastData.clear();
        _sentTime = 0;
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
        lock_guard _(_sem);
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
                // Duplicate or old incoming data is not processed.
                // // TODO: 正しい seq を受けたら (飛んでいない、古いデータは棄却)
                //// TODO
                // 受信失敗時の再送要求機構 with SEQが飛んだ場合の対処
                // TODO

                // 未来のデータ?
                uint64_t seq = t->with_lock([&t,&addr](){ return t->sequence(addr); });
                if(modify_uint64(seq, th->rudp.sequence) > seq)
                {
                    // seq + 1 より上なら飛んでいる
                    t->on_receive(addr, th);
                    t->onReceive(addr, th);
                }
                else { LIB_LOGW("Duplicated data has come:%u / %llu",th->rudp.sequence, seq); }
                break;
            }
        }
        data += th->size;
    }
}

#if !defined(NDEBUG)
String Communicator::debugInfo() const
{
    lock_guard _(_sem);
    String s;
    s += formatString("Communicator app_id:%u, %zu transceivers\n", _app_id, _transceivers.size());

    for(auto& t : _transceivers)
    {
        s += t->debugInfo() + '\n';
    }

    s += "peer list\n";
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

void Transceiver::reset()
{
    _sequence = 0;
    for(auto& it : _recvSeq) { it.second = 0; }
    for(auto& it : _recvAck) { it.second = 0; }
}

void Transceiver::clear(const MACAddress& addr)
{
    _recvSeq.erase(addr);
    _recvAck.erase(addr);
}

bool Transceiver::postReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
    return Communicator::instance().post_with_lock(peer_addr, buf, sizeof(buf));
}

bool Transceiver::sendReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
    return Communicator::instance().send_with_lock(peer_addr, buf, sizeof(buf));
}

bool Transceiver::postUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::NONE, peer_addr, data, length);
    return Communicator::instance().post_with_lock(peer_addr, buf, sizeof(buf));
}

bool Transceiver::sendUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::NONE, peer_addr, data, length);
    return Communicator::instance().send_with_lock(peer_addr, buf, sizeof(buf));
}

// Build _recvSeq/_recvAck based on the current peer list
// Existing element remain in place.
void Transceiver::build_peer_map()
{
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            // Make it if not exists
            MACAddress addr(info.peer_addr);
            if(!_recvSeq.count(addr)) {  _recvSeq[addr] = 0;  } 
            if(!_recvAck.count(addr))  { _recvAck[addr]  = 0; }
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
}

uint8_t* Transceiver::make_data(uint8_t* obuf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    // Header
    TransceiverHeader* th = (TransceiverHeader*)obuf;
    th->tid = _tid;
    th->size = sizeof(*th) + length;
    {
        auto fvalue = to_underlying(flag);
        th->rudp.flag = fvalue;
        if(fvalue) // Only RUDP
        {
            lock_guard _(_sem);
            th->rudp.sequence = (++_sequence) & 0xFF;
            th->rudp.ack = peer_addr ? (_recvSeq[MACAddress(peer_addr)] & 0xFF) : 0x00;
            LIB_LOGV("set:%s :%02x %u/%u | %llu/%llu",
                     MACAddress(peer_addr).toString().c_str(),
                     fvalue, th->rudp.sequence, th->rudp.ack,
                     _sequence, _recvSeq[MACAddress(peer_addr)]);

        }
    }
    if(data && length) { std::memcpy(obuf + sizeof(*th), data, length); } // Append payload
    return obuf;
}
                   
#if 0
bool equalAck(const uint8_t seq, const MACAddress& addr)
{
}

bool equalAck(const MACAddress& addr)
{
}
#endif

void Transceiver::on_receive(const MACAddress& addr, const TransceiverHeader* data)
{
    lock_guard _(_sem);
    LIB_LOGV("%s f:%02x", addr.toString().c_str(), data->rudp.flag);
    if(data->rudp.flag & (RUDP::flag_t)RUDP::Flag::ACK)
    {
        _recvSeq[addr] = modify_uint64(_recvSeq[addr], data->rudp.sequence);
        _recvAck[addr] = modify_uint64(_recvAck[addr], data->rudp.ack);
        LIB_LOGV("(%u) %llu/%llu", data->tid, _recvSeq[addr], _recvAck[addr]);
    }
}

void Transceiver::on_notify(const Notify notify, const void* arg)
{
    lock_guard _(_sem);
    auto paddr = (const MACAddress*)arg;
    
    switch(notify)
    {
    case Notify::ConnectionLost:
        clear(*paddr);
        break;
    default:
        break;
    }
}

#if !defined(NDEBUG)
String Transceiver::debugInfo() const
{
    lock_guard _(_sem);
    String s;
    s = formatString("ID:%u SEQ:%llu\n", _tid, _sequence);
    for(auto& it :  _recvSeq)
    {
        s += formatString("  Seq:[%s]:%llu\n", it.first.toString().c_str(), it.second);
    }
    for(auto& it :  _recvAck)
    {
        s += formatString("  ACK:[%s]:%llu\n", it.first.toString().c_str(), it.second);
    }
    s.trim();
    return s;
}
#endif
//
}}
