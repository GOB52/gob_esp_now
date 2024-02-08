/*!
  @file gob_esp_now.cpp
  @brief ESP-NOW wrapper, helper and utilities.
*/
#include "gob_esp_now.hpp"
#include "internal/gob_esp_now_log.hpp"
#include <esp32-hal.h> // millis
#include <ctime>
#include <cstdio>
#include <set>
#if !defined(NDEBUG)
#include <esp_random.h>
#endif

namespace
{
using namespace goblib::esp_now;

template<typename ...Args> String formatString(const char* fmt, Args... args)
{
    size_t sz = snprintf(nullptr, 0U, fmt, args...); // calculate length
    char buf[sz + 1];
    snprintf(buf, sizeof(buf), fmt, args...);
    return String(buf, sz);
}

// For debug
#if !defined(NDEBUG)
class profile
{
  public:
    explicit profile(int64_t& out) : _save(out) { _start_at = esp_timer_get_time(); }
    ~profile() {_save = esp_timer_get_time() - _start_at; }
  private:
    int64_t _start_at{};
    int64_t& _save;
};

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
PROGMEM const char notifyDisconnect[] = "DISCONNECT";
PROGMEM const char notifyConnectionLost[] = "CONNECTION_LOST";
PROGMEM const char* notifyStringTable[] =
{
    notifyDisconnect,
    notifyConnectionLost,
};

inline const char* notify_to_cstr(const Notify notify) { return notifyStringTable[to_underlying(notify)]; }
inline const char* err2cstr(esp_err_t e) { return (e == ESP_OK) ? err_ok : errTable[e - ESP_ERR_ESPNOW_BASE]; }
inline float randf() { return esp_random() / (float)UINT32_MAX; } // [0.0, 1.0]

String packet_to_str(const void* packet)
{
    String s;
    if(!packet) { return s; }

    auto ch = (const CommunicatorHeader*)packet;
    s += formatString("CH %u:%u:%u ",ch->app_id, ch->count, ch->size);

    auto cnt = ch->count;
    auto p = ((const uint8_t*)ch) + sizeof(*ch);
    while(cnt--)
    {
        auto th = (const TransceiverHeader*)p;
        s += formatString("[%u:%u(0x%02x:%u:%u:%u)] ",
                          th->tid, th->size,
                          th->rudp.flag, th->rudp.sequence, th->rudp.ack, th->payloadSize());
        p += th->size;
    }
    s.trim();
    return s;
}

String packet_to_str(const std::vector<uint8_t>& packet)
{
    return packet_to_str(packet.empty() ? nullptr : packet.data());
}

// Iteration of tranceiver data in packet.
template<typename Func, typename... Args>
void for_each_packet(const std::vector<uint8_t>& packet, Func func, Args&&... args)
{
    if(packet.empty()) { return; }
    auto p = packet.data();
    auto ch = (const CommunicatorHeader*)p;
    auto cnt = ch->count;
    p += sizeof(Communicator);
    while(cnt--)
    {
        auto th = (const TransceiverHeader*)p;
        func(th, std::forward<Args>(args)...);
        p += th->size;
    }
}

// Append queue data
std::vector<uint8_t> append_queue(std::vector<uint8_t>& a, std::vector<uint8_t>& b)
{
    if(b.empty()) { return a; }
    if(a.empty()) { return b; }

    auto ch = (CommunicatorHeader*)a.data();
    auto b_ch = (CommunicatorHeader*)b.data();
    auto s = b.data() + sizeof(*b_ch);
    auto ssz = b_ch->size - sizeof(*b_ch);
    auto osz = a.size();

    // TODO: check size!
    a.resize(osz + ssz);
    std::memcpy(a.data() + osz, s, ssz);
    ch->count += b_ch->count;
    ch->size = a.size();
    return a;
}

#else
inline const char* err2cstr(esp_err_t) { return ""; }
inline const char* notify_to_cstr(const Notify notify) { return ""; }
#endif

bool initialize_esp_now()
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
        return r == ESP_OK;
    }
    return true;
}

// Remove that do not need to be resended
bool remove_not_need_resend(std::vector<uint8_t>& packet)
{
    if(packet.empty()) { return false; }

    auto p = packet.data();
    auto ch = (CommunicatorHeader*)p;
    auto cnt = ch->count;
    p += sizeof(CommunicatorHeader);

    //LIB_LOGD("CH0:%u:%u", ch->count, ch->size);
    while(cnt--)
    {
        auto th = (TransceiverHeader*)p;
        // unreliable or NUL or only ACK with no payload
        //if(th->isUnreliable() || th->isNUL() || (th->onlyACK() && !th->hasPayload()) )
        // unreliable or only ACK with no payload
        if(th->isUnreliable() ||  (th->onlyACK() && !th->hasPayload()) )
        {
            --ch->count;
            ch->size -= th->size;
            auto rs = packet.begin() + (p - packet.data());
            auto re = packet.begin() + (p + th->size - packet.data());
            auto it = packet.erase(rs, re);
            p = packet.data() + std::distance(packet.begin(), it);
            continue;
        }
        p += th->size;
    }
    if(ch->count == 0) { packet.clear(); }
    //LIB_LOGD("CH1:%u:%u", ch->count, ch->size);
    return !packet.empty();
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
    _sem = xSemaphoreCreateRecursiveMutex();
    assert(_sem);

    _permitToSend = xQueueCreate(1, 0U);;
    assert(_permitToSend);
    
    _addr.get(ESP_MAC_WIFI_STA);
    _sysTransceiver = new Transceiver();
    assert(_sysTransceiver);
    _transceivers.push_back(_sysTransceiver);
}

bool Communicator::begin(const uint8_t app_id, const config_t& cfg)
{
    if(_began) { return true; }

    lock_guard _(_sem);
    
    // Initialize ESP-NOW if it has not already been initialized.
    if(!initialize_esp_now()) { return false; }

    if((esp_now_register_send_cb(callback_onSent) != ESP_OK)
       || (esp_now_register_recv_cb(callback_onReceive) != ESP_OK) )
    {
        LIB_LOGE("Failed to register cb");
        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
        return false;
    }

    _app_id = app_id;
    for(auto& t : _transceivers) { t->with_lock([&t](){ t->build_peer_map(); }); }

    esp_now_peer_info_t info{};
    auto rt = millis();
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            MACAddress addr(info.peer_addr);
            auto& md = _queue[addr];
            md.reserve(ESP_NOW_MAX_DATA_LEN);
            _state[addr].recvTime = rt; // Provisional settings
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }

    _config = cfg;
    _began = true;
    return true;
}

void Communicator::end()
{
    lock_guard _(_sem);
    if(_began)
    {
        _began = false;
        _transceivers.clear();
        _transceivers.push_back(_sysTransceiver);
        _lastSentTime = 0;
        clearPeer();
        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
    }
}

// TODO
// SYN パラメータの Comm反映
// SYN-SYN+ACK-ACK は その前に SYNするかを問い合わせる専用BROADCAST
void Communicator::update()
{
    if(!_began) { return; }

    auto ms = millis();
    unsigned long startTime{};
    bool sent{};

    {
        lock_guard _(_sem);

        // Remove acked data
        // If we don't do it here, the TRX update will make it easier to overflow the posted data.
        for(auto& q : _queue) { remove_acked(q.first, q.second); }

        // Update transceivers
        for(auto& t : _transceivers)
        {
            t->_update(ms, _config);
            t->update(ms);
        }
        //    if(!_canSend || numOfPeer() == 0) { return; }

        // Detect communication failure
        for(auto& ss : _state)
        {
            if(!ss.first) { continue; }
            if(ss.second.retry > _config.maxRetrans)
            {
                LIB_LOGD("<Exceeded retry count>");
                notify(Notify::ConnectionLost, &ss.first);
            }
        }
    
        // Send if exists queue (Order by MACAddresss ASC)
        // first : MACAddress
        // second : packet
        for(auto& q : _queue)
        {
            //        if(!_canSend || q.second.empty() || !_state.count(q.first)) { continue; }
            if(q.second.empty() || !_state.count(q.first)) { continue; }

            // Send / resend
            if(_state[q.first].state == State::None ||
               (/*_state[q.first].state != State::None &&*/
                   ms > _state[q.first].sentTime + _config.retransmissionTimeout))
            {
                if(_state[q.first].state != State::None) { LIB_LOGE("resend: %d", ms > _state[q.first].sentTime + _config.retransmissionTimeout); }

                LIB_LOGD("---- %s:%u:[%s]",
                         _state[q.first].state == State::None ? "Send" : "Resend",
                         _state[q.first].retry,q.first.toString().c_str());

                startTime = micros();
                sent = send_esp_now((bool)q.first ? q.first.data() : nullptr, q.second);
                if(!sent) { continue; }
                if(_state[q.first].state == State::None) { _state[q.first].retry = 0; }
                else { ++_state[q.first].retry; }
                break;
            }
        }
    }
    // Wait onSent
    if(sent)
    {
        xQueueReceive(_permitToSend, nullptr, portMAX_DELAY);
        auto oneTime = micros() - startTime;
        _time += oneTime;
        _minTime = std::min(oneTime, _minTime);
        _maxTime = std::max(oneTime, _maxTime);
    }

    lock_guard _(_sem);
    // Detect communication failures and heartbeat
    //    if(!_canSend || _transceivers.size() < 2 || !_config.nullSegmentTimeout) { return; }
    if(_transceivers.size() < 2 || !_config.nullSegmentTimeout) { return; }

    for(auto& ss : _state)
    {
        if(!ss.first) { continue; }
        // Send a NUL if I am the client when a null timeout occurs
        if(isSecondary()
           && (_queue.count(ss.first) && _queue.at(ss.first).empty())
           && ms > ss.second.sentTime + _config.nullSegmentTimeout)
        {
            LIB_LOGD("[NUL]");
            _sysTransceiver->post_nul(ss.first);
        }
        // If I have not received for double the time of the NUL timeout
        if(isPrimary()
           && ss.second.sentTime < ss.second.recvTime
           && ms > ss.second.recvTime + _config.nullSegmentTimeout * 2)
        {
            LIB_LOGE("<NUL never come>");
            notify(Notify::ConnectionLost, &ss.first);
        }
    }
}

// Post
bool Communicator::post(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { LIB_LOGE("peer_addr not exists"); return false; }

    MACAddress addr(peer_addr);

    auto& packet = _queue[addr]; // Create empty packet if not exiets.
    if(packet.capacity() < ESP_NOW_MAX_DATA_LEN) { packet.reserve(ESP_NOW_MAX_DATA_LEN); } // Expand memory

    // First time?
    if(packet.empty())
    {
        CommunicatorHeader ch{};
        ch.app_id = _app_id;
        packet.resize(sizeof(CommunicatorHeader));
        std::memcpy(packet.data(), &ch, sizeof(ch));
    }
    
    // Overflow?
    if(length + packet.size() > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Packet overflow"); return false; }

    // Append data
    auto p = packet.data();
    auto ch = (CommunicatorHeader*)p;
    auto osz = packet.size();
    packet.resize(osz + length);
    assert(packet.data() == p && "Pointer changed");
    std::memcpy(p + osz, data, length);
    ++ch->count;
    ch->size = packet.size();
    
    reset_sent_state(addr);
    //LIB_LOGD("CH:%u:%u:%zu", ch->count, ch->size,md.size());
    return true;
}

#if 0
// Send directly
bool Communicator::send(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    lock_guard _(_sem);

    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { LIB_LOGE("peer_addr not exists"); return false; }

    if(!_began || !_canSend) { LIB_LOGE("Not ready to send"); return false; }
    if(sizeof(CommunicatorHeader) + length > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Packet overflow"); return false; }

    std::vector<uint8_t> v;
    v.reserve(ESP_NOW_MAX_DATA_LEN);
    
    CommunicatorHeader ch{};
    ch.app_id = _app_id;
    ch.count = 1;
    ch.size = sizeof(ch) + length;
    v.resize(ch.size);
    auto dst = v.data();
    std::memcpy(dst, &ch, sizeof(ch));
    std::memcpy(dst + sizeof(ch), data, length);

    reset_sent_state(MACAddress(peer_addr));
    return send_esp_now(peer_addr, v);
}
#endif

// Call with communicator locked.
bool Communicator::send_esp_now(const uint8_t* peer_addr, std::vector<uint8_t>& packet)
{
    MACAddress addr(peer_addr);
    if(((bool)addr && !existsPeer(addr)) || packet.empty() || ((CommunicatorHeader*)packet.data())->count == 0)
    {
        LIB_LOGW("No peer or empty packet %s", addr.c_str());
        return false;
    }

#if 0    
#if !defined(NDEBUG)
    if(isEnableDebug() && randf() < _debugSendLoss)
    {
        remove_not_need_resend(packet);
        onSent(addr, ESP_NOW_SEND_FAIL);
        LIB_LOGD("[DF]:FailedS");
        return false;
    }
#endif
#endif
    
    auto ret = esp_now_send(peer_addr, packet.data(), packet.size());
    //    remove_not_need_resend(packet);
    if(ret != ESP_OK)
    {
        LIB_LOGE("Failed to esp_now_send %s %d:%s", MACAddress(peer_addr).toString().c_str(), ret, err2cstr(ret));
        return false;
    }
    _lastSentTime = millis();
    _lastSentAddr = addr;
    _lastSentQueue = packet;
    packet.clear();
    //    _canSend = false;

    return true;
}

// Remove data that has already been received by the other device
// return true if data available
// Call with communicator locked.
bool Communicator::remove_acked(const MACAddress& addr, std::vector<uint8_t>& packet)
{
    if(packet.empty()) { return false; }
    assert(packet.size() >= sizeof(CommunicatorHeader) && "illgal size");

    //LIB_LOGD(" -> packet:[%s]", packet_to_str(packet).c_str());
    auto p = packet.data();
    auto ch = (CommunicatorHeader*)p;
    auto cnt = ch->count;
    p += sizeof(CommunicatorHeader);

    auto oldCount = cnt;
    
    while(cnt--)
    {
        auto th = (TransceiverHeader*)p;
        auto t = transceiver(th->tid);
        if(!t) { LIB_LOGE("No transceiver %u", th->tid); continue; }

        if(th->isACK())
        {
            bool acked = t->with_lock([&t,&th,&addr]()
            {
                return (th->hasPayload() || th->isNUL()) && t->delivered(th->rudp.sequence, addr);
            });

            if(acked)
            {
                ch->count--;
                ch->size -= th->size;
                auto rs = packet.begin() + (p - packet.data());
                auto re = packet.begin() + (p + th->size - packet.data());
                auto it = packet.erase(rs, re);
                p = packet.data() + std::distance(packet.begin(), it);
                continue;
            }
        }
        p += th->size;
    }
    if(ch->count != oldCount)
    {
        LIB_LOGD("[%s] CH:%u -> %u", addr.toString().c_str(), oldCount, ch->count);
    }
    if(ch->count == 0) { packet.clear(); }

    //LIB_LOGD(" => packet:[%s]", packet_to_str(packet).c_str());
    return !packet.empty();
}

void Communicator::reset_sent_state(const MACAddress& addr)
{
#if 0
    if((bool)addr) { _state[addr].state = State::None; }
    else           { for(auto& ss : _state) { ss.second.state = State::None; } }
#endif
    _state[addr].state = State::None;
}


// Call with communicator locked.
void Communicator::notify(const Notify n, const void* arg)
{
    auto paddr = (const MACAddress*)arg;
    switch(n)
    {
    case Notify::Disconnect:
        LIB_LOGI("%s[%s]", notify_to_cstr(n), paddr->toString().c_str());
        unregisterPeer(*paddr);
        break;
    case Notify::ConnectionLost:
        LIB_LOGI("%s[%s]", notify_to_cstr(n), paddr->toString().c_str());
        unregisterPeer(*paddr);
        break;
    default:
        LIB_LOGE("%s", "UNKNOWN");
        break;
    }
    if(_notifyFunction) { _notifyFunction(n, arg); }
}

Transceiver* Communicator::transceiver(const uint8_t tid)
{
    auto it = std::find_if(_transceivers.begin(), _transceivers.end(), [&tid](Transceiver* t) { return t->identifier() == tid; });
    return it != _transceivers.end() ? *it : nullptr;
}

bool Communicator::registerTransceiver(Transceiver* t)
{
    if(!t) { return false; }

    // Already registered?
    auto it = std::find(_transceivers.begin(), _transceivers.end(), t);
    if(it != _transceivers.end()) { LIB_LOGE("Already registered %u", t->identifier()); return false; }

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
    t->build_peer_map();
    return true;
}

bool Communicator::unregisterTransceiver(Transceiver* t)
{
    if(t == _sysTransceiver) { LIB_LOGE("system transceiver"); return false; }

    auto it = std::remove(_transceivers.begin(), _transceivers.end(), t);
    if(it == _transceivers.end()) { LIB_LOGE("Not found"); return false; }
    _transceivers.erase(it, _transceivers.end());
    return true;
}

bool Communicator::registerPeer(const MACAddress& addr, const uint8_t channel, const bool encrypt, const uint8_t* lmk)
{
    if(!addr) { LIB_LOGE("Null address"); return false; }

    // Initialize ESP-NOW if it has not already been initialized.
    if(!initialize_esp_now()) { return false; }

    esp_err_t ret{ESP_OK};
    if(!esp_now_is_peer_exist(addr.data()))
    {
        LIB_LOGE("register [%s] %d/%d", addr.toString().c_str(), addr.isMulticast(), addr.isUniversal());

        esp_now_peer_info_t info{};
        std::memcpy(info.peer_addr, addr.data(), sizeof(info.peer_addr));
        info.channel = channel;
        info.encrypt = encrypt;
        if(encrypt) { std::memcpy(info.lmk, lmk, sizeof(info.lmk)); }

        ret = esp_now_add_peer(&info);
        if(ret == ESP_OK)
        {
            _state[addr] = {};
            _queue[addr] = {};
            for(auto& t : _transceivers)
            {
                t->with_lock([&t, &addr]()
                {
                    t->_peerInfo[addr] = {};
                });
            }
        }
    }
    if(ret != ESP_OK) { LIB_LOGE("Failed to add:%d [%s]", ret, err2cstr(ret)); }
    return ret == ESP_OK;;
}

void Communicator::unregisterPeer(const MACAddress& addr)
{
    if(!addr) { LIB_LOGE("Null address"); return; }

    LIB_LOGV("unregister %s", addr.toString().c_str());
    auto ret = esp_now_del_peer(addr.data());
    {
        _queue.erase(addr);
        _state.erase(addr);
        for(auto& t : _transceivers)
        {
            t->with_lock([&t, &addr]()
            {
                t->clear(addr);
            });
        }
    }
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

    _queue.clear();
    _state.clear();
    for(auto& t : _transceivers)
    {
        t->with_lock([&t]() { t->_peerInfo.clear(); });
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

void Communicator::callback_onSent(const uint8_t *peer_addr, esp_now_send_status_t status)
{
    MACAddress addr(peer_addr);
    auto& comm = instance();
    comm.onSent(addr, status);
}

void Communicator::onSent(const MACAddress& addr, const esp_now_send_status_t status)
{
    bool succeed{status == ESP_NOW_SEND_SUCCESS};
    if(!succeed) { LIB_LOGW("FAILED"); }

    lock_guard _(_sem);
    {
    // Composite the remaining elements and those added to the queue between esp_send and the callback
    if(succeed)
    {
        remove_not_need_resend(_lastSentQueue);
    }
    auto sz = _lastSentQueue.size();
    auto b = !_queue[addr].empty();
    _queue[addr] = append_queue(_lastSentQueue, _queue[addr]);
    if(b) { LIB_LOGE("%d append %zu => %zu", succeed, sz, _queue[addr].size()); }

    if(_state.count(addr))
    {
        _state[addr].state = (State::Status)((uint8_t)State::Status::Succeed + (succeed ? 0 : 1));
        _state[addr].sentTime = millis();
    }
    }
    //    _canSend = true;
    xQueueSend(_permitToSend, nullptr, 0);
}

void Communicator::callback_onReceive(const uint8_t* peer_addr, const uint8_t* data, int length)
{
    if(esp_now_is_peer_exist(peer_addr))
    {
        MACAddress addr(peer_addr);
        instance().onReceive(addr, data, length);
    }
}

void Communicator::onReceive(const MACAddress& addr, const uint8_t* data, const uint8_t length)
{
    auto ch = (const CommunicatorHeader*)data;
    // Check 
    if(ch->signeture != CommunicatorHeader::SIGNETURE
       || ch->app_id != _app_id) { LIB_LOGE("Illegal data"); return; }

#if 0
#if !defined(NDEBUG)
    if(isEnableDebug() && randf() < _debugRecvLoss)
    {
        LIB_LOGD("[DF]:FailedR tid:%u", t->identifier());
        return;
    }
#endif
#endif
    LIB_LOGD("---- RECV:[%s]", addr.toString().c_str());

    lock_guard _(_sem);
    
    auto ms = millis();
    _state[addr].recvTime = ms;

    auto cnt = ch->count;
    data += sizeof(CommunicatorHeader);
    while(cnt--)
    {
        auto th = (const TransceiverHeader*)data;
        for(auto& t : _transceivers)
        {
            if(t->identifier() == th->tid)
            {
                // Old or future incoming data is not processed if RUDP ACK
                auto correct = !th->isACK() ||
                        t->with_lock([&t, &ch, &th, &addr]()
                        {
                            auto rseq = (th->hasPayload() || th->isNUL()) ?
                                    t->_peerInfo[addr].recvSeq : t->_peerInfo[addr].recvAckSeq;

                            auto seq = restore_u64_later(rseq, th->rudp.sequence);
                            // Correct order?
                            return seq == rseq + 1;
                        });
                
                if(correct)
                {
                    t->on_receive(addr, th);
                    if(th->hasPayload()) { t->onReceive(addr, th->payload(), th->payloadSize()); }
                }
                else
                {
                    auto rs = (th->hasPayload() || th->isNUL()) ?
                            t->_peerInfo[addr].recvSeq : t->_peerInfo[addr].recvAckSeq;
                    LIB_LOGW("[Rejected[%u] P:%d %u/%llu(%llu)",
                             t->_tid,
                             th->hasPayload(),
                             th->rudp.sequence, rs, rs % 256);
                }
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
    return debug_info();
}
                   
String Communicator::debug_info() const
{
    String s;
    s += formatString("Communicator app_id:%u, %zu transceivers\n", _app_id, _transceivers.size());
    s += formatString("config:<%u/%u/%u/%u %u:%u:%u:%u>\n",
                      _config.retransmissionTimeout,
                      _config.cumulativeAckTimeout,
                      _config.nullSegmentTimeout,
                      _config.transferStateTimeout,
                      _config.maxRetrans,
                      _config.maxCumAck,
                      _config.maxOutOfSeq,
                      _config.maxAutoReset);
    
    for(auto& t : _transceivers)
    {
        s += t->debugInfo() + '\n';
    }

    s += "Peerlist(ESP-NOW)\n";
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            MACAddress addr(info.peer_addr);
            s += formatString("  [%s]\n",addr.toString().c_str());
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }

    s += formatString("Queue:%zu\n", _queue.size());
    for(auto& q : _queue)
    {
        if(!q.second.empty())
        {
            s += formatString("  [%s] %s\n", q.first.toString().c_str(), packet_to_str(q.second).c_str());
        }
        else
        {
            s += formatString("  [%s]\n", q.first.toString().c_str());
        }
    }
    s += formatString("State:%zu\n", _state.size());
    for(auto& st : _state)
    {
        s += formatString("  [%s] st:%u rty:%u ST:%lu RT:%lu\n",
                          st.first.toString().c_str(),
                          st.second.state, st.second.retry, st.second.sentTime, st.second.recvTime);
    }

    s += formatString("minTime:%lu maxTime:%lu\n", _minTime, _maxTime);
    s.trim();
    return s;
}
#endif

// -----------------------------------------------------------------------------
// class Transceiver
Transceiver::Transceiver()
{
    _sem = xSemaphoreCreateRecursiveMutex();
    assert(_sem);
}

Transceiver::Transceiver(const uint8_t tid) : _tid(tid)
{
    _sem = xSemaphoreCreateRecursiveMutex();
    assert(_sem);
}

Transceiver::~Transceiver()
{
    vSemaphoreDelete(_sem);
}

void Transceiver::reset()
{
    for(auto& it : _peerInfo) { it.second = {}; }
}

void Transceiver::clear(const MACAddress& addr)
{
    _peerInfo.erase(addr);
}

bool Transceiver::delivered(const uint64_t seq)
{
    return std::all_of(_peerInfo.begin(), _peerInfo.end(), [&seq](decltype(_peerInfo)::const_reference e)
    {
        return seq <= e.second.recvAck;
    });
}

uint64_t Transceiver::postReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return 0; }

    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    
    // Unicast
    if(peer_addr)
    {
        auto seq = make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
        return Communicator::instance().postWithLock(peer_addr, buf, sizeof(buf)) ? seq : 0;
    }
    //All peers (Separate to each peer)
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) != ESP_OK) { return 0; }

    uint64_t rseq{};
    do
    {

        auto seq = make_data(buf, RUDP::Flag::ACK, info.peer_addr, data, length);
        if(!Communicator::instance().postWithLock(info.peer_addr, buf, sizeof(buf)))
        {
            LIB_LOGE("Failed to post %s", MACAddress(info.peer_addr).toString().c_str());
            return 0;
        }
        if(!rseq) { rseq = seq; }
        if(rseq != seq)
        {
            LIB_LOGE("Inconsistency in the sequence for each peer\n"
                     "Are you mixing all peers and single peer send?");
            return 0;
        }
    }
    while(esp_now_fetch_peer(false, &info) == ESP_OK);
    return rseq;
}

#if 0
uint64_t Transceiver::sendReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(!peer_addr || (peer_addr && !esp_now_is_peer_exist(peer_addr))) { return 0; }

    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    auto seq = make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
    return Communicator::instance().sendWithLock(peer_addr, buf, sizeof(buf)) ? seq : 0;
}
#endif

bool Transceiver::postUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::NONE, peer_addr, data, length);
    return Communicator::instance().postWithLock(peer_addr, buf, sizeof(buf));
}

#if 0
bool Transceiver::sendUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::NONE, peer_addr, data, length);
    return Communicator::instance().sendWithLock(peer_addr, buf, sizeof(buf));
}
#endif

void Transceiver::build_peer_map()
{
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            _peerInfo[MACAddress(info.peer_addr)] = {};
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
}

// WARN:Will lock
bool Transceiver::post_rudp(const uint8_t* peer_addr, const RUDP::Flag flag, const void* data, const uint8_t length)
{
    if(!peer_addr) { LIB_LOGE("Need peer_addr"); return false; }
    LIB_LOGD("[RUDP]:[%s]:0x%02x", MACAddress(peer_addr).toString().c_str(), to_underlying(flag));

    uint8_t buf[sizeof(TransceiverHeader) + length];
    make_data(buf, flag, peer_addr, data, length);
    return Communicator::instance().post(peer_addr, buf, sizeof(buf));
}

// WARN:Will lock
uint64_t Transceiver::make_data(uint8_t* obuf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    MACAddress addr(peer_addr);
    if(addr.isBroadcast()) { LIB_LOGE("Prohibited addr %s", addr.toString().c_str()); return 0; }

    // Header
    TransceiverHeader* th = (TransceiverHeader*)obuf;
    th->tid = _tid;
    th->size = sizeof(*th) + length;
    th->rudp.flag = to_underlying(flag);

    uint64_t seq{};
    auto& pi = _peerInfo[addr];
    {
        lock_guard _(_sem);
        pi.needReturnACK = false;

        if(th->isNUL())
        {
            seq = ++pi.sequence;
            th->rudp.sequence = seq & 0xFF;
            pi.sentAck = pi.recvSeq;
            th->rudp.ack = (pi.recvSeq & 0xFF);
        }
        else if(th->isACK())
        {
            // TODO: ここで ++pi.sequnce して post で overflow するとやばい気がする
            // post error の場合 sequence とばないか?
            // そのばあいそもそも破綻するのだからいいのでは説
            seq = length ? ++pi.sequence : ++pi.ackSequence;
            th->rudp.sequence = seq & 0xFF;
            pi.sentAck = pi.recvSeq;
            th->rudp.ack = (pi.recvSeq & 0xFF);
        }
    }
    if(data && length) { std::memcpy(obuf + sizeof(*th), data, length); } // Append payload

    LIB_LOGD("[%s] (0x%02x:S:%u:A:%u P:%u) | S64:%llu A64:%llu",
             addr.toString().c_str(),
             flag, th->rudp.sequence, th->rudp.ack, th->payloadSize(),
             seq, pi.recvSeq);

    return seq;
}

void Transceiver::_update(const unsigned long ms, const Communicator::config_t& cfg)
{
    // Force send ACK?
    std::set<MACAddress> addrs;

    lock_guard _(_sem);

    for(auto& pi : _peerInfo)
    {
        if(!pi.first || !esp_now_is_peer_exist(pi.first.data()) || !pi.second.needReturnACK) { continue; }

        // Send ACK if nothing is sent for a certain period of time after receiving ACK with payload
        auto rtm = pi.second.recvTime;
        if(rtm && ms > rtm + cfg.cumulativeAckTimeout)
        {
            LIB_LOGD(">> ForceACK:Timeout");
            addrs.insert(pi.first);
            continue;
        }
        // Post ACK if the number of unrespond ACKs exceeds a certain number
        if(pi.second.recvSeq > pi.second.sentAck + cfg.maxCumAck)
        {
            LIB_LOGD(">> FoeceACK:Cum");
            addrs.insert(pi.first);
        }
    }
    for(auto& addr : addrs) { post_ack(addr.data()); }
}

void Transceiver::on_receive(const MACAddress& addr, const TransceiverHeader* th)
{
    uint64_t rs{},ra{};
    auto& pi = _peerInfo[addr];
    {
        lock_guard _(_sem);
        pi.recvTime = millis();
    }

    auto base = (th->hasPayload() || th->isNUL()) ? pi.recvSeq : pi.recvAckSeq;
    rs = restore_u64_later(base, th->rudp.sequence);
    ra = restore_u64_later(pi.recvAck, th->rudp.ack);

    LIB_LOGD("[RECV]:%u 0x%02x S:%u A:%u P:%u -> S:%llu => %llu A:%llu => %llu",
             th->tid, th->rudp.flag, th->rudp.sequence, th->rudp.ack, th->payloadSize(),
             base, rs, pi.recvAck, ra);
    
    // isACK include NUL
    if(th->isACK())
    {
        lock_guard _(_sem);
        LIB_LOGD("<%s>:%u", th->isNUL() ? "NUL" : "ACK", th->payloadSize());
        ((th->hasPayload() || th->isNUL()) ?  pi.recvSeq : pi.recvAckSeq) = rs;
        if(ra > pi.recvAck) { pi.recvAck = ra; }
        pi.needReturnACK |= (th->hasPayload() || th->isNUL());
    }
    if(th->isRST())
    {
        LIB_LOGD("<RST>");
        auto& comm = Communicator::instance();
        comm.notify(Notify::Disconnect, &addr);
    }
    if(th->isNUL())
    {
        post_ack(addr);
    }
}

#if !defined(NDEBUG)
String Transceiver::debugInfo() const
{
    lock_guard _(_sem);
    String s;
    s = formatString("TID:%u\n", _tid);
    for(auto& r : _peerInfo)
    {
        s += formatString("  [%s] S(S:%llu SA;%llu A:%llu) R(S:%llu A:%llu T:%lu) NRA:%d\n",
                          r.first.toString().c_str(),
                          r.second.sequence, r.second.ackSequence, r.second.sentAck,
                          r.second.recvSeq, r.second.recvAck, r.second.recvTime,
                          r.second.needReturnACK
                          );

    }
    s.trim();
    return s;
}
#endif
//
}}

