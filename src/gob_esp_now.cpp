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
PROGMEM const char* notifyTable[] =
{
    notifyDisconnect,
    notifyConnectionLost,
};

inline const char* notify_to_cstr(const Notify notify) { return notifyTable[to_underlying(notify)]; }
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
        s += formatString("[%u:%u(0x%02x:%u:%u)] ",th->tid, th->size, th->rudp.flag, th->rudp.sequence, th->rudp.ack);
        p += th->size;
    }
    s.trim();
    return s;
}

String packet_to_str(const std::vector<uint8_t>& packet)
{
    return packet_to_str(packet.empty() ? nullptr : packet.data());
}

#if 0
uint8_t append_packet(std::vector<uint8_t>& dv /*packet*/, std::vector<uint8_t>& sv /*packet*/)
{
    auto dst = dv.data();
    auto dch = (CommunicatorHeader*)dst;
    auto src = sv.data();
    auto sch = (CommunicatorHeader*)src;

    if(sv.empty() || src == dst) { return dch->count; }

    if(((uint16_t)dch->size) + sch->size > ESP_NOW_MAX_DATA_LEN)
    {
        LIB_LOGE("overflow");
        return dch->count;
    }
    //    LIB_LOGD("DST:%s", packet_to_str(dv).c_str());
    //    LIB_LOGD("SRC:%s", packet_to_str(sv).c_str());

    dch->count += sch->count;
    auto osz = dch->size;
    auto stsz = sch->size - sizeof(CommunicatorHeader); // only Tranceiver data
    dch->size += stsz;
    src += sizeof(CommunicatorHeader);

    dv.resize(dch->size); 
    // data() is not changed after resize because it is assumed to be reserved(ESP_NOW_MAX_DATA_LEN) (it should be).
    assert(dv.data() == dst && "pointer changed");

    std::memcpy(dst + osz, src, stsz);
    //    LIB_LOGD("==> RES:%s", packet_to_str(dv).c_str());
    
    return dch->count;
}
#endif

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

#else
inline const char* err2cstr(esp_err_t) { return ""; }
inline const char* notify_to_cstr(const Notify notify) { return ""; }
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
        // Unreliable? Not ACK? ACK with no payload?
        if(th->isUnreliable() || !th->isACK() || !th->hasPayload())
        {
            ch->count--;
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
    _sem = xSemaphoreCreateBinary();
    xSemaphoreGive(_sem);
    _addr.get(ESP_MAC_WIFI_STA);
}

bool Communicator::begin(const uint8_t app_id, const config_t* cfg)
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
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            MACAddress addr(info.peer_addr);
            auto& md = _queue[addr];
            md.reserve(ESP_NOW_MAX_DATA_LEN);
            _sentState[addr] = {};
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }

    if(!cfg) { _config = config_t::defaultValue(); }
    else     { _config = *cfg; }

    return true;
}


void Communicator::end()
{
    if(_began)
    {
        lock_guard _(_sem);
        _began = false;
        _transceivers.clear();
        _queue.clear();
        _sentState.clear();
        _lastSentTime = 0;

        // TODO
        // unregisterPeer()? for(auto& r : _peerInfo) {...}
        
        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
    }
}

// TODO
// SYN パラメータの Comm反映
// SYN-SYN+ACK-ACK は その前に SYNする?BROADCAST

void Communicator::update()
{
    if(!_began) { return; }

    auto ms = millis();

    lock_guard _(_sem);

    // Update transceivers
    for(auto& t : _transceivers)
    {
        t->_update(ms, _sentState, _config);
        t->update(ms);
    } 

    if(!_canSend) { return; }

    // Send if exists queue (Order by MACAddresss ASC, Null MACAddress is top)
    for(auto& q : _queue)
    {
        // Remove acked data
        if(!remove_acked(q.first, q.second)) { _sentState[q.first].reset(); continue; }
        // Transmission is not possible before receiving the transmission callback
        if(!_canSend){ continue; }

        // LIB_LOGD("  >[%s] st:%d", q.first.toString().c_str(), _sentState[q.first].state);
        // Send or Resend
        if(_sentState[q.first].state == SendState::None ||
           (_sentState[q.first].state != SendState::None &&
            ms > _sentState[q.first].sentTime + _config.retransmissionTimeout))
        {
            LIB_LOGD("---- %s:[%s]",
                     _sentState[q.first].state == SendState::None ? "Send" : "Resend", q.first.toString().c_str());

            if(_sentState[q.first].state == SendState::None) { _sentState[q.first].retry = 0; }
            else                                             { ++_sentState[q.first].retry;   }
            if(!q.first) // All peer?
            {
                for(auto& ss :_sentState) { if((bool)ss.first) { ss.second.retry = _sentState[q.first].retry; } }
            }
            
            send_esp_now((bool)q.first ? q.first.data() : nullptr, q.second); // Set _canSend = false if sent
        }
    }
}

// Post
bool Communicator::post(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }

    MACAddress addr(peer_addr);
    auto& packet = _queue[addr]; // Create empty packet if not exiets.
    if(packet.capacity() < ESP_NOW_MAX_DATA_LEN) { packet.reserve(ESP_NOW_MAX_DATA_LEN); } // Expand memory

    if(packet.empty())
    {
        CommunicatorHeader ch{};
        ch.app_id = _app_id;
        packet.resize(sizeof(CommunicatorHeader));
        std::memcpy(packet.data(), &ch, sizeof(ch));
    }
    
    // Overflow
    // TODO: ..... 蓄積と再ポストはどうする?
    if(length + packet.size() > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Overflow"); return false; }

    // Append data
    auto p = packet.data();
    auto ch = (CommunicatorHeader*)p;
    auto osz = packet.size();
    packet.resize(osz + length);
    assert(packet.data() == p && "pointer changed");
    std::memcpy(p + osz, data, length);
    ++ch->count;
    ch->size = packet.size();

    reset_sent_state(addr);
    //LIB_LOGD("CH:%u:%u:%zu", ch->count, ch->size,md.size());
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
    
    CommunicatorHeader ch{};
    ch.app_id = _app_id;
    ch.count = 1;
    ch.size = sizeof(ch) + length;
#if 1
    v.resize(ch.size);
    auto dst = v.data();
    std::memcpy(dst, &ch, sizeof(ch));
    std::memcpy(dst + sizeof(ch), data, length);
#else
    v.insert(v.end(), (uint8_t*)&ch, (uint8_t*)&ch + sizeof(ch));
    v.insert(v.end(), (uint8_t*)data, (uint8_t*)data + length);
#endif

    reset_sent_state(MACAddress(peer_addr));
    return send_esp_now(peer_addr, v);
}

// Call with communicator locked.
bool Communicator::send_esp_now(const uint8_t* peer_addr, std::vector<uint8_t>& packet)
{
    MACAddress addr(peer_addr);

    if(packet.empty() || ((CommunicatorHeader*)packet.data())->count == 0)
    {
        LIB_LOGW("empty");
        return false;
    }
    
#if !defined(NDEBUG)
    if(isEnableDebug() && randf() < _debugSendLoss)
    {
        remove_not_need_resend(packet);
        onSent(addr, ESP_NOW_SEND_FAIL);
        LIB_LOGD("[DF]:FailedS");
        return false;
    }
#endif
    auto ret = esp_now_send(peer_addr, packet.data(), packet.size());
    if(ret != ESP_OK)
    {
        LIB_LOGE("Failed to esp_now_send %s %d:%s", MACAddress(peer_addr).toString().c_str(), ret, err2cstr(ret));
        remove_not_need_resend(packet);
        return false;
    }

    _lastSentTime = millis();
    _lastSentAddr = addr;
    _canSend = false;
    remove_not_need_resend(packet);
    return true;
}

#if 0
// Transmitted data append them to _sentQueue
void Communicator::append_to_sent(const uint8_t* peer_addr, std::vector<uint8_t>& packet)
{
#if 0
    MACAddress addr(peer_addr);
    if(_sentQueue[addr].data() == packet.data()) { return; }

    //LIB_LOGD("[%s]", addr.toString().c_str());
    //remove_acked(addr, _sentQueue[addr]);
    if(_sentQueue[addr].empty())
    {
        _sentQueue[addr] = std::move(packet);
        //LIB_LOGV("==> move %s", packet_to_str(_sentQueue[addr]).c_str());
        return;
    }
    if(!append_packet(_sentQueue[addr], packet)) { _sentQueue[addr].clear(); }
#endif
}
#endif

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
        assert(t && "No transceiver");
        if(th->isACK())
        {
#if 0
            LIB_LOGD("  >S:%u => (%llu)%llu ack:%llu",
                     th->rudp.sequence,
                     t->_sequence,
                     restore_u64_earlier(t->_sequence, th->rudp.sequence),
                     t->_peerInfo[addr].ack);
#endif
            bool acked = t->with_lock([&t,&th,&addr]()
            {
                return (bool)addr ? t->peerReceived(th->rudp.sequence, addr) : t->peerReceived(th->rudp.sequence);
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
    if(ch->count != oldCount) { LIB_LOGD("[%s] CH:%u -> %u", addr.toString().c_str(), oldCount, ch->count); }
    if(ch->count == 0) { packet.clear(); }

    //LIB_LOGD(" => packet:[%s]", packet_to_str(packet).c_str());
    return !packet.empty();
}

void Communicator::reset_sent_state(const MACAddress& addr)
{
    if((bool)addr) { _sentState[addr].state = SendState::None; }
    else           { for(auto& ss : _sentState) { ss.second.state = SendState::None; } }
}


// Call with communicator locked.
void Communicator::notify(const Notify n, const void* arg)
{
    LIB_LOGI("%s", notify_to_cstr(n));

    auto paddr = (const MACAddress*)arg;
    switch(n)
    {
    case Notify::Disconnect:
    case Notify::ConnectionLost:
        LIB_LOGI("[%s]", paddr->toString().c_str());
        unregisterPeer(*paddr);
        _queue.erase(*paddr);
        _sentState.erase(*paddr);
        break;
    }
    for(auto& t : _transceivers)
    {
        t->on_notify(n, arg);
        t->onNotify (n, arg);
    }
}

Transceiver* Communicator::transceiver(const uint8_t tid)
{
    auto it = std::find_if(_transceivers.begin(), _transceivers.end(), [&tid](Transceiver* t) { return t->identifier() == tid; });
    return it != _transceivers.end() ? *it : nullptr;
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

void Communicator::callback_onSent(const uint8_t *peer_addr, esp_now_send_status_t status)
{
    MACAddress addr(peer_addr);
    auto& comm = instance();
    comm.with_lock([&comm, &addr, &status](){ comm.onSent(addr, status); });
}

void Communicator::onSent(const MACAddress& addr, const esp_now_send_status_t status)
{
    LIB_LOGD("%s [%s]", status == ESP_NOW_SEND_SUCCESS ? "SEND_SUCCESS" : "SEND_FAILED",
             addr.toString().c_str());

    _sentState[addr].state = (SendState::State)((int)SendState::Succeed + (status != ESP_NOW_SEND_SUCCESS));
    _sentState[addr].sentTime = millis();

    if(!_lastSentAddr)
    {
        if(_sentState[addr].state > _sentState[_lastSentAddr].state ) { _sentState[_lastSentAddr].state = _sentState[addr].state; }
        _sentState[_lastSentAddr].sentTime = _sentState[addr].sentTime;
    }
    // When sending to all peers, until all callbacks are called
    _canSend = (bool)_lastSentAddr ? true : std::all_of(_sentState.begin(), _sentState.end(),
                                               [](decltype(_sentState)::const_reference ss)
                                               {
                                                   return !ss.first || ss.second.state != SendState::None;
                                               });
}

void Communicator::callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length)
{
    MACAddress addr(mac_addr);
    instance().onReceive(addr, data, length);
}

void Communicator::onReceive(const MACAddress& addr, const uint8_t* data, const uint8_t length)
{
    LIB_LOGD("---- RECV:[%s]", addr.toString().c_str());
    
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
                if(isEnableDebug() && randf() < _debugRecvLoss)
                {
                    LIB_LOGD("[DF]:FailedR tid:%u", t->identifier());
                    continue;
                }
#endif
                //LIB_LOGD("(%u)[%s]", th->tid, addr.toString().c_str());
                // Old or future incoming data is not processed if RUDP ACK
                auto b = !th->isACK() || t->with_lock([&t,&th,&addr]()
                {
                    //LIB_LOGD("%llu:%u => %llu", t->_peerInfo[addr].sequence, th->rudp.sequence, restore_u64_later(t->_peerInfo[addr].sequence, th->rudp.sequence));
                    auto rseq = restore_u64_later(t->_peerInfo[addr].sequence, th->rudp.sequence);
                    // First time receiving or accept if in correct order.
                    return rseq == (t->_peerInfo[addr].sequence + 1) || t->_peerInfo[addr].sequence == 0;
                });
                
                if(b)
                {
                    t->on_receive(addr, th);
                    t->onReceive(addr, th);
                }
                else
                {
                    //LIB_LOGW("Old or future data has come.%u / %llu",th->rudp.sequence, t->_peerInfo[addr].sequence);
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
    s += formatString("SendState:%zu\n", _sentState.size());
    for(auto& st : _sentState)
    {
        s += formatString("  [%s] st:%u,rty:%u,%lu\n", st.first.toString().c_str(), st.second.state, st.second.retry, st.second.sentTime);
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
    for(auto& it : _peerInfo) { it.second = {}; }
}

void Transceiver::clear(const MACAddress& addr)
{
    _peerInfo.erase(addr);
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

// Build variables based on the current peer list
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
            if(!_peerInfo.count(addr))  { _peerInfo[addr] = {}; }
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
}

// WARN:Locked in this
bool Transceiver::post_ack(const uint8_t* peer_addr)
{
    LIB_LOGD("A:[%s]", MACAddress(peer_addr).toString().c_str());
    uint8_t buf[sizeof(TransceiverHeader)];
    make_data(buf, RUDP::Flag::ACK, peer_addr);
    return Communicator::instance().post(peer_addr, buf, sizeof(buf));
}

// WARN:Locked in this
uint64_t Transceiver::make_data(uint8_t* obuf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    // Header
    TransceiverHeader* th = (TransceiverHeader*)obuf;
    th->tid = _tid;
    th->size = sizeof(*th) + length;

    MACAddress addr(peer_addr);
    
    auto fvalue = to_underlying(flag);
    if(addr == BROADCAST) { LIB_LOGW("Broadcast is rudp prohibited"); fvalue = 0; }
    th->rudp.flag = fvalue;

    if(th->isACK())
    {
        lock_guard _(_sem);
        th->rudp.sequence = (++_sequence) & 0xFF;
        _peerInfo[addr].needReturn = false;
        _peerInfo[addr].sentSequence = _sequence;
        uint64_t ack = _peerInfo[addr].sequence;
        if(!peer_addr)
        {
            // Gets the minimum sequence of all peers exclude Null MAC
            auto it = std::min_element(_peerInfo.begin(), _peerInfo.end(),
                                       [](decltype(_peerInfo)::const_reference a, decltype(_peerInfo)::const_reference b)
                                       {
                                           if(!a.first) return false;
                                           if(!b.first) return true;
                                           return a.second.sequence < b.second.sequence;
                                       });
            ack = (it != _peerInfo.end()) ? it->second.sequence : 0;
        }
        _peerInfo[addr].sentAck = ack;
        th->rudp.ack = ack & 0xFF;
#if 0
        LIB_LOGD("  [%s] RUDP(0X%02x:S:%u:A:%u) | S:%llu A:%llu",
                 addr.toString().c_str(),
                 fvalue, th->rudp.sequence, th->rudp.ack,
                 _sequence, ack);
#endif
    }
    if(data && length) { std::memcpy(obuf + sizeof(*th), data, length); } // Append payload
    return _sequence;
}

void Transceiver::_update(const unsigned long ms, Communicator::state_map_t& sentState,  const Communicator::config_t& cfg)
{
    std::set<MACAddress> addrs;
    
    // Detect communication failure
    with_lock([this, &addrs, &ms, &sentState, &cfg]()
    {
        for(auto& pi : _peerInfo)
        {
            if(!pi.first) { continue; }
            //if(sentState[pi.first].retry > cfg.maxRetrans && pi.second.sentSequence > pi.second.ack)
            if(sentState[pi.first].retry > cfg.maxRetrans)
            {
                LIB_LOGD("No more retry");
                addrs.insert(pi.first);
            }
        }
    });
    for(auto& addr : addrs)
    {
        Communicator::instance().notify(Notify::ConnectionLost, &addr);
        //        post_rct(addr);
    }

    // Send ACK force
    addrs.clear();
    with_lock([this, &addrs, &ms, &sentState, &cfg]()
    {
        for(auto& pi : _peerInfo)
        {
            if(!pi.first || !esp_now_is_peer_exist(pi.first.data()) || !pi.second.needReturn) { continue; }
            // Send ACK if nothing is sent for a certain period of time after receiving ACK
            auto rtm = pi.second.recvTime;
            //            if(rtm && rtm > sentState[pi.first].sentTime && ms > rtm + cfg.cumulativeAckTimeout)
            if(rtm && ms > rtm + cfg.cumulativeAckTimeout)
            {
                LIB_LOGD(">> FA:T");
                addrs.insert(pi.first);
                continue;
            }
            // Post ACK if the number of unrespond ACKs exceeds a certain number
            //auto stm = pi.second.sentTime;
            //if(stm && rtm && pi.second.sequence > pi.second.sentAck + cfg.maxCumAck)
            if(pi.second.sequence > pi.second.sentAck + cfg.maxCumAck)
            {
                LIB_LOGD("%lu %llu %llu", rtm, pi.second.sequence, pi.second.sentAck);
                LIB_LOGD(">> FA:C");
                addrs.insert(pi.first);
            }
        }
    });
    for(auto& addr : addrs)
    {
        post_ack(addr.data());
    }
}

void Transceiver::on_receive(const MACAddress& addr, const TransceiverHeader* th)
{
    {
        lock_guard _(_sem);
        if(th->isACK())
        {
            auto& pi = _peerInfo[addr];

            //            LIB_LOGD("S:%llu:%u => %llu", _peerInfo[addr].sequence, th->rudp.sequence, restore_u64_later(_peerInfo[addr].sequence, th->rudp.sequence));
            //            LIB_LOGD("A:%llu:%u => %llu", _peerInfo[addr].ack, th->rudp.ack, restore_u64_later(_peerInfo[addr].ack, th->rudp.ack));
            auto seq = restore_u64_later(pi.sequence, th->rudp.sequence);
            auto ack = restore_u64_later(pi.ack, th->rudp.ack);
            pi.sequence = seq;
            pi.ack = ack;
            pi.recvTime = millis();
            LIB_LOGD("[RECV]:(%u): PL:%d PS:%llu PA:%llu", th->tid, th->hasPayload(), pi.sequence, pi.ack);
            pi.needReturn |= th->hasPayload();
        }
    }
    if(th->isRST()) { Communicator::instance().notify(Notify::Disconnect, &addr); }
}

void Transceiver::on_notify(const Notify notify, const void* arg)
{
    lock_guard _(_sem);
    auto paddr = (const MACAddress*)arg;
    switch(notify)
    {
    case Notify::Disconnect:
    case Notify::ConnectionLost:
        clear(*paddr);
        break;
    }
}

#if !defined(NDEBUG)
String Transceiver::debugInfo() const
{
    lock_guard _(_sem);
    String s;
    s = formatString("TID:%u SEQ:%llu\n", _tid, _sequence);
    for(auto& r : _peerInfo)
    {
        s += formatString("  [%s] %d:(%llu:%llu %lu)|(%llu:%llu)\n", r.first.toString().c_str(),
                          r.second.needReturn,
                          r.second.sequence, r.second.ack, r.second.recvTime,
                          r.second.sentSequence,r.second.sentAck);
    }
    s.trim();
    return s;
}
#endif
//
}}
