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
        // Unreliable, ACK only?
        if(th->rudp.flag == 0 || (th->rudp.flag == to_underlying(RUDP::Flag::ACK) && !th->hasPayload()))
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
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            MACAddress addr(info.peer_addr);
            auto& md = _queue[addr];
            md.reserve(ESP_NOW_MAX_DATA_LEN);
            md = _sentQueue[addr];
            md.reserve(ESP_NOW_MAX_DATA_LEN);
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
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
        _sentTime = 0;

        esp_now_unregister_send_cb();
        esp_now_unregister_recv_cb();
        _params = RUDP::SYN_Params{};
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

    for(auto& t : _transceivers) { t->_update(ms); t->update(ms); } // Update transceivers

    // Remove responded data from the sent queue
    for(auto& q : _sentQueue) { remove_acked(q.first, q.second); }
    
    if(!_canSend) { return; }

    // Send if exists queue (Order by MACAddresss ASC, Null MACAddress is top)
    bool sent{};
    for(auto& q : _queue)
    {
        if(q.second.empty()) { continue; }
        LIB_LOGD("---- Send:[%s]", q.first.toString().c_str());
        if(send_esp_now((bool)q.first ? q.first.data() : nullptr, q.second))
        {
            _resendTimer = ms + _params.retransmissionTimeout;
            sent = true;
            // Transmission is not possible before receiving the transmission callback, so it is terminated
            break;
        }
    }
    if(sent) { return; }

    // Resend RUDP data that has not been confirmed within a certain period of time
    for(auto& q : _sentQueue)
    {
        if(!remove_acked(q.first, q.second)) { continue; }
        if(_resendTimer && ms >= _resendTimer)
        {
            LIB_LOGD("---- Rsend:[%s]", q.first.toString().c_str());
            if(send_esp_now((bool)q.first ? q.first.data() : nullptr, q.second))
            {
                _resendTimer = ms + _params.retransmissionTimeout;
                sent = true;
                break;
            }
        }
    }
#if 0
    // Post a NUL if no transmission continues for a period of time.
    if(!sent && ms > _sentTimer + _params.nullSegmentTimeout)
    {
    }
#endif
}

// Post
bool Communicator::post(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }

    MACAddress addr = peer_addr ? MACAddress(peer_addr) : MACAddress();
    auto& md = _queue[addr]; // Create empty packet if not exiets.
    if(md.capacity() < ESP_NOW_MAX_DATA_LEN) { md.reserve(ESP_NOW_MAX_DATA_LEN); } // Expand memory

    // Modify header
    if(md.empty())
    {
        CommunicatorHeader ch{};
        ch.app_id = _app_id;
        md.resize(sizeof(CommunicatorHeader));
        std::memcpy(md.data(), &ch, sizeof(ch));
    }

    // Overflow
    // TODO:..... 蓄積と再ポストはどうする?
    if(length + md.size() > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Overflow"); return false; }

    // 
    auto t = transceiver( ((TransceiverHeader*)data)->tid );
    assert(t && "No transceiver");
    t->with_lock([&t,&addr]() { t->_peerRecv[addr].time = millis(); });
    
    // Append data
    auto p = md.data();
    auto ch = (CommunicatorHeader*)p;
    auto osz = md.size();
    md.resize(osz + length);
    assert(md.data() == p && "pointer changed");
    std::memcpy(p + osz, data, length);
    ++ch->count;
    ch->size = md.size();
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
    if(send_esp_now(peer_addr, v))
    {
        auto t = transceiver( ((TransceiverHeader*)data)->tid );
        assert(t && "No transceiver");
        t->with_lock([&t, &peer_addr]() { t->_peerRecv[MACAddress(peer_addr)].time = millis(); });
        return true;
    }
    return false;
}

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
        if(!_sentTime) { _sentTime = millis(); }
        remove_not_need_resend(packet);
        append_to_sent(peer_addr, packet);
        if(packet.data() != _sentQueue[addr].data()) { packet.clear(); }
        LIB_LOGD("[DF]:FailedS");
        return false;
    }
#endif
    auto ret = esp_now_send(peer_addr, packet.data(), packet.size());
    if(ret != ESP_OK)
    {
        LIB_LOGE("Failed to esp_now_send %s %d:%s", MACAddress(peer_addr).toString().c_str(), ret, err2cstr(ret));
        remove_not_need_resend(packet);
        append_to_sent(peer_addr, packet);
        if(packet.data() != _sentQueue[addr].data()) { packet.clear(); }
        return false;
    }

    if(!_sentTime) { _sentTime = millis(); }
    _canSend = false;
    remove_not_need_resend(packet);
    append_to_sent(peer_addr, packet);
    if(packet.data() != _sentQueue[addr].data()) { packet.clear(); }
    return true;
}

// Transmitted data append them to _sentQueue
void Communicator::append_to_sent(const uint8_t* peer_addr, std::vector<uint8_t>& packet)
{
    MACAddress addr(peer_addr);
    //LIB_LOGD("[%s]", addr.toString().c_str());
    //remove_acked(addr, _sentQueue[addr]);
    if(_sentQueue[addr].empty())
    {
        _sentQueue[addr] = std::move(packet);
        //LIB_LOGV("==> move %s", packet_to_str(_sentQueue[addr]).c_str());
        return;
    }
    if(!append_packet(_sentQueue[addr], packet)) { _sentQueue[addr].clear(); }
}

// Remove data that has already been received by the other device
// return true if data available
bool Communicator::remove_acked(const MACAddress& addr, std::vector<uint8_t>& packet)
{
    if(packet.empty()) { return false; }
    assert(packet.size() >= sizeof(CommunicatorHeader) && "illgal size");

    //LIB_LOGD("packet:[%s]", packet_to_str(packet).c_str());
    auto p = packet.data();
    auto ch = (CommunicatorHeader*)p;
    auto cnt = ch->count;
    p += sizeof(CommunicatorHeader);

    while(cnt--)
    {
        auto th = (TransceiverHeader*)p;
        auto t = transceiver(th->tid);
        assert(t && "No transceiver");
        if(th->isACK())
        {
#if 0
            // need t->with_lock
            LIB_LOGV("tid:%u [%s] restored:%u =>%llu ack:%llu",
                     t->_tid,
                     addr.toString().c_str(),
                     th->rudp.sequence,
                     restore_u64_earlier(t->_sequence, th->rudp.sequence),
                     t->_peerAck[addr]);
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
    if(ch->count == 0) { packet.clear(); }

    //LIB_LOGD("=> packet:[%s]", packet_to_str(packet).c_str());
    return !packet.empty();
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
        Communicator::instance().unregisterPeer(*paddr);
        _queue.erase(*paddr);
        _sentQueue.erase(*paddr);
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
        _sentTime = 0;
    }
    else
    {
        LIB_LOGE("ESP_NOW_SEND_FAIL %s", addr.toString().c_str());
    }
}

void Communicator::callback_onReceive(const uint8_t *mac_addr, const uint8_t* data, int length)
{
    MACAddress addr(mac_addr);
    LIB_LOGV("static receive from %s:%d", addr.toString().c_str(), length);
    LIB_LOGD("---- Recv");
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
                LIB_LOGD("(%u)[%s]", th->tid, addr.toString().c_str());
                // Old or future incoming data is not processed if RUDP ACK

                // トランシーバーの初めての受信
                // _peerSeq[addr] == 0 なら全て受けて入れる?

                auto b = !th->isACK() || t->with_lock([&t,&th,&addr]()
                {
                    auto rseq = restore_u64_later(t->_peerRecv[addr].sequence, th->rudp.sequence);
                    //LIB_LOGD("th:%u rseq:%llu recvSeq:%llu", th->rudp.sequence, rseq, t->_peerRecv[addr].sequence);
                    return rseq == (t->_peerRecv[addr].sequence + 1) || t->_peerRecv[addr].sequence == 0;
                    
                });
                if(b)
                {
                    t->on_receive(addr, th);
                    t->onReceive(addr, th);
                }
                else { LIB_LOGW("Old or future data has come.%u / %llu",th->rudp.sequence, t->_peerRecv[addr].sequence); }
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
    s += formatString("SYN<%u/%u/%u/%u %u:%u;%u:%u\n",
                      _params.retransmissionTimeout,
                      _params.cumulativeAckTimeout,
                      _params.nullSegmentTimeout,
                      _params.transferStateTimeout,
                      _params.maxRetrans,
                      _params.maxCumAck,
                      _params.maxOutOfSeq,
                      _params.maxAutoReset);
                      
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
            s += formatString("  [%s]\n",addr.toString().c_str());
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }

    s += formatString("queue:%zu\n", _queue.size());
    for(auto& it : _queue)
    {
        if(!it.second.empty())
        {
            s += formatString("  [%s] %s\n", it.first.toString().c_str(), packet_to_str(it.second).c_str());
        }
        else
        {
            s += formatString("  [%s]\n", it.first.toString().c_str());
        }
    }

    s += formatString("sentQueue:%zu\n", _sentQueue.size());
    for(auto& it : _sentQueue)
    {
        if(!it.second.empty())
        {
            s += formatString("  [%s] %s\n", it.first.toString().c_str(), packet_to_str(it.second).c_str());
        }
        else
        {
            s += formatString("  [%s]\n", it.first.toString().c_str());
        }
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
    for(auto& it : _peerRecv) { it.second = {}; }
}

void Transceiver::clear(const MACAddress& addr)
{
    _peerRecv.erase(addr);
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
            if(!_peerRecv.count(addr))  { _peerRecv[addr] = {}; }
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

    auto fvalue = to_underlying(flag);
    if(MACAddress(peer_addr) == BROADCAST) { LIB_LOGW("Broadcast is rudp prohibited"); fvalue = 0; }
    th->rudp.flag = fvalue;

    if(th->isACK())
    {
        lock_guard _(_sem);
        th->rudp.sequence = (++_sequence) & 0xFF;
        uint64_t ack = _peerRecv[MACAddress(peer_addr)].sequence;
        if(!peer_addr)
        {
            // Gets the minimum sequence of all peers exclude Null MAC
            auto it = std::min_element(_peerRecv.begin(), _peerRecv.end(),
                                       [](decltype(_peerRecv)::const_reference a, decltype(_peerRecv)::const_reference b)
                                       {
                                           if(!a.first) return false;
                                           if(!b.first) return true;
                                           return a.second.sequence < b.second.sequence;
                                       });
            ack = (it != _peerRecv.end()) ? it->second.sequence : 0;
        }
        th->rudp.ack = ack & 0xFF;
        
        LIB_LOGD("set:[%s] RUDP(%02x:%u:%u) | %llu/%llu",
                 MACAddress(peer_addr).toString().c_str(),
                 fvalue, th->rudp.sequence, th->rudp.ack,
                 _sequence, ack);

    }
    if(data && length) { std::memcpy(obuf + sizeof(*th), data, length); } // Append payload
    return _sequence;
}

void Transceiver::_update(const unsigned long ms)
{
    return;

    std::vector<MACAddress> post;

    // Send ACK if nothing is sent for a certain period of time after receiving ACK
    // ACK受信後、一定期間何も送信していない場合は ACK 投げる
    // Post ACK if the number of unrespond ACKs exceeds a certain number
    // 未応答の ACK が一定数超えたら ACK なげる
    with_lock([this, &post, &ms]()
    {
        for(auto& r : _peerRecv)
        {
            if(!r.first) { continue; } // skip Null MAC
            if(r.second.time && ms > r.second.time + 750)
            {
                post.push_back(r.first);
            }
#if 0
            // _sequence が同期されている前提...
            if(this->_sequence > r.second.sequence + NNN)
            {

            }
#endif
        }
    });

    for(auto& addr : post)
    {
        LIB_LOGD("EmptyA:[%s]", addr.toString().c_str());
        post_ack(addr.data());
    }
}

void Transceiver::on_receive(const MACAddress& addr, const TransceiverHeader* th)
{
    {
        lock_guard _(_sem);
        if(th->isACK())
        {
            auto seq = restore_u64_later(_peerRecv[addr].sequence, th->rudp.sequence);
            auto ack = restore_u64_later(_peerRecv[addr].ack, th->rudp.ack);
            _peerRecv[addr] = Recv{ seq, ack, millis() };
            LIB_LOGD("[RECV]:(%u):%llu/%llu", th->tid, _peerRecv[addr].sequence, _peerRecv[addr].ack);
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
    for(auto& r : _peerRecv)
    {
        s += formatString("  [%s] %llu/%llu %lu\n", r.first.toString().c_str(),
                          r.second.sequence, r.second.ack, r.second.time);
    }
    s.trim();
    return s;
}
#endif
//
}}
