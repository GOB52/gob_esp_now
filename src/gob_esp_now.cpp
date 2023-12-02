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
    auto ch = (const CommunicatorHeader*)packet;
    String s;
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
    if(packet.empty()) { return String(); }
    assert(packet.size() >= sizeof(CommunicatorHeader) && "illgal size");
    return packet_to_str(packet.data());
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

template<typename Func> void for_each_packet(const std::vector<uint8_t>& packet, Func func)
{
    auto p = packet.data();
    auto ch = (const CommunicatorHeader*)p;
    if(packet.empty() || ch->count == 0) { return; }
    p += sizeof(Communicator);
    auto cnt = ch->count;
    while(cnt--)
    {
        auto th = (const TransceiverHeader*)p;
        func(th);
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

bool remove_unreliable(std::vector<uint8_t>& packet)
{
    auto p = packet.data();
    auto ch = (CommunicatorHeader*)p;
    if(packet.empty() || ch->count == 0) { return false; }
    
    auto cnt = ch->count;
    p += sizeof(CommunicatorHeader);

    //LIB_LOGD("CH0:%u:%u", ch->count, ch->size);
    while(cnt--)
    {
        auto th = (TransceiverHeader*)p;
        // Unreliable?
        if(th->rudp.flag == 0)
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
    }
}

#if 0
//update (eempry interval)
// 各ピアに ACK send

// post
[0][...]
Queue:
[0][...]

// post
[B][...]
Queue:
[0][...]
[B][...]

// post
[A][...]
Queue:
[0][...]
[A][...]
[B][...]

// update
Send
Queie:
[A][...]
[B][...]
Sent:
[0][..] // unrelizable 消去

// update
Send
Queie:
[B][...]
Sent:
[0][..] 
[A][.] // unrelizable 消去

//post 
[0][...]
Queie:
Queie:
[B][...]
Sent:
[0][..]
[B][...]

// update
Send
Queie:
[B][...]
Sent:
[0][.....]  // 同アドレスは足す
[A][.]

// update
Sentd:
[0]acked  // Sent から send ack ずみ削除して acked してないものがあれば送信
Queie:
[B][...]
Sent:

// update
Queie:
Sent:
[0][..] 
[A][.]
[B][.] // unrelizable 消去

// update
Queie:
Sent:
[0][.] 
[A][.]
[B][.]

#endif

void Communicator::update()
{
    if(!_began) { return; }

    auto ms = millis();

    lock_guard _(_sem);

    for(auto& t : _transceivers) { t->_update(ms); t->update(ms); } // update transceivers

    // Remove responded data from the sent queue
    for(auto& it : _sentQueue) { remove_acked(it.first, it.second); }
    
    if(!_canSend) { return; }

    // Send if exists queue (Order by MACAddresss ASC, Null MACAddress is top)
    bool sent{};
    for(auto& it : _queue)
    {
        if(!it.second.empty())
        {
            LIB_LOGD("---- Send");
            send_esp_now((bool)it.first ? it.first.data() : nullptr, it.second);
            _resendTimer = ms;
            sent = true;
            break;
        }
    }


    if(!sent)
    {
        // Resend data that has not been received and confirmed within a certain period of time
        bool resent{};
        for(auto& it : _sentQueue)
        {
            if(remove_acked(it.first, it.second) && !resent && _resendTimer && ms >= _resendTimer + _resendInterval)
            {
                LIB_LOGD("---- Rsend");
                _resendTimer = ms;
                resent = true;
                send_esp_now((bool)it.first ? it.first.data() : nullptr, it.second);
                break;
            }
        }
    }
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
    //ここまで溜まること自体が問題なので Disconnect でもいい気がする
    if(length + md.size() > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Overflow"); return false; }

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
    return send_esp_now(peer_addr, v);
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
        remove_unreliable(packet);
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
        remove_unreliable(packet);
        append_to_sent(peer_addr, packet);
        if(packet.data() != _sentQueue[addr].data()) { packet.clear(); }
        return false;
    }

    if(!_sentTime) { _sentTime = millis(); }
    _canSend = false;
    remove_unreliable(packet);
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
    auto p = packet.data();
    auto ch = (CommunicatorHeader*)p;
    if(packet.empty() || ch->count == 0) { return false; }
    assert(packet.size() >= sizeof(CommunicatorHeader) && "illgal size");

    //LIB_LOGD("packet:[%s]", packet_to_str(packet).c_str());
    
    p += sizeof(CommunicatorHeader);
    auto cnt = ch->count;

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
                    auto rseq = restore_u64_later(t->_peerSeq[addr], th->rudp.sequence);
                    //LIB_LOGD("th:%u rseq:%llu recvSeq:%llu", th->rudp.sequence, rseq, t->_peerSeq[addr]);
                    return rseq == (t->_peerSeq[addr] + 1) || t->_peerSeq[addr] == 0;
                    
                });
                if(b)
                {
                    t->on_receive(addr, th);
                    t->onReceive(addr, th);
                }
                else { LIB_LOGW("Old or future data has come.%u / %llu",th->rudp.sequence, t->_peerSeq[addr]); }
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
    for(auto& it : _peerSeq) { it.second = 0; }
    for(auto& it : _peerAck) { it.second = 0; }
}

void Transceiver::clear(const MACAddress& addr)
{
    _peerSeq.erase(addr);
    _peerAck.erase(addr);
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
            if(!_peerSeq.count(addr))  { _peerSeq[addr] = 0; }
            if(!_peerAck.count(addr))  { _peerAck[addr] = 0; }
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
}

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
    uint64_t seq{};
    // Header
    TransceiverHeader* th = (TransceiverHeader*)obuf;
    th->tid = _tid;
    th->size = sizeof(*th) + length;
    auto fvalue = to_underlying(flag);
    th->rudp.flag = fvalue;
    if(th->isACK())
    {
        lock_guard _(_sem);
        th->rudp.sequence = (++_sequence) & 0xFF;
        seq = _sequence;

        uint64_t ack = _peerSeq[MACAddress(peer_addr)];
        if(!peer_addr)
        {
            // Smallest of peers except NULL MACAddress
            auto it = std::min_element(_peerSeq.begin(), _peerSeq.end(),
                                       [](decltype(_peerSeq)::const_reference a, decltype(_peerSeq)::const_reference b)
                                       {
                                           if(!a.first) return false;
                                           if(!b.first) return true;
                                           return a.second < b.second;
                                       });
            ack = (it != _peerSeq.end()) ? it->second : 0x00;
        }
        th->rudp.ack = ack & 0xFF;
        
        LIB_LOGD("set:[%s] RUDP(%02x:%u:%u) | %llu/%llu",
                 MACAddress(peer_addr).toString().c_str(),
                 fvalue, th->rudp.sequence, th->rudp.ack,
                 _sequence, ack);

    }
    if(data && length) { std::memcpy(obuf + sizeof(*th), data, length); } // Append payload
    return seq;
}

void Transceiver::_update(const unsigned long ms)
{
    static unsigned long sack{ms};

    if(ms > sack + 500 && _tid == 56)
    {
        sack = ms;
        //        post_ack(nullptr);
    }
    // TODO:一定期間送信動作していなければ ACK を投げる? nullptr では ack が 0.......
    // post_ack();
#if 0
    受診してから一定期間送信していない場合
    if(needReturnAck) { post_ack(addr); }
#endif

}

void Transceiver::on_receive(const MACAddress& addr, const TransceiverHeader* data)
{
    {
        lock_guard _(_sem);
        if(data->isACK())
        {
            _peerSeq[addr] = restore_u64_later(_peerSeq[addr], data->rudp.sequence);
            _peerAck[addr] = restore_u64_later(_peerAck[addr], data->rudp.ack);
            LIB_LOGD("[RECV]:(%u):%llu/%llu", data->tid, _peerSeq[addr], _peerAck[addr]);
        }
    }
    if(data->isRST())
    {
        Communicator::instance().notify(Notify::Disconnect, &addr);
    }
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
    for(auto& it :  _peerSeq)
    {
        s += formatString("  Seq:[%s]:%llu\n", it.first.toString().c_str(), it.second);
    }
    for(auto& it :  _peerAck)
    {
        s += formatString("  ACK:[%s]:%llu\n", it.first.toString().c_str(), it.second);
    }
    s.trim();
    return s;
}
#endif
//
}}
