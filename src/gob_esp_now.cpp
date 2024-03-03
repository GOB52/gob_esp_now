/*!
  @file gob_esp_now.cpp
  @brief ESP-NOW wrapper, helper and utilities.
*/
#include "gob_esp_now.hpp"
#include "internal/gob_esp_now_log.hpp"
#include "internal/gob_systemTRX.hpp"
#include "internal/gob_esp_now_utility.hpp"
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
PROGMEM const char notifyDisconnect[] = "DISCONNECT";
PROGMEM const char notifyConnectionLost[] = "CONNECTION_LOST";
PROGMEM const char notifyShookhand[] = "SHOOKHAND";
PROGMEM const char* notifyStringTable[] =
{
    notifyDisconnect,
    notifyConnectionLost,
    notifyShookhand,
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

    // TODO: if overflow....
    a.resize(osz + ssz);
    std::memcpy(a.data() + osz, s, ssz);
    ch->count += b_ch->count;
    ch->size = a.size();

    if(a.size() > ESP_NOW_MAX_DATA_LEN) { LIB_LOGE("Packet overflow"); }
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
            LIB_LOGE("Failed to init:0x%x [%s]", r, err2cstr(r));
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
        // unreliable or only ACK with no payload
        //        if(!th->isRUDP() || (th->onlyACK() && !th->hasPayload())
        if(!th->isACK() || (th->onlyACK() && !th->hasPayload()) )
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
    if(ch->count == 0) { ch->size = 0; packet.clear(); }
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

    _permitToSend = xQueueCreate(1, 0U);
    assert(_permitToSend);
    _postedAny = xQueueCreate(1, 0U);
    assert(_postedAny);
    
    _addr.get(ESP_MAC_WIFI_STA);
    _sysTRX = new SystemTRX();
    assert(_sysTRX);
    _sysTRX->_tid = 0; // Overwrite ID (0 means systemTRX) 0 cannot be specified at the application layer.
    _transceivers.push_back(_sysTRX);
}

bool Communicator::begin(const uint8_t app_id, const config_t& cfg)
{
    if(_began) { LIB_LOGW("Already begun"); return true; }

    lock_guard _(_sem);

    _config = cfg;
    
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

    _receive_queue = xQueueCreate(_config.receive_queue_size, sizeof(RecvQueueData));
    xTaskCreateUniversal(receive_task, "gen_receive",
                         _config.task_stack_size, this, _config.receive_priority, &_receive_task, _config.receive_core);

    // If update_priority is zero, the user explicitly calls it.
    if(_config.update_priority)
    {
        xTaskCreateUniversal(update_task, "gen_update",
                             _config.task_stack_size, this, _config.update_priority, &_update_task, _config.update_core);
    }
    
    _began = (_config.update_priority ? _update_task != nullptr : true) && _receive_task && _receive_queue;
    //_began = (_config.update_priority ? _update_task != nullptr : true);
    if(!_began) { end(); }
    return _began;
}

void Communicator::end()
{
    lock_guard _(_sem);

    _began = false;
    _transceivers.clear();
    _transceivers.push_back(_sysTRX);
    _lastSentTime = 0;
    clearPeer();
    esp_now_unregister_send_cb();
    esp_now_unregister_recv_cb();

    if(_update_task)  { vTaskDelete(_update_task); _update_task = nullptr; }
    if(_receive_task) {vTaskDelete(_receive_task);  _receive_task = nullptr; }
    vQueueDelete(_receive_queue);
    _receive_queue = nullptr;
}

void Communicator::update()
{
    lock_guard _(_sem);
    if(!_began) { return; }

    ++_update_count;
    
    auto ms = millis();
    unsigned long startTime{};
    bool sent{};

    // Remove acked data
    // If we don't do it here, the TRX update will make it easier to overflow the posted data.
    for(auto& q : _queue) { remove_acked(q.first, q.second); }

    // Update transceivers
    for(auto& t : _transceivers)
    {
        t->with_lock([this, &t, &ms]
        {
            t->_update(ms, this->_config.cumulativeAckTimeout, this->_config.maxCumAck);
            t->update(ms);
        });
    }

    // Detect communication failure
    for(auto& ss : _state)
    {
        if(_config.maxRetrans && ss.second.retry > _config.maxRetrans)
        {
            LIB_LOGE("<Exceeded retry count>");
            MACAddress copy(ss.first);
            notify(Notify::ConnectionLost, &copy); // Passing ss.first is disabled inside notify, so copy
        }
    }
    
    // Send if exists queue (Order by MACAddresss ASC)
    // first : MACAddress
    // second : packet
    for(auto& q : _queue)
    {
        if(q.second.empty() || !_state.count(q.first)) { continue; }
        // Send / resend
        if(_state[q.first].state == State::None ||
           (/*_state[q.first].state != State::None &&*/
               ms > _state[q.first].sentTime + _config.retransmissionTimeout))
        {
            // TODO : realy need it?
            if(!remove_acked(q.first, q.second)) { continue; }

            LIB_LOGD("-- %s:%u:[%s]",
                     _state[q.first].state == State::None ? "Send" : "Resend",
                     _state[q.first].retry,q.first.toString().c_str());

            startTime = micros();
            sent = send_esp_now((bool)q.first ? q.first.data() : nullptr, q.second);
            if(!sent) { ++_state[q.first].retry; continue; }

            if(_state[q.first].state == State::None) { _state[q.first].retry = 0; }
            else { ++_state[q.first].retry; }
#if 1
            // Wait called callback_onSent
            xSemaphoreGiveRecursive(_sem);
            while(xQueueReceive(_permitToSend, nullptr, portMAX_DELAY) != pdTRUE);
            xSemaphoreTakeRecursive(_sem, portMAX_DELAY);
            auto oneTime = micros() - startTime;
            ++_sentCount;
            _time += oneTime;
            _minTime = std::min(oneTime, _minTime);
            _maxTime = std::max(oneTime, _maxTime);
#else
            break;
#endif
        }
    }
#if 0
    // Wait called callback_onSent
    if(sent)
    {
        while(xQueueReceive(_permitToSend, nullptr, portMAX_DELAY) != pdTRUE);
        auto oneTime = micros() - startTime;
        ++_sentCount;
        _time += oneTime;
        _minTime = std::min(oneTime, _minTime);
        _maxTime = std::max(oneTime, _maxTime);
    }
#endif

    // Detect communication failures and heartbeat
    if(!_config.nullSegmentTimeout) { return; }
    for(auto& ss : _state)
    {
        if(!ss.first || ss.first.isMulticast()) { continue; }

        // Send a NUL if I am the client when a null timeout occurs
        if(isSecondary()
           && (_queue.count(ss.first) && _queue.at(ss.first).empty())
           && ms > ss.second.sentTime + _config.nullSegmentTimeout)
        {
            LIB_LOGD("[NUL]");
            _sysTRX->post_nul(ss.first);
        }
        // If I have not received for double the time of the NUL timeout
        if(isPrimary()
           && ss.second.sentTime < ss.second.recvTime
           && ms > ss.second.recvTime + _config.nullSegmentTimeout * 2)
        {
            LIB_LOGE("<NUL never come>");
            MACAddress copy(ss.first);
            notify(Notify::ConnectionLost, &copy); // Passing ss.first is disabled inside notify, so copy
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

    // Make conmmunicator header?
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
    //LIB_LOGD("CH:%u:%u", ch->count, ch->size);
    return true;
}

#if 0
// Send directly
bool Communicator::send(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    lock_guard _(_sem);

    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { LIB_LOGE("peer_addr not exists"); return false; }

    if(!_began) { LIB_LOGE("Not ready to send"); return false; }
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
        LIB_LOGW("No peer or empty packet %s", addr.toString().c_str());
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
        LIB_LOGE("Failed to esp_now_send %s 0x%x:%s", MACAddress(peer_addr).toString().c_str(), ret, err2cstr(ret));
        return false;
    }
    _lastSentTime = millis();
    _lastSentAddr = addr;
    _lastSentQueue = packet;
    packet.clear();

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
                return th->needReturnACK() && t->delivered(th->rudp.sequence, addr);
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
    LIB_LOGW("%s", debugInfo().c_str());

    auto paddr = (const MACAddress*)arg;
    switch(n)
    {
    case Notify::Disconnect: // [[fallthrough]]
    case Notify::ConnectionLost:
        LIB_LOGI("%s[%s]", notify_to_cstr(n), paddr->toString().c_str());
        unregisterPeer(*paddr);
        break;
    case Notify::Shookhands: break;
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

    // Add and sort by id ASC
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
    if(t == _sysTRX) { LIB_LOGE("System TRX cannot be unregistered."); return false; }

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
        LIB_LOGD("register [%s] Multi:%d Univ:%d", addr.toString().c_str(), addr.isMulticast(), addr.isUniversal());

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
    if(ret != ESP_OK) { LIB_LOGE("Failed to add:0x%x [%s]", ret, err2cstr(ret)); }
    return ret == ESP_OK;;
}

void Communicator::unregisterPeer(const MACAddress& addr)
{
    if(!addr) { LIB_LOGE("Null address"); return; }

    LIB_LOGD("unregister %s", addr.toString().c_str());
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
    if(ret != ESP_OK) { LIB_LOGE("Failed to del:0x%x [%s]", ret, err2cstr(ret)); }
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

const std::vector<MACAddress> Communicator::getPeerAddresses() const
{
    std::vector<MACAddress> v;
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do { v.emplace_back(info.peer_addr); }while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
    return v;
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

    //LIB_LOGE("<%s>", status ? "FAILED" : "SUCC");
    if(!succeed) { LIB_LOGW("FAILED:%s", addr.toString().c_str()); }

    {
        lock_guard _(_sem);
        // Composite the remaining elements and those added to the queue between esp_send and the callback
        //if(succeed)
        {
            remove_not_need_resend(_lastSentQueue);
        }
        auto sz = _lastSentQueue.size();
        auto b = !_queue[addr].empty();
        _queue[addr] = append_queue(_lastSentQueue, _queue[addr]);
        if(b) { LIB_LOGD("%d append %zu => %zu", succeed, sz, _queue[addr].size()); }
        
        if(_state.count(addr))
        {
            _state[addr].state = (State::Status)((uint8_t)State::Status::Succeed + (succeed ? 0 : 1));
            _state[addr].sentTime = millis();
        }
    }
    xQueueSend(_permitToSend, nullptr, 0);
}

void Communicator::callback_onReceive(const uint8_t* peer_addr, const uint8_t* data, int length)
{
    auto& comm = instance();
    auto ch = (const CommunicatorHeader*)data;
    if(ch->signeture != CommunicatorHeader::SIGNETURE
       || ch->app_id != comm._app_id) { LIB_LOGE("Illegal data"); return; }


#if 1
    RecvQueueData rqd = { MACAddress(peer_addr), {}, (uint8_t)length };
    memcpy(rqd.buf, data, length);
    if(xQueueSend(comm._receive_queue, &rqd, 0) != pdPASS)
    {
        LIB_LOGE(">>> CAUTION << "
                 "Queues are overflowing because there are not enough."
                 "Increase the number of queues or adjust the communication frequency.");
    }
#else
    MACAddress addr(peer_addr);
    if(esp_now_is_peer_exist(peer_addr)) { comm.onReceive(addr, data, length); }
    // Data probably from broadcast communication (no peers registered yet)
    else                                 { comm.onReceiveNotRegistered(addr, data, length); }
#endif
}

void Communicator::onReceiveNotRegistered(const MACAddress& addr, const uint8_t* data, const uint8_t length)
{
    // Data probably from broadcast communication (no peers registered yet)
    // Pass to system transceiver
    auto ch = (const CommunicatorHeader*)data;
    auto cnt = ch->count;
    data += sizeof(CommunicatorHeader);
    while(cnt--)
    {
        auto th = (const TransceiverHeader*)data;
        if(th->tid == 0) { _sysTRX->on_receive(addr, th); }
        else { LIB_LOGW("Reject data from unregistered peers %s:%u", addr.toString().c_str(), th->tid); }
        data += th->size;
    }
}

void Communicator::onReceive(const MACAddress& addr, const uint8_t* data, const uint8_t length)
{
    auto ch = (const CommunicatorHeader*)data;

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

    ++_receive_count;
    
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
                            auto rseq = th->needReturnACK() ?
                                    t->_peerInfo[addr].recvSeq : t->_peerInfo[addr].recvAckSeq;

                            auto seq = restore_u64_later(rseq, th->rudp.sequence);
                            // Correct order?
                            return seq == rseq + 1;
                        });
                
                if(correct)
                {
                    t->on_receive(addr, th);
                }
                else
                {
                    auto rs = th->needReturnACK() ?
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


bool Communicator::isHandshakeAllowed() const
{
    return _sysTRX->isHandshakeAllowed();
}

bool Communicator::isHandshakeDenied() const
{
    return _sysTRX->isHandshakeDenied();
}

void Communicator::enableHandshake(const bool enable)
{
    _sysTRX->enableHandshake(enable);
}

uint8_t Communicator::getMaxHandshakePeer() const
{
    return _sysTRX->getMaxHandshakePeer();
}

void Communicator::setMaxHandshakePeer(const uint8_t num)
{
    _sysTRX->setMaxHandshakePeer(num);
}

bool Communicator::broadcastHandshake()
{
    if(isHandshakeDenied()) { LIB_LOGE("Handshake denied"); return false; }
    if(!existsPeer(BROADCAST) &&!registerPeer(BROADCAST)) { return false; }
    return _sysTRX->broadcastHandshake();
}

bool Communicator::postSYN(const MACAddress& addr)
{
    return _sysTRX->postSYN(addr, _config);
}

void Communicator::update_task(void* arg)
{
    Communicator* comm = (Communicator*)arg;
    config_t cfg = comm->config();

    for(;;)
    {
        comm->update();
        // Do not get stuck in WDT and ensure that processing is passed on to lower priority tasks.
        if(cfg.update_core == 0 || cfg.update_priority > 1) { delay(1); }
        else {  taskYIELD(); } // Provide execution opportunities for other tasks of the same priority
    }
}

void Communicator::receive_task(void* arg)
{
    Communicator* comm = (Communicator*)arg;
    //config_t cfg = comm->config();
    RecvQueueData data;
    // If there is nothing in the Queue, the process moves to another task (include lower priority),
    // so unlike update_task, no explicit delay is called.
    while(xQueueReceive(comm->_receive_queue, &data, portMAX_DELAY) == pdTRUE)
    {
        comm->onReceive(data.addr, data.buf, data.size);
        taskYIELD(); // Provide execution opportunities for other tasks of the same priority
    }
}


#if !defined(NDEBUG)
String Communicator::debugInfo() const
{
    lock_guard _(_sem);

    String s;
    s += formatString("Communicator app_id:%u, %zu transceivers\n", _app_id, _transceivers.size());
    s += formatString("config:<RT:%u/CAT:%u/NST:%u/TST:%u mRt:%u mCa::%u mOs:%u mAr:%u>\n",
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

    s += formatString("aveTime:%llu minTime:%lu maxTime:%lu\n",
                      _sentCount ? _time / _sentCount : 0, _minTime, _maxTime);
    s.trim();
    return s;
}
#endif

//
}}

