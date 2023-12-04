/*!
  @file gob_heartbeat_transceiver.cpp
  @brief Transceiver for heartbeat (Alive monitoring)
*/
#include "gob_heartbeat_transceiver.hpp"
#include "internal/gob_esp_now_log.hpp"
#include <esp32-hal.h> // millis

namespace goblib { namespace esp_now {

bool HeartbeatTransceiver::begin(const bool sender)
{
    if(_began) { return true; }

    _sender = sender;
    _began = true;
    return true;
}

void HeartbeatTransceiver::end()
{
    with_lock([this]()
    {
        this->_began = false;
        this->_sent.clear();
        this->reset();
    });
}

void HeartbeatTransceiver::update(const unsigned long ms)
{
    if(!_began) { return; }

    // Have all ACK of the specified sequence number been received?
    with_lock([this]()
    {
        uint64_t acked{};
        for(auto it = _sent.crbegin(); it != _sent.crend(); ++it)
        {
            if(this->peerReceived(it->sequence)) { acked = it->sequence; break; }
        }
        if(!acked) return;

        auto it = std::remove_if(_sent.begin(), _sent.end(), [&acked](const Sent& s) { return s.sequence <= acked; });
        _sent.erase(it, _sent.end());
        this->_acked = acked;
        LIB_LOGV("[HBT]:acked %llu", acked);
    });
    
    if(_sender) { update_sender  (ms); }
    else        { update_receiver(ms); }
}

void HeartbeatTransceiver::update_sender(const unsigned long ms)
{
    if(Communicator::instance().numOfPeer() > 0 && ms > _lastSent + _interval)
    {
        //        if(_peerRev[



        post_heart_beat(ms);
    }
    // If no response is received _ccl times, the connection is considered lost.
    std::vector<MACAddress> lost;
    if(_sent.empty()) { return; }

    with_lock([this, &lost]()
    {
        auto& pr = peerInfo();
        for(auto& r : pr)
        {
            if(!(r.first)
               || !r.first.isUnicast()
               || ((bool)_addr && _addr != r.first)) { continue; }
            //LIB_LOGV("  %s", r.first.toString().c_str());
            if(_sent.back().sequence - r.second.ack  > this->_ccl)
            {
                LIB_LOGD("last seq:%llu acked:%llu", _sent.back().sequence, r.second.ack);
                lost.push_back(r.first);
            }
        }
    });
    if(!lost.empty()) { LIB_LOGI("Detect lost"); }
    for(auto& a : lost) { Communicator::instance().notify(Notify::ConnectionLost, &a); }
}

void HeartbeatTransceiver::update_receiver(const unsigned long ms)
{
    // Receiver considers connection lost if NUL is not received for more than twice the interval
    std::vector<MACAddress> lost;
    with_lock([this, &ms, &lost]()
    {
        for(auto& addr : _senderAddress)
        {
            if(ms > this->ackTime(addr) + this->_interval * 2)
            {
                lost.push_back(addr);
            }
        }
    });
    if(!lost.empty()) { LIB_LOGI("Detect lost"); }
    for(auto& a : lost) { Communicator::instance().notify(Notify::ConnectionLost, &a); }
}

void HeartbeatTransceiver::post_heart_beat(const unsigned long ms)
{
    if(Communicator::instance().numOfPeer() == 0) { return; }

    uint8_t buf[sizeof(TransceiverHeader)];
    auto paddr = (bool)_addr ? _addr.data() : nullptr; // specific target or all peer.
    auto seq = make_data(buf, RUDP::Flag::NUL, paddr);
    Communicator::instance().post((bool)_addr ? _addr.data() : nullptr, buf, sizeof(buf));

    LIB_LOGD("[HBT]:S %u %lu", ((TransceiverHeader*)buf)->rudp.sequence, ms);
    _sent.emplace_back(Sent{ms, seq});
    _lastSent = ms;
}

void HeartbeatTransceiver::onReceive(const MACAddress& addr, const TransceiverHeader* data)
{
    with_lock([this, &addr, &data]()
    {
        if(!_sender) { _senderAddress.insert(addr); }
        LIB_LOGD("[HBT]:R [%s](%u:%u) ", addr.toString().c_str(), data->rudp.sequence, data->rudp.ack);
    });

    // Return ACK to addr if received NUL
    if(data->rudp.flag == to_underlying(RUDP::Flag::NUL))
    {
        LIB_LOGD("[HBT]:A [%s]", addr.toString().c_str());
        post_ack(addr.data());
    }
}

void HeartbeatTransceiver::onNotify(const Notify notify, const void* arg)
{
    auto paddr = (const MACAddress*)arg;
    switch(notify)
    {
    case Notify::Disconnect:
    case Notify::ConnectionLost:
        with_lock([this, &paddr]() { this->_senderAddress.erase(*paddr); });
        break;
    }
}
//
}}
