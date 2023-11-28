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
    
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            MACAddress addr(info.peer_addr);
            _recv[addr] = 0;
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
    else { return false; }

    _sender = sender;
    _began = true;
    return true;
}

void HeartbeatTransceiver::end()
{
    with_lock([this]()
    {
        this->_began = false;
        for(auto& it : this->_recv) { it.second = 0; }
        this->_sent = this->_sentBasis = 0;
        this->reset();
    });
}

void HeartbeatTransceiver::update(const unsigned long ms)
{
    if(!_began) { return; }

    if(_sender) { update_sender(ms);   }
    else        { update_receiver(ms); }

    // Check ACK receive time
    if(!_sentBasis) { return; }

    MACAddress addr = with_lock([this, &ms]()
    {
        auto& acks = this->acks();
        //LIB_LOGV("acks:%zu", acks.size());
        for(auto& it : acks)
        {
            if(!(it.first)
               || !it.first.isUnicast()
               || !_recv.count(it.first) 
               || ((bool)_addr && _addr != it.first)) { continue; }
            //LIB_LOGV("  %s", it.first.toString().c_str());
            if(it.second < sequence())
            {
                //LIB_LOGE("_recv:%lu", _recv[it.first]);
                if(ms > _recv[it.first] + _interval * 2)
                {
                    return it.first;
                }
            }
        }
        return MACAddress();
    });
    if((bool)addr) { Communicator::instance().notify(Notify::ConnectionLost, &addr); }
}

void HeartbeatTransceiver::update_sender(const unsigned long ms)
{
    // Transmission at regular intervals
    if(ms > _sent + _interval)
    {
        // TODO:返答がない場合
        post_heart_beat(ms);
    }
}

void HeartbeatTransceiver::update_receiver(const unsigned long ms)
{
}

void HeartbeatTransceiver::post_heart_beat(const unsigned long ms)
{
    if(Communicator::instance().numOfPeer() == 0) { return; }

    uint8_t buf[sizeof(TransceiverHeader)];
    auto pa = (bool)_addr ? _addr.data() : nullptr; // specific target or all peer.
    make_data(buf, RUDP::Flag::NUL, pa);
    Communicator::instance().post((bool)_addr ? _addr.data() : nullptr, buf, sizeof(buf));

    LIB_LOGD("[HBT]:S %u %lu", ((TransceiverHeader*)buf)->rudp.sequence, ms);
    _sent = ms;
    if(!_sentBasis) { _sentBasis = _sent; }
}

void HeartbeatTransceiver::post_ack(const MACAddress& addr)
{
    LIB_LOGD("[HBT]:A %s", addr.toString().c_str());
    uint8_t buf[sizeof(TransceiverHeader)];
    make_data(buf, RUDP::Flag::ACK, addr.data());
    Communicator::instance().post(addr.data(), buf, sizeof(buf));
}

void HeartbeatTransceiver::onReceive(const MACAddress& addr, const TransceiverHeader* data)
{
    with_lock([this, &addr, &data]()
    {
        auto ms = millis();
        LIB_LOGD("[HBT]:R [%s] S:%U A:%u %lu", addr.toString().c_str(), data->rudp.sequence, data->rudp.ack, ms);
        this->_recv[addr] = ms;
    });

    // Return ACK to addr if received NUL
    if(data->rudp.flag == (RUDP::flag_t)RUDP::Flag::NUL)
    {
        post_ack(addr);
    }
}

void HeartbeatTransceiver::onNotify(const Notify notify, const void* arg)
{
    if(notify != Notify::ConnectionLost) { return; }

    auto paddr = (MACAddress*)arg;
    LIB_LOGI("CONNECTION LOST: %s", paddr->toString().c_str());
    Communicator::instance().unregisterPeer(*paddr);
    with_lock([this,&paddr]()
    {
        this->_recv.erase(*paddr);
    });
}

//
}}
