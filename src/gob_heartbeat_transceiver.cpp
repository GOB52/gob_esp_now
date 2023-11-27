/*!
  @file gob_heartbeat_transceiver.cpp
  @brief Transceiver for heartbeat (Alive monitoring)
*/
#include "gob_heartbeat_transceiver.hpp"
#include "internal/gob_esp_now_log.hpp"
#include <esp32-hal.h> // millis

namespace goblib { namespace esp_now {

void HeartbeatTransceiver::begin()
{
    if(_do) { return; }

    _do = true;

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

    this->post_heart_beat(millis());
}

void HeartbeatTransceiver::update(const unsigned long ms)
{
    if(!_do) { return; }
    
    // Transmission at regular intervals
    if(ms > _sent + _interval)
    {
        post_heart_beat(ms);
    }

    // Check ACK receive time
    if(!_sentBasis) { return; }

    with_lock([this, &ms]()
    {
        auto& acks = this->ack();
        //LIB_LOGV("acks:%zu", acks.size());
        for(auto& it : acks)
        {
            if(!(it.first) || !it.first.isUnicast() || !_recv.count(it.first)) { continue; }
            //LIB_LOGV("  %s", it.first.toString().c_str());
            if(it.second < sequence())
            {
                //LIB_LOGE("_recv:%lu", _recv[it.first]);
                if(ms > _recv[it.first] + _interval * 2)
                {
                    //LIB_LOGE("NOTIFY");
                    Communicator::instance().notify(Notify::ConnectionLost, &it.first);

                }
            }
        }
    });
}

void HeartbeatTransceiver::post_heart_beat(const unsigned long ms)
{
    if(Communicator::instance().numOfPeer() == 0) { return; }

    LIB_LOGD("[HBT:S] %lu", ms);

    uint8_t buf[sizeof(TransceiverHeader)];
    make_data(buf, RUDP::Flag::NUL, nullptr);
    Communicator::instance().post(nullptr, buf, sizeof(buf));

    _sent = ms;
    if(!_sentBasis) { _sentBasis = _sent; }
}

void HeartbeatTransceiver::onReceive(const MACAddress& addr, const TransceiverHeader* data)
{
    with_lock([this, &addr]()
    {
        auto ms = millis();
        LIB_LOGD("[HBT:R] %s %lu", addr.toString().c_str(), ms);
        this->_recv[addr] = ms;
    });
}

void HeartbeatTransceiver::onNotify(const Notify notify, const void* arg)
{
    if(notify == Notify::ConnectionLost)
    {
        auto addr = (MACAddress*)arg;
        LIB_LOGE("CONNECTION LOST: %s", addr->toString().c_str());
        Communicator::instance().unregisterPeer(*addr);
        _recv.erase(*addr);
    }
}
//
}}
