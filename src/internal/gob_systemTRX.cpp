/*!
  @file gob_systemTRX.cpp
  @brief System transceiver for communicator

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include "gob_systemTRX.hpp"
#include "gob_esp_now.hpp"
#include "internal/gob_esp_now_log.hpp"

namespace goblib { namespace esp_now {

SystemTRX::SystemTRX() : Transceiver(52/*dummy value*/)
{
}

SystemTRX:: ~SystemTRX()
{
}

bool SystemTRX::postSYN(const MACAddress& addr, RUDP::config_t& cfg)
{
    return post_rudp(addr.data(), RUDP::Flag::SYN, &cfg, sizeof(cfg));
}

void SystemTRX::update(const unsigned long ms)
{
    auto& comm = Communicator::instance();
    auto cfg = comm.config();

    // Return my config
    for(auto& addr : _postSynAddr)
    {
        bool b = post_rudp(addr.data(), RUDP::Flag::SYN_ACK, &cfg, sizeof(cfg));
        if(!b) { LIB_LOGE("Failed to post %s", addr.toString().c_str()); }
    }

    // Post ACk that has been received the primary config
    for(auto& addr : _recvSynAddr)
    {
        bool b = post_ack(addr);
        if(!b) { LIB_LOGE("Failed to post %s", addr.toString().c_str()); }
    }

    _postSynAddr.clear();
    _recvSynAddr.clear();
}

void SystemTRX::on_receive(const MACAddress& addr, const TransceiverHeader* th)
{
    Transceiver::on_receive(addr, th);

    with_lock([this](const MACAddress& addr, const TransceiverHeader* th)
    {
        auto& comm = Communicator::instance();
        if(th->isSYN())
        {
            // Receive SYN request
            if(!th->isACK())
            {
                LIB_LOGD("SYN req");
                if(comm.isSecondary())
                {
                    LIB_LOGE("Reject SYN request");
                    return;
                }
                comm.setRole(Role::Primary);
                _postSynAddr.push_back(addr);
                return;
            }
            // Receive SYN+ACK and apply config
            else
            {
                LIB_LOGD("SYN+ACK");
                // TODO sequence.ack,ackseq....
                comm.setRole(Role::Secondary);
                const RUDP::config_t* cfg = (const RUDP::config_t*)th->payload();
                //comm.applyConfig(cfg);
                _recvSynAddr.push_back(addr);
            }
        }
    }, addr, th);
}
//
}}
