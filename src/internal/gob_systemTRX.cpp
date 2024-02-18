/*!
  @file gob_systemTRX.cpp
  @brief System transceiver for communicator

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include "gob_systemTRX.hpp"
#include "gob_esp_now.hpp"
#include "internal/gob_esp_now_log.hpp"
#include <esp32-hal.h> // millis

namespace
{
// 
struct Payload
{
    static constexpr uint32_t MAGIC_NO = 0x5F25AD0C;

    uint32_t  magicNo{ MAGIC_NO };
    enum Function : uint8_t { AllowConnection = 1, };
    uint8_t function{};
#if 0
    // Extra data
    union
    {
    };
#endif
} __attribute__((__packed__));
//
}

namespace goblib { namespace esp_now {

SystemTRX::SystemTRX() : Transceiver(52/*dummy value*/)
{
}

SystemTRX:: ~SystemTRX()
{
}

bool SystemTRX::broadcastAllowConnection()
{
    Payload pl;
    pl.function = Payload::Function::AllowConnection;
    return postUnreliable(goblib::esp_now::BROADCAST, pl);
}

bool SystemTRX::postSYN(const MACAddress& addr, RUDP::config_t& cfg)
{
    uint64_t seq{};
    return post_rudp(seq, addr.data(), RUDP::Flag::SYN, &cfg, sizeof(cfg));
}

// Locked
void SystemTRX::update(const unsigned long ms)
{
    auto& comm = Communicator::instance();
    auto cfg = comm.config();
    uint64_t seq{};

    // first : MACAddress
    // second: SynInfo
    auto it = _synInfo.begin();
    while(it != _synInfo.end())
    {
        switch(it->second.status)
        {
        case SynInfo::State::None: break;
            // P
        case SynInfo::State::PostSYNACK:
            LIB_LOGD("Post SYNACK");
            if(post_rudp(seq, it->first.data(), RUDP::Flag::SYN_ACK, &cfg, sizeof(cfg)))
            {
                it->second.tm = ms;
                it->second.status = SynInfo::State::WaitACK;
                it->second.sequence = seq;
            }
            else
            {
                LIB_LOGE("Failed to postSYNACK");
                it->second.status = SynInfo::State::None;
            }
            break;
        case SynInfo::State::WaitACK:
            if(delivered(it->second.sequence, it->first))
            {
                LIB_LOGD("Recv ACK");
                it->second.status = SynInfo::State::Shookhand;
            }
            else if(ms - it->second.tm > SynInfo::TIMEOUT)
            {
                LIB_LOGW("Wait ACK timeout");
                comm.unregisterPeer(it->first);
                it->second.status = SynInfo::State::None;
            }
            break;
            // S
        case SynInfo::State::PostSYN:
            LIB_LOGD("Post SYN");
            if(post_rudp(seq, it->first.data(), RUDP::Flag::SYN, &cfg, sizeof(cfg)))
            {
                it->second.tm = ms;
                it->second.status = SynInfo::State::WaitSYNACK;
            }
            else
            {
                LIB_LOGE("Failed to postSYN");
                it->second.status = SynInfo::State::None;
            }
            break;
        case SynInfo::State::WaitSYNACK:
            if(ms - it->second.tm > SynInfo::TIMEOUT)
            {
                LIB_LOGW("Wait SYNACK timeout");
                comm.unregisterPeer(it->first);
                it->second.status = SynInfo::State::None;
            }
            break;
        case SynInfo::State::PostACK:
            LIB_LOGD("Post ACK");
            if(post_ack(it->first)) { it->second.status = SynInfo::State::Shookhand; }
            else { LIB_LOGE("Failed to postACK"); }
            break;

        case SynInfo::State::Shookhand:
            LIB_LOGE("Shookhand %s", it->first.toString().c_str());
            comm.notify(Notify::Shookhands, &it->first);
            it->second.status = SynInfo::State::None;
            break;
        }
        it = (it->second.status == SynInfo::State::None) ? _synInfo.erase(it) : std::next(it);
    }
}

// No locked
void SystemTRX::on_receive(const MACAddress& addr, const TransceiverHeader* th)
{
    Transceiver::on_receive(addr, th);

    auto& comm = Communicator::instance();
    //LIB_LOGW("RECV:0X%02x", th->rudp.flag);
    
    // Unreliable
    if(!th->isRUDP())
    {
        const Payload* pl = (const Payload*)th->payload();
        if(!pl || pl->magicNo != Payload::MAGIC_NO)
        {
            LIB_LOGW("Invalid unreliable data");
            return;
        }

        // Receive allow connection? (Secondary side)
        if(pl->function == Payload::Function::AllowConnection)
        {
            if(comm.existsPeer(addr))
            {
                LIB_LOGW("Already registered %s", addr.toString().c_str());
                return;
            }
            LIB_LOGE("Receive allowConnection %s", addr.toString().c_str());
            auto& comm = Communicator::instance();
            comm.registerPeer(addr);
            _synInfo[addr].status = SynInfo::State::PostSYN;
            return;
        }
    }

    with_lock([this](const MACAddress& addr, const TransceiverHeader* th)
    {
        auto& comm = Communicator::instance();
        if(th->isSYN())
        {
            // Receive SYN (Primary side)
            if(!th->isACK() && _enableSYN)
            {
                LIB_LOGE("Recv SYN");
                if(comm.existsPeer(addr) || comm.isSecondary() || (_peerMax && comm.numOfPeer() >= _peerMax))
                {
                    LIB_LOGE("Reject SYN request %d:%d:%u/%u",
                             comm.existsPeer(addr), comm.isSecondary(), comm.numOfPeer(), _peerMax);
                    return;
                }
                comm.setRole(Role::Primary);
                Communicator::instance().registerPeer(addr);
                _synInfo[addr].status = SynInfo::State::PostSYNACK;
                return;
            }
            // Receive SYN+ACK and apply config (Secondary side)
            else
            {
                LIB_LOGE("Recv SYNACK");
                if(comm.isPrimary())
                {
                    LIB_LOGE("Reject SYNACK");
                    _synInfo[addr].status = SynInfo::State::None;
                    return;
                }

                // TODO sequence.ack,ackseq....
                comm.setRole(Role::Secondary);
                const RUDP::config_t* cfg = (const RUDP::config_t*)th->payload();
                comm._config = *cfg;
                _synInfo[addr].status = SynInfo::State::PostACK;
            }
        }
    }, addr, th);
}

#if !defined(NDEBUG)
String SystemTRX::debugInfo() const
{
    String s = Transceiver::debugInfo() + '\n';
    with_lock([this, &s]()
    {
        s += formatString("  SynInfo:%zu\n", this->_synInfo.size());
        for(auto& si : this->_synInfo)
        {
            s += formatString("    [%s]:%u:%lu:%llu",
                              si.first.toString().c_str(),
                              si.second.status, si.second.tm, si.second.sequence);
        }
    });
    s.trim();
    return s;
}
#endif
//
}}
