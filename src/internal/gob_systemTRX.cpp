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

void SystemTRX::enableHandshake(const bool enable)
{
    with_lock([this, &enable]()
    {
        _enableHandshake = enable;
        if(!enable)
        {
            for(auto& si : _synInfo) { si.second.status = SynInfo::State::None; }
        }
    });
}

void SystemTRX::setMaxHandshakePeer(const uint8_t num)
{
    with_lock([this, &num]()
    {
        if(num && num < _handshaked)
        {
            LIB_LOGE("Failed already handshaked %u/%u", _handshaked, num);
            return;
        }
        _maxPeer = num;
    });
}

bool SystemTRX::broadcastHandshake()
{
    Payload pl;
    pl.function = Payload::Function::AllowConnection;
    return postUnreliable(goblib::esp_now::BROADCAST, pl);
}

bool SystemTRX::postSYN(const MACAddress& addr, const config_t& cfg)
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
        auto pi = peerInfo(it->first);
        if(pi)
        { 
            switch(it->second.status)
            {
            case SynInfo::State::None: break;
                // Primary side
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
                // Secondary side
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
                // Already posted force by parent class?
                if(it->second.recvSeqSynAck && pi->sentAck >= it->second.recvSeqSynAck)
                {
                    LIB_LOGE("Already posted ACK");
                    it->second.status = SynInfo::State::Shookhand;
                    break;
                }
                LIB_LOGD("Post ACK");
                if(post_ack(it->first)) { it->second.status = SynInfo::State::Shookhand; }
                else { LIB_LOGE("Failed to postACK"); }
                break;

            case SynInfo::State::Shookhand:
                LIB_LOGE("Shookhand %s", it->first.toString().c_str());
                comm.notify(Notify::Shookhands, &it->first);
                it->second.status = SynInfo::State::None;
                ++_handshaked;
                break;
            }
        }
        else
        {
            it->second.status = SynInfo::State::None;
        }
        it = (it->second.status == SynInfo::State::None) ? _synInfo.erase(it) : std::next(it);
    }
}

// No locked
void SystemTRX::on_receive(const MACAddress& addr, const TransceiverHeader* th)
{
    Transceiver::on_receive(addr, th);

    if(!_enableHandshake) { return; }
    
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

        // Receive handshake broadcast? (Secondary side)
        if(pl->function == Payload::Function::AllowConnection)
        {
            if(comm.existsPeer(addr))
            {
                LIB_LOGW("Already registered %s", addr.toString().c_str());
                return;
            }
            LIB_LOGE("Receive Handshake %s", addr.toString().c_str());
            auto& comm = Communicator::instance();
            if(comm.registerPeer(addr))
            {
                _synInfo[addr].status = SynInfo::State::PostSYN;
            }
            else
            {
                LIB_LOGE("Failed to register peer %s %u/%u", addr.toString().c_str(), _handshaked, _maxPeer);
                _synInfo[addr].status = SynInfo::State::None;
            }
            return;
        }
    }

    with_lock([this](const MACAddress& addr, const TransceiverHeader* th)
    {
        auto& comm = Communicator::instance();
        auto pi = peerInfo(addr);
        assert(pi);

        if(th->isSYN())
        {
            // Primary side
            // Receive SYN
            if(!th->isACK())
            {
                LIB_LOGE("Recv SYN");
                if(comm.existsPeer(addr) || comm.isSecondary() || (_maxPeer && _handshaked >= _maxPeer))
                {
                    LIB_LOGE("Reject SYN request %d:%d:%u/%u",
                             comm.existsPeer(addr), comm.isSecondary(), _handshaked, _maxPeer);
                    return;
                }
                comm.setRole(Role::Primary);
                if(Communicator::instance().registerPeer(addr))
                {
                    _synInfo[addr].status = SynInfo::State::PostSYNACK;
                }
                else
                {
                    LIB_LOGE("Failed to register peer %s %u/%u", addr.toString().c_str(), _handshaked, _maxPeer);
                    _synInfo[addr].status = SynInfo::State::None;
                }
            }
            // Secondary side
            // Receive SYN+ACK and apply config
            else
            {
                LIB_LOGE("Recv SYNACK");
                if(_synInfo[addr].status != SynInfo::State::WaitSYNACK || !comm.isNoRole())
                {
                    LIB_LOGE("Reject SYNACK %d", _synInfo[addr].status);
                    _synInfo[addr].status = SynInfo::State::None;
                    return;
                }
                _synInfo[addr].recvSeqSynAck = pi->recvSeq;
                
                // TODO sequence.ack,ackseq....
                comm.setRole(Role::Secondary);
                auto cfg = (const config_t*)th->payload();
                comm._config = *cfg;
                comm._primaryAddr = addr;
                _synInfo[addr].status = SynInfo::State::PostACK;
            }
        }
    }, addr, th);
}

#if !defined(NDEBUG)
String SystemTRX::debugInfo() const
{
    String s = Transceiver::debugInfo() + '\n';
    s += formatString("  sysTRX %d: %u/%u\n", _enableHandshake, _handshaked, _maxPeer);
    with_lock([this, &s]()
    {
        s += formatString("  SynInfo:%zu\n", this->_synInfo.size());
        for(auto& si : this->_synInfo)
        {
            s += formatString("    [%s]:%u:%lu:%llu:%llu",
                              si.first.toString().c_str(),
                              si.second.status, si.second.tm, si.second.sequence, si.second.recvSeqSynAck);
        }
    });
    s.trim();
    return s;
}
#endif
//
}}
