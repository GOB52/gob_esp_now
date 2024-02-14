/*!
  @file gob_transceiver.cpp
  @brief Transceiver(TRX) base class.
  @details The transceiver belongs to the communicator and sends and receives data to and from other transceivers with the same ID on other devices.

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include "gob_transceiver.hpp"
#include "gob_esp_now.hpp"
#include "internal/gob_esp_now_log.hpp"
#include <set>
#include <esp32-hal.h> // millis

namespace goblib { namespace esp_now {

// -----------------------------------------------------------------------------
// class Transceiver
Transceiver::Transceiver()
{
    _sem = xSemaphoreCreateRecursiveMutex();
    assert(_sem);
}

Transceiver::Transceiver(const uint8_t tid) : _tid(tid)
{
    _sem = xSemaphoreCreateRecursiveMutex();
    assert(_sem);
}

Transceiver::~Transceiver()
{
    vSemaphoreDelete(_sem);
}

void Transceiver::reset()
{
    for(auto& it : _peerInfo) { it.second = {}; }
}

void Transceiver::clear(const MACAddress& addr)
{
    _peerInfo.erase(addr);
}

bool Transceiver::delivered(const uint64_t seq)
{
    return std::all_of(_peerInfo.begin(), _peerInfo.end(), [&seq](decltype(_peerInfo)::const_reference e)
    {
        return seq <= e.second.recvAck;
    });
}

uint64_t Transceiver::postReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return 0; }

    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    
    // Unicast
    if(peer_addr)
    {
        auto seq = make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
        return Communicator::instance().postWithLock(peer_addr, buf, sizeof(buf)) ? seq : 0;
    }
    //All peers (Separate to each peer)
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) != ESP_OK) { return 0; }

    uint64_t rseq{};
    do
    {

        auto seq = make_data(buf, RUDP::Flag::ACK, info.peer_addr, data, length);
        if(!Communicator::instance().postWithLock(info.peer_addr, buf, sizeof(buf)))
        {
            LIB_LOGE("Failed to post %s", MACAddress(info.peer_addr).toString().c_str());
            return 0;
        }
        if(!rseq) { rseq = seq; }
        if(rseq != seq)
        {
            LIB_LOGE("Inconsistency in the sequence for each peer\n"
                     "Are you mixing all peers and single peer send?");
            return 0;
        }
    }
    while(esp_now_fetch_peer(false, &info) == ESP_OK);
    return rseq;
}

#if 0
uint64_t Transceiver::sendReliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(!peer_addr || (peer_addr && !esp_now_is_peer_exist(peer_addr))) { return 0; }

    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    auto seq = make_data(buf, RUDP::Flag::ACK, peer_addr, data, length);
    return Communicator::instance().sendWithLock(peer_addr, buf, sizeof(buf)) ? seq : 0;
}
#endif

bool Transceiver::postUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::NONE, peer_addr, data, length);
    return Communicator::instance().postWithLock(peer_addr, buf, sizeof(buf));
}

#if 0
bool Transceiver::sendUnreliable(const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    if(peer_addr && !esp_now_is_peer_exist(peer_addr)) { return false; }
    uint8_t buf[sizeof(TransceiverHeader) + length]{};
    make_data(buf, RUDP::Flag::NONE, peer_addr, data, length);
    return Communicator::instance().sendWithLock(peer_addr, buf, sizeof(buf));
}
#endif

void Transceiver::build_peer_map()
{
    esp_now_peer_info_t info{};
    if(esp_now_fetch_peer(true, &info) == ESP_OK)
    {
        do
        {
            _peerInfo[MACAddress(info.peer_addr)] = {};
        }
        while(esp_now_fetch_peer(false, &info) == ESP_OK);
    }
}

// WARN:Will lock
bool Transceiver::post_rudp(const uint8_t* peer_addr, const RUDP::Flag flag, const void* data, const uint8_t length)
{
    if(!peer_addr) { LIB_LOGE("Need peer_addr"); return false; }
    LIB_LOGD("[RUDP]:[%s]:0x%02x", MACAddress(peer_addr).toString().c_str(), to_underlying(flag));

    uint8_t buf[sizeof(TransceiverHeader) + length];
    make_data(buf, flag, peer_addr, data, length);
    return Communicator::instance().post(peer_addr, buf, sizeof(buf));
}

// WARN:Will lock
uint64_t Transceiver::make_data(uint8_t* obuf, const RUDP::Flag flag, const uint8_t* peer_addr, const void* data, const uint8_t length)
{
    MACAddress addr(peer_addr);
    if(addr.isBroadcast() && to_underlying(flag))
    {
        LIB_LOGE("Addresses not allowed as RUDP destinations %s", addr.toString().c_str());
        return 0;
    }

    // Header
    TransceiverHeader* th = (TransceiverHeader*)obuf;
    th->tid = _tid;
    th->size = sizeof(*th) + length;
    th->rudp.flag = to_underlying(flag);

    uint64_t seq{};
    auto& pi = _peerInfo[addr];
    {
        lock_guard _(_sem);
        pi.needReturnACK = false;

        if(th->isNUL())
        {
            seq = ++pi.sequence;
            th->rudp.sequence = seq & 0xFF;
            pi.sentAck = pi.recvSeq;
            th->rudp.ack = (pi.recvSeq & 0xFF);
        }
        else if(th->isACK())
        {
            // TODO: ここで ++pi.sequnce して post で overflow するとやばい気がする
            // post error の場合 sequence とばないか?
            // そのばあいそもそも破綻するのだからいいのでは説
            seq = length ? ++pi.sequence : ++pi.ackSequence;
            th->rudp.sequence = seq & 0xFF;
            pi.sentAck = pi.recvSeq;
            th->rudp.ack = (pi.recvSeq & 0xFF);
        }
    }
    if(data && length) { std::memcpy(obuf + sizeof(*th), data, length); } // Append payload

    LIB_LOGD("[%s] (0x%02x:S:%u:A:%u P:%u) | S64:%llu A64:%llu",
             addr.toString().c_str(),
             (uint8_t)flag, th->rudp.sequence, th->rudp.ack, th->payloadSize(),
             seq, pi.recvSeq);

    return seq;
}

void Transceiver::_update(const unsigned long ms, const RUDP::config_t& cfg)
{
    // Send ACk force?
    std::set<MACAddress> addrs;

    for(auto& pi : _peerInfo)
    {
        if(!pi.first || !esp_now_is_peer_exist(pi.first.data()) || !pi.second.needReturnACK) { continue; }

        // Send ACK if nothing is sent for a certain period of time after receiving ACK with payload
        auto rtm = pi.second.recvTime;
        if(rtm && ms > rtm + cfg.cumulativeAckTimeout)
        {
            LIB_LOGD(">> ForceACK:Timeout");
            addrs.insert(pi.first);
            continue;
        }
        // Post ACK if the number of unrespond ACKs exceeds a certain number
        if(!cfg.maxCumAck || pi.second.recvSeq > pi.second.sentAck + cfg.maxCumAck)
        {
            LIB_LOGD(">> FoeceACK:Cum");
            addrs.insert(pi.first);
        }
    }
    for(auto& addr : addrs) { post_ack(addr.data()); }
}

void Transceiver::on_receive(const MACAddress& addr, const TransceiverHeader* th)
{
    uint64_t rs{},ra{};
    auto& pi = _peerInfo[addr];
    {
        lock_guard _(_sem);
        pi.recvTime = millis();
    }

    auto base = th->needReturnACK() ? pi.recvSeq : pi.recvAckSeq;
    rs = restore_u64_later(base, th->rudp.sequence);
    ra = restore_u64_later(pi.recvAck, th->rudp.ack);

    LIB_LOGD("[RECV]:%u 0x%02x S:%u A:%u P:%u -> S:%llu => %llu A:%llu => %llu",
             th->tid, th->rudp.flag, th->rudp.sequence, th->rudp.ack, th->payloadSize(),
             base, rs, pi.recvAck, ra);
    
    // isACK include NUL/SYN
    if(th->isACK())
    {
        lock_guard _(_sem);
        LIB_LOGD("<%s>:%u", th->isNUL() ? "NUL" : (th->isSYN() ? "SYN" : "ACK"), th->payloadSize());
        (th->needReturnACK() ?  pi.recvSeq : pi.recvAckSeq) = rs;
        if(ra > pi.recvAck) { pi.recvAck = ra; }
        pi.needReturnACK |= th->needReturnACK();
    }
    if(th->isRST())
    {
        LIB_LOGD("<RST>");
        auto& comm = Communicator::instance();
        comm.notify(Notify::Disconnect, &addr);
    }
    if(th->isNUL())
    {
        post_ack(addr);
    }

    if(th->hasPayload())
    {
        lock_guard _(_sem);
        onReceive(addr, th->payload(), th->payloadSize());
    }
}

#if !defined(NDEBUG)
String Transceiver::debugInfo() const
{
    lock_guard _(_sem);
    String s;
    s = formatString("TID:%u\n", _tid);
    for(auto& r : _peerInfo)
    {
        s += formatString("  [%s] S(S:%llu SA;%llu A:%llu) R(S:%llu A:%llu T:%lu) NRA:%d\n",
                          r.first.toString().c_str(),
                          r.second.sequence, r.second.ackSequence, r.second.sentAck,
                          r.second.recvSeq, r.second.recvAck, r.second.recvTime,
                          r.second.needReturnACK
                          );

    }
    s.trim();
    return s;
}
#endif
//
}}
