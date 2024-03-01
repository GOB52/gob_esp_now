/*!
  @file gob_systemTRX.hpp
  @brief System transceiver for communicator

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include "gob_transceiver.hpp"
#include "gob_esp_now_vmap.hpp"
#include "gob_esp_now_config.hpp"

namespace goblib { namespace esp_now {

class Comminicator;

class SystemTRX : public Transceiver
{
  public:
    SystemTRX();
    virtual ~SystemTRX();

    // Handshake
    inline bool canHandshake() const { return _enableHandshake; }
    inline bool isHandshakeAllowed() const { return canHandshake(); }
    inline bool isHandshakeDenied() const  { return !canHandshake(); }
    void enableHandshake(const bool enable);
    inline uint8_t  getMaxHandshakePeer() const { return _maxPeer; }
     void setMaxHandshakePeer(const uint8_t num);
    bool broadcastHandshake();
    bool postSYN(const MACAddress& addr, const config_t& cfg);

#if !defined(NDEBUG)
    virtual String debugInfo() const override;
#endif
    
  protected:
    virtual void update(const unsigned long ms) override;
    virtual void on_receive(const MACAddress& addr, const TransceiverHeader* th) override;

  private:
    bool _enableHandshake{true};
    uint8_t _maxPeer{}; // 0 means as much as memory and ESP-NOW will allow
    uint8_t _handshaked{};
    
    // For handshake
    struct SynInfo
    {
        static constexpr unsigned long TIMEOUT = 1000 * 5;
        enum State : uint8_t
        {
            None,
            PostSYNACK, // (P)
            WaitACK,    // (P)
            PostSYN,    // (S)
            WaitSYNACK, // (S)
            PostACK,    // (S)
            Shookhand,  // (P/S)
        };
        State status{State::None};
        unsigned long tm{};
        uint64_t sequence{};
        uint64_t recvSeqSynAck{}; // received SYNACK sequence no
    };
    map_t<MACAddress, SynInfo> _synInfo;
    friend class Communicator;
};
//
}}
