/*!
  @file gob_systemTRX.hpp
  @brief System transceiver for communicator

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include "gob_transceiver.hpp"
#include "gob_esp_now_vmap.hpp"

namespace goblib { namespace esp_now {

class Comminicator;

class SystemTRX : public Transceiver
{
  public:
    SystemTRX();
    virtual ~SystemTRX();

    void acceptSYN(const bool enable) { _enableSYN = enable; }
    bool broadcastAllowConnection();
    bool postSYN(const MACAddress& addr, RUDP::config_t& cfg);

#if !defined(NDEBUG)
    virtual String debugInfo() const override;
#endif
    
  protected:
    virtual void update(const unsigned long ms) override;
    virtual void on_receive(const MACAddress& addr, const TransceiverHeader* th) override;

  private:
    bool _enableSYN{true};
    uint8_t _peerMax{}; // 0 means as much as memory and ESP-NOW will allow
    
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
    };
    map_t<MACAddress, SynInfo> _synInfo;

    //std::vector<MACAddress> _postSynAddr;
    //std::vector<MACAddress> _recvSynAddr;

    friend class Communicator;
};
//
}}
