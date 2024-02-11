/*!
  @file gob_systemTRX.hpp
  @brief System transceiver for communicator

  @copyright 2023 GOB
  @copyright Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/
#include "gob_transceiver.hpp"

namespace goblib { namespace esp_now {

class Comminicator;

class SystemTRX : public Transceiver
{
  public:
    SystemTRX();
    virtual ~SystemTRX();

    bool postSYN(const MACAddress& addr, RUDP::config_t& cfg);

  protected:
    virtual void update(const unsigned long ms) override;
    virtual void on_receive(const MACAddress& addr, const TransceiverHeader* th) override;

  private:
    bool _retunACK{}, _returnSYNACK{};
    std::vector<MACAddress> _postSynAddr;
    std::vector<MACAddress> _recvSynAddr;
};
//
}}
