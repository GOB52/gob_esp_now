/*!
  @file gob_heartbeat_transceiver.hpp
  @brief Transceiver for heartbeat (Alive monitoring)
*/
#ifndef GOBLIB_HEARTBEAT_TRANSCEIVER_HPP
#define GOBLIB_HEARTBEAT_TRANSCEIVER_HPP

#include "gob_esp_now.hpp"

namespace goblib { namespace esp_now {

/*! @class HeartbeatTransciver
  @brief Transceiver for heartbeat (Alive monitoring)
 */
class HeartbeatTransceiver :  public Transceiver
{
  public:
    explicit HeartbeatTransceiver(const uint8_t tid) : Transceiver(tid) {}
    HeartbeatTransceiver() = delete;

    //inline unsigned long interval() const { return _interval; }   //!< @brief Gets the heartbeat interval
    //inline void setInterval(unsigned long ms) { _interval = ms; } //!< @brief Set the heartbeat interval
    //inline void setDestination(const uint8_t* peer_addr) { _addr = MACAddress(peer_addr); }
    
    void begin();
    inline void end()   { _do = false; }
    
  protected:
    virtual void update(const unsigned long ms) override;
    virtual void onReceive(const MACAddress& addr, const TransceiverHeader* data) override;
    virtual void onNotify(const Notify notify, const void* arg) override;

    void post_heart_beat(const unsigned long ms);
    
  private:
    static constexpr unsigned long DEFAULT_INTERVAL = 1000 * 5;
    unsigned long _interval{DEFAULT_INTERVAL};
    unsigned long _sent{}, _sentBasis{};
    std::map<MACAddress,unsigned long> _recv;
    bool _do{};
    //MACAddress _addr;
};

//
}}
#endif
