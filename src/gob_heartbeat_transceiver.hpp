/*!
  @file gob_heartbeat_transceiver.hpp
  @brief Transceiver for heartbeat (Alive monitoring)
*/
#ifndef GOBLIB_HEARTBEAT_TRANSCEIVER_HPP
#define GOBLIB_HEARTBEAT_TRANSCEIVER_HPP

#include "gob_esp_now.hpp"

namespace goblib { namespace esp_now {

/*! @class HeartbeatTransceiver
  @brief Transceiver for heartbeat (Alive monitoring)  
  If no ACK is returned within twice the specified interval, the connection is considered lost.
*/
class HeartbeatTransceiver :  public Transceiver
{
  public:
    explicit HeartbeatTransceiver(const uint8_t tid) : Transceiver(tid) {}
    HeartbeatTransceiver() = delete;

    inline unsigned long interval() const { return _interval; }   //!< @brief Gets the heartbeat interval
    inline void setInterval(unsigned long ms); //!< @brief Set the heartbeat interval
    inline void setDestination(const uint8_t* peer_addr) { _addr = MACAddress(peer_addr); } //!< @brief set target address

    /*!
      @brief begin heartbeat
      @param sender True:sender False:receiver
    */
    bool begin(const bool sender);
    void end();
    
  protected:
    virtual void update(const unsigned long ms) override;
    void update_sender(const unsigned long ms);
    void update_receiver(const unsigned long ms);

    virtual void onReceive(const MACAddress& addr, const TransceiverHeader* data) override;
    virtual void onNotify(const Notify notify, const void* arg) override;

    void post_heart_beat(const unsigned long ms);
    void post_ack(const MACAddress& addr);

    static constexpr unsigned long DEFAULT_INTERVAL = 1000 * 10;
    unsigned long _sent{}, _sentBasis{};
    std::map<MACAddress,unsigned long> _recv{};

  private:
    bool _began{}, _sender{};
    unsigned long _interval{DEFAULT_INTERVAL};
    MACAddress _addr{};
};
//
}}
#endif
