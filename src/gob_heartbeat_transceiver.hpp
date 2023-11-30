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
class HeartbeatTransceiver :public Transceiver
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
    static constexpr uint8_t DEFAULT_CONSIDER_CONNECTION_LOST = 4;

  private:    
    struct Sent { unsigned long time; uint64_t sequence; };

#if defined(GOBLIB_ESP_NOW_USING_STD_MAP)
    using recv_map_t = std::map<MACAddress, unsigned long>;
#else
    using recv_map_t = vmap<MACAddress, unsigned long>;
#endif
    recv_map_t _recv{};
    std::vector<Sent> _sent;
    bool _began{}, _sender{};
    uint64_t _acked{};
    unsigned long _interval{DEFAULT_INTERVAL}, _lastSent{};
    uint8_t _ccl{DEFAULT_CONSIDER_CONNECTION_LOST};
    MACAddress _addr{};
};
//
}}
#endif
