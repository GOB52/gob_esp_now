/*
  start synchronization TRX
 */
#ifndef START_TRX_HPP
#define START_TRX_HPP

#include <gob_transceiver.hpp>
#include <time.h>
#include <internal/gob_esp_now_log.hpp>

class StartTRX : public goblib::esp_now::Transceiver
{
  public:
    using MACAddress = goblib::esp_now::MACAddress;
    using goblib::esp_now::Transceiver::postReliable;

    struct Payload
    {
        time_t tm;
        uint8_t stage;
    };

    explicit StartTRX(const uint8_t tid) : goblib::esp_now::Transceiver(tid) {}

    time_t time() const { return with_lock([this]() { return this->_payload.tm; }); }
    uint8_t stage() const { return with_lock([this]() { return this->_payload.stage; }); }
    
    uint64_t post(const uint8_t stage, const time_t& tm)
    {
        _payload = { tm, stage };
        return (_seq = postReliable(_payload));
    }

  protected:
    virtual void onReceive(const MACAddress& addr, const void* data, const uint8_t length)
    {
        LIB_LOGE("-- recv STRX");
        auto payload = (const Payload*)data;
        _payload = *payload;
        post_ack(addr);
    }

  private:
    Payload _payload{};
    uint64_t _seq{};
};
#endif
