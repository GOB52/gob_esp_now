/*
  Transceiver for send/receive button status
  Shared code for examples
 */
#ifndef BUTTON_TRX_HPP
#define BUTTON_TRX_HPP

#include <gob_transceiver.hpp>
#include <utility/Button_Class.hpp> // M5Unified
#include "internal/gob_esp_now_log.hpp"


// Receive button status and update Button_Class
class ButtonTRX : public goblib::esp_now::Transceiver
{
  public:
    using MACAddress = goblib::esp_now::MACAddress;
    using Button_Class = m5::Button_Class;
    template<typename K,typename T> using map_t = goblib::esp_now::map_t<K,T>;

    union ButtonBit
    {
        ButtonBit() : btn(0) {}
        ButtonBit(const bool a, const bool b, const bool c) : btnA(a), btnB(b), btnC(c) {}
        uint8_t btn;
        struct
        {
            uint8_t btnA : 1;
            uint8_t btnB : 1;
            uint8_t btnC : 1;
        };
    } __attribute__((__packed__));

    struct  Button
    {
        ButtonBit bbit;
        std::array<Button_Class, 3> buttons;
    };
    
    explicit ButtonTRX(const uint8_t tid) : goblib::esp_now::Transceiver(tid) {}
    virtual ~ButtonTRX() {}

    inline bool enabled() const { return _enable; }
    inline const Button_Class& BtnA(const MACAddress& addr) { return _button[addr].buttons[0]; }
    inline const Button_Class& BtnB(const MACAddress& addr) { return _button[addr].buttons[1]; }
    inline const Button_Class& BtnC(const MACAddress& addr) { return _button[addr].buttons[2]; }
    inline uint8_t raw(const MACAddress& addr) const
    {
        return _button.count(addr) ? _button.at(addr).bbit.btn : 0;
    }

    void begin() { with_lock([this]() { this->_enable = true; this->_sequence = 0; }); }

    using goblib::esp_now::Transceiver::postReliable;
    bool postReliable(const MACAddress& addr, const bool a, const bool b, const bool c)
    {
        if(with_lock([this](){ return _enable; }))
        {
            ButtonBit bb(a, b, c);
            _sequence = postReliable(addr, bb);
            return (_sequence != 0ULL);
        }
        return false;
    }

  protected:
    virtual void update(const unsigned long ms) override
    {
        // first:MACAddress, second:struct Button
        // Apply received button status to Button_Class
        for(auto& bb : _button)
        {
            for(size_t idx = 0; idx < bb.second.buttons.size(); ++idx)
            {
                bb.second.buttons[idx].setRawState(ms, bb.second.bbit.btn & (1 << idx));
            }
        }
    }

    // Called with exclusive control
    virtual void onReceive(const MACAddress& addr, const void* data, const uint8_t length) override
    {
        if(!_enable)
        {
            LIB_LOGD("Enabled cause by ither device");
            _enable = true;
#if 0
            // Only ACK is sent in the beginning.
            // After that, postReliable also serves as ACK. (ACK with payload)
            //post_ack(addr);
#endif
        }
        assert(length == sizeof(ButtonBit));
        _button[addr].bbit = *(const ButtonBit*)data;
    }

  private:
    map_t<MACAddress, Button> _button;
    uint64_t _sequence{}; // posted sequence
    bool _enable{};
};

#endif
