/*
  Transceiver for send/receive button status
  Shared code for examples
 */
#ifndef BUTTON_TRX_HPP
#define BUTTON_TRX_HPP

#include <gob_transceiver.hpp>
#include <utility/Button_Class.hpp>

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
    };

    struct  Button
    {
        ButtonBit bbit;
        std::array<Button_Class, 3> buttons;
    };
    
    explicit ButtonTRX(const uint8_t tid) : goblib::esp_now::Transceiver(tid) { _time = millis(); }
    virtual ~ButtonTRX() {}

    inline bool enabled() const { return _enable; }
    inline const Button_Class& BtnA(const MACAddress& addr) { return _button[addr].buttons[0]; }
    inline const Button_Class& BtnB(const MACAddress& addr) { return _button[addr].buttons[1]; }
    inline const Button_Class& BtnC(const MACAddress& addr) { return _button[addr].buttons[2]; }

    void begin() { _enable = true; _sequence = 0; }
    bool postReliable(const MACAddress& addr, bool a, bool b, bool c)
    {
        if(with_lock([this](){ return _enable; }))
        {
            ButtonBit bb(a, b, c);
            _sequence = goblib::esp_now::Transceiver::postReliable(addr, bb);
            return _sequence != 0;
        }
        return false;
    }
    
  protected:
    // Called with exclusive control
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
            _enable = true;
            // Only ACK is sent in the beginning.
            // After that, postReliable also serves as ACK. (ACK with payload)
            //post_ack(addr);
        }
        assert(length == sizeof(ButtonBit));
        _button[addr].bbit = *(const ButtonBit*)data;
    }

  private:
    map_t<MACAddress, Button> _button;
    uint64_t _sequence{}; // posted sequence
    unsigned long _time{};
    bool _enable{};
};

#endif
