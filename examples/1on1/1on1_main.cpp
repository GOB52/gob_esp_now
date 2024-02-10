/*
  gob_esp_now example: 1on1
  Send the status of M5.BtnX to the other device.
*/
#include <M5Unified.h>
#include <gob_esp_now.hpp>
#include <gob_unifiedButton.hpp> // for CoreS3
#include <WiFi.h>
#include "../shared/buttonTRX.hpp"

/*
  WARNING
  Set MAC Address string of two devices by compile option or direct description
  e.g. #define DEVICE_A "11:22:33:44:55"
 */
#if !defined(DEVICE_A)
#error Need define DEVICE_A
#endif
#if !defined(DEVICE_B)
#error Need define DEVICE_B
#endif

using namespace goblib::esp_now;

namespace
{
// Communication takes place between devices with the same value of Application ID and Transceiver ID.
constexpr uint8_t APP_ID = 11;
constexpr uint8_t BUTTON_TRANSCEIVER_ID = 1;

MACAddress devices[] =
{
    MACAddress(DEVICE_A),
    MACAddress(DEVICE_B),
};
MACAddress target;
ButtonTRX  buttonTRX(BUTTON_TRANSCEIVER_ID);

auto& lcd = M5.Display;
auto& comm = Communicator::instance();
goblib::UnifiedButton unifiedButton;
bool failed{};

#if 0
const char* button_state_string[] =
{
    "nochange ",
    "clicked  ",
    "hold     ",
    "decide_cc",
};
const char* bstr(const m5::Button_Class::button_state_t s)
{
    return button_state_string[(uint8_t)s];
}
#endif

//
}

// This callback is called in the same task as Communicator::update()
void comm_callback(const Notify notify, const void* arg)
{
    failed = true;
    MACAddress addr;
    switch(notify)
    {
    case Notify::ConnectionLost:
        addr = *(const MACAddress*)arg;
        lcd.clear(0);
        lcd.setCursor(0,0);
        lcd.printf("CONNECTION LOST\n%s", addr.toString(true).c_str());
        break;
    default: break;
    }
}

void setup()
{
    esp_log_level_set("*", ESP_LOG_DEBUG);

    M5_LOGI("Heap:%u", esp_get_free_heap_size());
    M5.begin();
    unifiedButton.begin(&lcd);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);

    auto before = esp_get_free_heap_size();

    comm.registerTransceiver(&buttonTRX);
    M5_LOGI("  Self: %s", comm.address().toString().c_str());
    for(auto& addr : devices)
    {
        if(!addr || addr == comm.address()) { continue; }
        M5_LOGI("Target: %s", addr.toString().c_str());
        comm.registerPeer(addr);
        target = addr;
    }
    comm.setRole(comm.address() == devices[0] ? Communicator::Role::Primary : Communicator::Role::Secondary);
    comm.registerNotifyCallback(comm_callback);

    if(!target || !comm.existsPeer(target))
    {
        M5_LOGE("Not registered target");
        lcd.clear(TFT_RED);
        while(true) { delay(1000); }
    }
    
    auto cfg = comm.config();
    cfg.retransmissionTimeout = 300;
    cfg.cumulativeAckTimeout = 100;
    cfg.maxRetrans = 4;
    //cfg.nullSegmentTimeout = 1000 * 5;
    cfg.nullSegmentTimeout = 0; // 0 means no use heartbeat.

    comm.begin(APP_ID, cfg);
    
    auto after = esp_get_free_heap_size();
    M5_LOGI("library usage:%u", before - after);

    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    M5_LOGI("%s", comm.debugInfo().c_str());
    
    lcd.setFont(&fonts::Font4);
    unifiedButton.setFont(&fonts::Font4);
    lcd.clear(TFT_DARKGREEN);
}

void loop()
{
    bool dirty{};
    
    M5.update();
    unifiedButton.update();

    // Starts when any key is pressed or received from the other device.
    if(M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed())
    {
        buttonTRX.begin();
    }
    if(M5.BtnPWR.wasClicked())
    {
        M5_LOGI("%s", comm.debugInfo().c_str());
    }
    
    // Post my button status if TRX has begun.(Check in postReliable)
    buttonTRX.postReliable(target, M5.BtnA.isPressed(), M5.BtnB.isPressed(), M5.BtnC.isPressed());

    comm.update();
    unifiedButton.draw(dirty);

    if(failed) { return; }

    // Can get the other device's button state like M5.BtnX
    auto& a = buttonTRX.BtnA(target);
    auto& b = buttonTRX.BtnB(target);
    auto& c = buttonTRX.BtnC(target);

    lcd.setCursor(0,0);
    lcd.printf("%s Heap:%u\n", buttonTRX.enabled() ? "COMM" : "NONE", esp_get_free_heap_size());
    lcd.printf("  Self:%s\n", Communicator::instance().address().toString(true).c_str());
    lcd.printf("Target:%s\n", target.toString(true).c_str());

    // Print button status that received from target
    lcd.setCursor(0,96);
    /*
    lcd.printf(" BtnA: %s\n", bstr(a.getState()));
    lcd.printf(" BtnB: %s\n", bstr(b.getState()));
    lcd.printf(" BtnC: %s\n", bstr(c.getState()));
    */
    lcd.printf(" A: P:%dR:%dH:%dWC:%dWDC:%d\n",
               a.isPressed(), a.isReleased(), a.isHolding(), a.wasClicked(), a.wasDoubleClicked());
    lcd.printf(" B: P:%dR:%dH:%dWC:%dWDC:%d\n",
               b.isPressed(), b.isReleased(), b.isHolding(), b.wasClicked(), b.wasDoubleClicked());
    lcd.printf(" C: P:%dR:%dH:%dWC:%dWDC:%d\n",
               c.isPressed(), c.isReleased(), c.isHolding(), c.wasClicked(), c.wasDoubleClicked());

}
