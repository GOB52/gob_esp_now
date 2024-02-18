/*
  gob_esp_now example: star
  Handshaking for star network
*/
#include <M5Unified.h>
#include <gob_esp_now.hpp>
#include <gob_unifiedButton.hpp> // for CoreS3
#include <WiFi.h>
#include <cmath>

using namespace goblib::esp_now;

namespace
{
// Communication takes place between devices with the same value of Application ID and Transceiver ID.
constexpr uint8_t APP_ID = 200;
constexpr uint8_t BUTTON_TRANSCEIVER_ID = 100;

auto& lcd = M5.Display;
goblib::UnifiedButton unifiedButton;
bool failed{}, dirty{true};

enum Mode : uint8_t { Idle, Waiting, Connect, Failed };
Mode mode{Idle};

//
}


// This callback is called in the same task as Communicator::update()
void comm_callback(const Notify notify, const void* arg)
{
    MACAddress addr;
    switch(notify)
    {
    case Notify::ConnectionLost:
        failed = true;
        /*
        addr = *(const MACAddress*)arg;
        lcd.clear(0);
        lcd.setCursor(0,0);
        lcd.printf("CONNECTION LOST\n%s", addr.toString(true).c_str());
        */
        break;
    case Notify::Shookhands:
        addr = *(const MACAddress*)arg;
        M5_LOGI("Complete handshake with %s", addr.toString().c_str());
        dirty = true;
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

    auto& comm = Communicator::instance();
    //comm.registerTransceiver(&buttonTRX);
    M5_LOGI("  Self: %s", comm.address().toString().c_str());
    comm.registerNotifyCallback(comm_callback);

    auto cfg = comm.config();
    cfg.retransmissionTimeout = 300;
    cfg.cumulativeAckTimeout = 100;
    cfg.maxRetrans = 4;
    cfg.nullSegmentTimeout = 1000 * 5;

    comm.begin(APP_ID, cfg);
    
    auto after = esp_get_free_heap_size();
    M5_LOGI("library usage:%u", before - after);

    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    M5_LOGI("%s", comm.debugInfo().c_str());
    
    //lcd.setFont(&fonts::Font4);
    unifiedButton.setFont(&fonts::Font4);
    lcd.clear(TFT_DARKGREEN);
}

void disp()
{
    constexpr int16_t radius = 16;
    constexpr float d2r = M_PI / 180.f;

    auto& comm = Communicator::instance();
    //if(comm.isNoRole()) { return; }

    auto& self = comm.address();
    auto& primary = comm.primaryAddress();

    auto cx = lcd.width()/2;
    auto cy = lcd.height()/2;
    auto distance = ((cx > cy) ? cy : cx) - radius;


    ///// TODO    

    uint8_t idx = 0;
    auto v = comm.getPeerAddresses();
    uint8_t num = comm.numOfPeer();
    for(auto& addr : v)
    {
        if(addr == primary) { continue; }

        float rad  = (360 / num) * (float)idx * d2r;
        auto x = distance *  cos(rad) + cx;
        auto y = distance * -sin(rad) + cy;
        lcd.fillCircle(x, y, radius, self == addr ? TFT_RED : TFT_BLUE);
        ++idx;
    }
    lcd.fillCircle(cx, cy, radius, primary == self ? TFT_RED : TFT_BLUE);
}

void loop()
{
    auto& comm = Communicator::instance();
    
    M5.update();
    unifiedButton.update();

    #if 0
    switch(mode)
    {
    case Mode::Idle:    idle_loop(); break;
    case Mode::Host:    host_loop(); break;
        //    case Mode::Connect:
    default: break;
    }
    #endif
    
    // Broadcasting
    if(comm.isNoRole() && M5.BtnA.wasClicked())
    {
        M5_LOGI("Allow");
        if(comm.broadcastAllowConnection())
        {
            lcd.clear(0);
            dirty = true;
        }
        else
        {
            M5_LOGE("Failed to broadcastAllowConnection");
        }
    }

    comm.update();

    if(M5.BtnPWR.wasClicked())
    {
        M5_LOGI("%s", comm.debugInfo().c_str());
    }
    
    if(dirty) { disp(); dirty = false; }
    unifiedButton.draw();
}
