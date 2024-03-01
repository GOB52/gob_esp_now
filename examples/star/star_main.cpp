/*
  gob_esp_now example: star
  Handshaking for star network
*/
#include <M5Unified.h>
#include <gob_esp_now.hpp>
#include <gob_unifiedButton.hpp> // for CoreS3
#include <WiFi.h>
#include <cmath>
#include "../shared/buttonTRX.hpp"

using namespace goblib::esp_now;

namespace
{
// Communication takes place between devices with the same value of Application ID and Transceiver ID.
constexpr uint8_t APP_ID = 200;
constexpr uint8_t BUTTON_TRANSCEIVER_ID = 100;

auto& lcd = M5.Display;
goblib::UnifiedButton unifiedButton;
auto& comm = Communicator::instance();
ButtonTRX  buttonTRX(BUTTON_TRANSCEIVER_ID);
bool failed{}, dirty{true};
unsigned long brtime{};

enum Mode : uint8_t { Idle, Waiting, Primary, Secondary };
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
        addr = *(const MACAddress*)arg;
        M5_LOGE("CONNECTION LOST\n%s", addr.toString().c_str());
        dirty = true;
        break;
    case Notify::Shookhands:
        addr = *(const MACAddress*)arg;
        M5_LOGI("Complete handshake with %s", addr.toString().c_str());
        mode = comm.isPrimary() ? Mode::Primary : Mode::Secondary;

        buttonTRX.begin();
        comm.unregisterPeer(BROADCAST);

        M5_LOGI("%s", comm.debugInfo().c_str());
        dirty = true;
    default: break;
    }
}


void waitWIFI()
{
    static unsigned long ms{};
    static wl_status_t ps{};
    ps = (wl_status_t)999;
    ms = millis();
    do
    {
        auto s = WiFi.status();
        if(s != ps)
        {
            M5_LOGI("WiFi:%d", s);
            ps = s;
        }
    }
    while(millis() < ms + 5000);
}

void setup()
{
    //esp_log_level_set("*", ESP_LOG_VERBOSE);

    M5_LOGI("Heap:%u", esp_get_free_heap_size());

    M5.begin();
    unifiedButton.begin(&lcd);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    M5_LOGI("  Self: %s", comm.address().toString().c_str());
    comm.registerNotifyCallback(comm_callback);
    comm.registerTransceiver(&buttonTRX);
    
    auto cfg = comm.config();
#if 0
    cfg.retransmissionTimeout = esp_random() % 200;
    cfg.cumulativeAckTimeout = esp_random() % 100;
    cfg.maxRetrans = 1 + esp_random() % 4;;
    cfg.nullSegmentTimeout = 1000 + (esp_random() % 5) * 1000;
    comm.begin(APP_ID, cfg);
#else
    cfg.retransmissionTimeout = 300;
    cfg.cumulativeAckTimeout = 100;
    cfg.maxRetrans = 4;
    //cfg.nullSegmentTimeout = 1000 * 5;
    cfg.nullSegmentTimeout = 0; // 0 means no use heartbeat.
    comm.begin(APP_ID);
#endif
    
    //esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    M5_LOGI("%s", comm.debugInfo().c_str());
    
    lcd.setFont(&fonts::Font4);
    lcd.setTextColor(TFT_WHITE);
    unifiedButton.setFont(&fonts::Font4);
    lcd.clear(TFT_DARKGREEN);
}

void waiting_loop()
{
    lcd.setCursor(0, 0);
    lcd.printf("Waiting handshake...");
    // Timeout?
    if(millis() - brtime > 1000)
    {
        mode = Mode::Idle;
        lcd.clear(TFT_DARKGREEN);
    }
}


void disp()
{
    if(!dirty) { return; }
    dirty = false;
    lcd.clear(TFT_ORANGE);
    
    constexpr int16_t radius = 16;
    constexpr float d2r = M_PI / 180.f;

    auto& comm = Communicator::instance();

    auto& self = comm.address();
    auto& primary = comm.primaryAddress();

    auto cx = lcd.width()/2;
    auto cy = lcd.height()/2;
    auto distance = ((cx > cy) ? cy : cx) - radius;

    uint8_t idx = 0;
    auto v = comm.getPeerAddresses();
    v.push_back(self);
    uint8_t num = v.size() - 1;
    if(!num) { return; }

    for(auto& addr : v)
    {
        if(addr == primary)
        {
            lcd.fillCircle(cx, cy, radius, primary == self ? TFT_RED : TFT_BLUE);
            continue;
        }
        float rad  = (360 / num) * (float)idx * d2r;
        auto x = distance *  cos(rad) + cx;
        auto y = distance * -sin(rad) + cy;
        lcd.fillCircle(x, y, radius, self == addr ? TFT_RED : TFT_BLUE);
        ++idx;
    }
}

void primary_loop()
{
    disp();
    lcd.setCursor(0, 0);
    lcd.printf("%c:%s\n", comm.isPrimary() ? 'P' : 'S', comm.address().toString(true).c_str());

    auto v = comm.getPeerAddresses();
    lcd.printf("Peer:%zu\n", v.size());
    for(auto& addr : v)
    {
        lcd.printf(" [%s]:%02x\n", addr.toString(true).c_str(), buttonTRX.raw(addr));
    }

#if 1
    M5_LOGW("post btn P");
    if(!buttonTRX.postReliable(MACAddress(), M5.BtnA.isPressed(), M5.BtnB.isPressed(), M5.BtnC.isPressed()))
    {
        M5_LOGE("Failed to btn post");
    }
#else
    uint8_t tmp[3]{};
    if(M5.BtnA.wasClicked()) { buttonTRX.postReliable(tmp); }
#endif
}

void secondary_loop()
{
    disp();
    lcd.setCursor(0, 0);
    lcd.printf("%c:%s\n", comm.isPrimary() ? 'P' : 'S', comm.address().toString(true).c_str());
    lcd.printf("P:%s:%02x\n",  comm.primaryAddress().toString(true).c_str(), buttonTRX.raw(comm.primaryAddress()));

#if 1
    M5_LOGW("post btn S");
    if(!buttonTRX.postReliable(comm.primaryAddress(), M5.BtnA.isPressed(), M5.BtnB.isPressed(), M5.BtnC.isPressed()))
    {
        M5_LOGE("Failed to btn post");
    }
#else
    uint8_t tmp[3]{};
    if(M5.BtnA.wasClicked()) { buttonTRX.postReliable(tmp, sizeof(tmp)); }
#endif

}

void idle_loop()
{
    // Broadcasting
    if(comm.isNoRole() && M5.BtnA.wasClicked())
    {
        M5_LOGI("Broadcast");
        if(comm.broadcastHandshake())
        {
            brtime = millis();
            mode = Mode::Waiting;
            lcd.clear(TFT_DARKGRAY);
            return;
        }
        else
        {
            M5_LOGE("Failed to broadcastHandshake");
        }
    }
}

void loop()
{
    M5.update();
    unifiedButton.update();

    switch(mode)
    {
    case Mode::Primary:   primary_loop();   break;
    case Mode::Secondary: secondary_loop(); break;
    case Mode::Waiting:   waiting_loop();   break;
    default: idle_loop(); break;
    }
    if(M5.BtnPWR.wasClicked()) { M5_LOGI("%s", comm.debugInfo().c_str()); }

    unifiedButton.draw();
}
