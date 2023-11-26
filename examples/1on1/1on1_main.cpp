/*
  gob_esp_now example: 1on1
    1 on 1 sending and receiving
*/
#include <M5Unified.h>
#include <gob_esp_now.hpp>
#include <gob_unifiedButton.hpp> // for CoreS3
#include <WiFi.h>

#if !defined(TARGET_ADDR)
// Specify with compile options or define here.
//#define TARGET_ADDR "12:34:56:67:9a:bc"
#error "Need TARGET_ADDR defines is required. See also platformio.ini"
#endif

using namespace goblib::esp_now;

namespace
{
constexpr uint8_t APP_ID = 12;
constexpr uint8_t BUTTON_TRANSCEIVER_ID = 34;
constexpr uint8_t COLOR_TRANSCEIVER_ID = 56;

MACAddress dest(TARGET_ADDR);

// Data structures for transmission and reception, and transceivers
union ButtonData
{
    volatile uint8_t btn{};
    struct
    {
        uint8_t btnA : 1;
        uint8_t btnB : 1;
        uint8_t btnC : 1;
    };
};

// Button Status Transceiver
class ButtonTransceiver : public Transceiver
{
  public:
    explicit ButtonTransceiver(const uint8_t tid) : Transceiver(tid) {}

    bool received() const { return with_lock([this](){ return _received; }); }

    void clear() { _received = false; }
    const ButtonData& data() const { return _data; }
    uint64_t rcount() const { return _count; }
    
    // Transmitted data with APP_ID and Transceiver ID matching comes in. (Called from another task)
    virtual void onReceive(const MACAddress& addr, const TransceiverHeader* data) override
    {
        with_lock([this](const TransceiverHeader* th)
        {
            auto pd = (ButtonData*)th->payload();
            M5_LOGI("[RB]:%u:Btn:%x", th->rudp.sequence, pd->btn);
            this->_data = *pd;
            this->_received = true;
            this->_count++;
        }, data);
    }

    virtual void onNotify(const Notify notify, void* arg) override
    {
        if(notify == Notify::Disconnect)
        {
            const MACAddress* addr = (const MACAddress*)arg;
            M5_LOGE("Connection lost %s", addr->toString().c_str());
        }
    }
    
  private:
    ButtonData _data{};
    volatile bool _received{};
    volatile uint64_t _count{};
};

class ColorTransceiver : public Transceiver
{
  public:
    explicit ColorTransceiver(const uint8_t tid) : Transceiver(tid) {}

    bool received() const { return with_lock([this](){ return _received; }); }

    void clear() { _received = false; }
    uint16_t data() const { return _color; }
    uint64_t rcount() const { return _count; }

    // Transmitted data with APP_ID and Transceiver ID matching comes in. (Called from another task)
    virtual void onReceive(const MACAddress& addr, const TransceiverHeader* data) override
    {
        with_lock([this](const TransceiverHeader* th)
        {
            M5_LOGD("Clr");
            auto pd = (uint16_t*)th->payload();
            M5_LOGI("[RC]:%u:Clr:%x", th->rudp.sequence, *pd);
            this->_color = *pd;
            this->_received = true;
            this->_count++;
        }, data);
    }

    virtual void onNotify(const Notify notify, void* arg) override
    {
        if(notify == Notify::Disconnect)
        {
            const MACAddress* addr = (const MACAddress*)arg;
            M5_LOGE("Connection lost %s", addr->toString().c_str());
        }
    }
    
  private:
    uint16_t _color{};
    volatile bool _received{};
    volatile uint64_t _count{};
};


// TODO
// 受信失敗時の再送要求機構 with SEQが飛んだ場合の対処
// TODO

ButtonTransceiver btnT(BUTTON_TRANSCEIVER_ID);
ColorTransceiver colorT(COLOR_TRANSCEIVER_ID);
auto& lcd = M5.Display;
goblib::UnifiedButton unifiedButton;
}

void setup()
{
    M5_LOGI("Heap:%u", esp_get_free_heap_size());
    //esp_log_level_set(goblib::esp_now::LIB_TAG, ESP_LOG_VERBOSE);
    //esp_log_level_set(goblib::esp_now::LIB_TAG, ESP_LOG_DEBUG);
    //esp_log_level_set(goblib::esp_now::LIB_TAG, ESP_LOG_WARN);
    esp_log_level_set("*", ESP_LOG_NONE);
    M5_LOGI("LogLevel:%u", esp_log_level_get(goblib::esp_now::LIB_TAG));
    
    M5.begin();
    unifiedButton.begin(&lcd);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);

    auto& comm = Communicator::instance();
    comm.begin(APP_ID); // Set unique application identifier
    Communicator::registerPeer(dest);
    Communicator::registerPeer(MACAddress("11:22:33:44:55:66")); // nonexistent address
    comm.registerTransceiver(&colorT);
    comm.registerTransceiver(&btnT);
    //comm.setLossOfConnectionTime(1000 * 10);

#if !defined(NDEBUG)
    M5_LOGI("%s", comm.debugInfo().c_str());

    // Using debug features
    //comm.enableDebug(true);
    //comm.setDebugSendLoss(0.25f);
    //comm.setDebugSendLoss(0.5f);
    //comm.setDebugSendLoss(0.9f);
#endif
    
    lcd.setFont(&fonts::Font4);
    unifiedButton.setFont(&fonts::Font2);
    lcd.clear(TFT_DARKGREEN);
    lcd.setTextColor(TFT_BLACK, TFT_DARKGREEN);

    M5_LOGI("  Self: %s", comm.address().toString().c_str());
    M5_LOGI("Target: %s", dest.toString().c_str());
}

void loop()
{
    static unsigned long ms = millis();
    bool dirty{};
    
    M5.update();
    unifiedButton.update();
    
    // Send button status when changes.
    static ButtonData last;
    ButtonData d;
    d.btnA = M5.BtnA.isPressed();
    d.btnB = M5.BtnB.isPressed();
    d.btnC = M5.BtnC.isPressed();
    if(last.btn != d.btn)
    {
        M5_LOGI("Post btn:%x", d.btn);
        // Data accumulation(Send not yet).
#if 1
        if(!btnT.postReliable(dest, d)) { M5_LOGE("Failed to post"); } // => *1 send on update() 
#else
        if(!btnT.sendReliable(dest, d)) { M5_LOGE("Failed to send"); } // Send directly
#endif
        last = d;
    }

    // Change background color every 30 seconds and send
    // The sending side is assumed to be the side with the lower MACAddress.
    auto now = millis();
    if(now - ms >= 1000 * 30 && Communicator::instance().address() < dest)
    {
        uint16_t color = esp_random() & 0xFFFF;
        M5_LOGI("Post color:%x", color);
        lcd.clear(color);
        lcd.setTextColor(color ^ 0xFFFF, color);
        // If no destination is specified, send to the registered peer.
        if(!colorT.postReliable(color)) { M5_LOGE("Failed to post"); }
        ms = now;
        dirty = true;
    }

#if !defined(NDEBUG)
    if(M5.BtnC.wasHold())
    {
        M5_LOGI("%s", Communicator::instance().debugInfo().c_str());
    }
#endif

    // Change to received background color.
    if(colorT.received())
    {
        colorT.with_lock([]()
        {
            uint16_t color = colorT.data();
            colorT.clear();
            lcd.clear(color);
            lcd.setTextColor(color ^ 0xFFFF, color);
        });
        dirty = true;

        // Return ACK
        ///colorT.postReliable(dest);
    }

    // [*1] Send data in this function if posted.
    Communicator::instance().update(); 

    // Draw my button status.
    lcd.fillRect(16 + (64+16) * 0, 160, 64,32, d.btnA ? TFT_WHITE : TFT_BLACK);
    lcd.fillRect(16 + (64+16) * 1, 160, 64,32, d.btnB ? TFT_WHITE : TFT_BLACK);
    lcd.fillRect(16 + (64+16) * 2, 160, 64,32, d.btnC ? TFT_WHITE : TFT_BLACK);
    
    // Draw the received button status.
    static ButtonData rd;
    if(btnT.received())
    {
        // Functions called by with_lock are thread-safe
        btnT.with_lock([]()
        {
            rd = btnT.data();
            btnT.clear();
        });
    }
    lcd.fillRect(16 + (64+16) * 0, 120, 64,32, rd.btnA ? TFT_WHITE : TFT_BLACK);
    lcd.fillRect(16 + (64+16) * 1, 120, 64,32, rd.btnB ? TFT_WHITE : TFT_BLACK);
    lcd.fillRect(16 + (64+16) * 2, 120, 64,32, rd.btnC ? TFT_WHITE : TFT_BLACK);

    //
    lcd.setCursor(0,0);
    lcd.printf("[B]S:%llu A:%llu R:%llu", btnT.sequence(), btnT.ack(dest), btnT.rcount());
    lcd.setCursor(0,26*1);
    lcd.printf("[C]S:%llu A:%llu R:%llu", colorT.sequence(), colorT.ack(dest), colorT.rcount());
    lcd.setCursor(0,26*2);
    lcd.printf("Heap:%u", esp_get_free_heap_size());
    lcd.setCursor(0,26*3);
    lcd.printf("MAC:%s", Communicator::instance().address().toString(true).c_str());

    unifiedButton.draw(dirty);
}
