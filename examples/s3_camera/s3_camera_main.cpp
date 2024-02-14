/*
  gob_esp_now example: s3_camera
  Sending and receiving camera-captured images (Sender must be CoreS3, Core,Core2 only receiver)
*/
#include <M5Unified.h>
#include <gob_esp_now.hpp>
#include <gob_unifiedButton.hpp> // for CoreS3
#include <WiFi.h>
#include <esp_camera.h>
#include "MainClass.h"
#include "imageTRX.hpp"
#include <queue>

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
constexpr uint8_t APP_ID = 70;;
constexpr uint8_t IMAGE_TRANSCEIVER_ID = 24;

// Note that increasing quality increases data volume. (Transmission speed will slow down)
#if !defined JPEG_QUALITY
# define JPEG_QUALITY (80)
#endif
int jpeg_quality = JPEG_QUALITY;

MACAddress devices[] =
{
    MACAddress(DEVICE_C),
    MACAddress(DEVICE_A),
};
MACAddress target;
constexpr size_t trx_num = 1;
ImageTRX* trx_table[trx_num];
int current_trx = 0;

auto& lcd = M5.Display;
goblib::UnifiedButton unifiedButton;
MainClass decoder;

struct Image
{
    std::unique_ptr<uint8_t[]> ptr;
    size_t size;
};
std::queue<Image> imageQueue; // For receiver

enum Mode : uint8_t { Idle, Send, Recv, Failed };
Mode mode{Idle};

camera_config_t camera_config =
{
    .pin_pwdn     = -1,
    .pin_reset    = -1,
    .pin_xclk     = 2,
    .pin_sscb_sda = 12,
    .pin_sscb_scl = 11,
    .pin_d7 = 47,
    .pin_d6 = 48,
    .pin_d5 = 16,
    .pin_d4 = 15,
    .pin_d3 = 42,
    .pin_d2 = 41,
    .pin_d1 = 40,
    .pin_d0 = 39,
    .pin_vsync = 46,
    .pin_href  = 38,
    .pin_pclk  = 45,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 0,
    //x.fb_count     = 1,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    //.fb_location  = CAMERA_FB_IN_DRAM,
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1,
};
//
}

void trx_callback(ImageTRX* trx, const ImageTRX::Status s)
{
    M5_LOGD("finished:%u s:%d", trx->identifier(), (int)s);
    switch(s)
    {
    case ImageTRX::Send:
        trx->purge();
        break;
    case ImageTRX::Recv:
        imageQueue.emplace(Image{std::move(trx->uptr()), trx->size()});
        break;
    default: break;
    }
}

// This callback is called in the same task as Communicator::update()
void comm_callback(const Notify notify, const void* arg)
{
    mode = Failed;
    MACAddress addr;
    switch(notify)
    {
    case Notify::ConnectionLost:
        while(decoder.isBusy()) { delay(1); }
        addr = *(const MACAddress*)arg;
        lcd.clear(0);
        lcd.setCursor(0,0);
        lcd.printf("CONNECTION LOST\n%s", addr.toString(true).c_str());
        break;
    default: break;
    }
}

// Communicator task
void comm_task(void*)
{
    auto& comm = Communicator::instance();
    for(;;)
    {
        comm.update();
        delay(1);
    }
    // NOT REACHED
    //comm.end();
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

    uint8_t idx{};
    for(auto& trx : trx_table)
    {
        trx = new ImageTRX(IMAGE_TRANSCEIVER_ID + (idx++));
        assert(trx);
        trx->setFinishedCallback(trx_callback);
        comm.registerTransceiver(trx);
    }

    M5_LOGI("  Self: %s", comm.address().toString().c_str());
    for(auto& addr : devices)
    {
        if(!addr || addr == comm.address()) { continue; }
        M5_LOGI("Target: %s", addr.toString().c_str());
        comm.registerPeer(addr);
        target = addr;
    }
    comm.setRole(comm.address() == devices[0] ? Role::Primary : Role::Secondary);
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

    xTaskCreateUniversal(comm_task, "comm", 1024 * 8, nullptr, 2 /*priority */, nullptr, 1 /* core */);
}

void send_loop()
{
    static uint32_t scnt{}, sfps{}, ccnt{}, cfps{};
    static unsigned long tm = millis();

    auto fb = esp_camera_fb_get();
    
    if(fb)
    {
        size_t jlen{};
        uint8_t* jbuf{};
        auto trx = trx_table[current_trx];
        //while(trx->inProgress()) { delay(1); }
        
#if !defined(USING_UNRELIABLE)
        if(!trx->inProgress())
#endif
        {
            // RGB565 to JPEG
            // Note that increasing quality increases data volume.
            if(frame2jpg(fb, jpeg_quality, &jbuf, &jlen))
            {
                if(trx->send(target, jbuf, jlen))
                {
                    M5_LOGD("Send %d", current_trx);
                    current_trx = (current_trx + 1) % trx_num;
                    ++scnt;
                }
                else
                {
                    free(jbuf);
                    //M5_LOGE("skip ");
                }
            }
            else
            {
                M5_LOGE("Failed to frame2jpg");
            }
        }
#if !defined(USING_UNRELIABLE)
        else
        {
            M5_LOGD("Skip send %d", current_trx);
        }
#endif
        esp_camera_fb_return(fb);
        ++ccnt;
    }


    if(M5.BtnA.isPressed()) { --jpeg_quality; }
    if(M5.BtnC.isPressed()) { ++jpeg_quality; }
    // If the quality is too high, frame2jpg will generate an error, so suppress it.
    jpeg_quality = std::max(std::min(jpeg_quality, 80), 0); // [0...80]

    auto now = millis();
    if(now - tm >= 1000)
    {
        sfps = scnt;
        cfps = ccnt;
        tm = now;
        scnt = ccnt = 0;
    }
    lcd.setCursor(0,0);
    lcd.printf("HEAP:%u\n", esp_get_free_heap_size());
    lcd.printf("CFPS:%u SFPS:%i\n", cfps, sfps);
    lcd.printf("JPEG:%d", jpeg_quality);

    unifiedButton.draw();
}

void recv_loop()
{
    static uint32_t cnt{}, fps{};
    static unsigned long tm = millis();

    // Incoming data
    if(!imageQueue.empty())
    {
        Image img = std::move(imageQueue.front()); // queue has a unique_ptr.
        imageQueue.pop();
        ++cnt;

        //int64_t v{};
        {
            //goblib::esp_now::profile pf(v);
            decoder.drawJpg(img.ptr.get(), img.size); // Using Core0/1 for rendering
        }
        //M5_LOGI("decode:%lldms", v/1000);
    }
    auto now = millis();
    if(now - tm >= 1000)
    {
        if(cnt != fps)
        {
            fps = cnt;
            M5_LOGW("FPS:%u HEAP:%u", fps, esp_get_free_heap_size());
        }
        tm = now;
        cnt = 0;
    }
}


void idle_loop()
{
    // Receive image data?
    if(trx_table[0]->inProgress())
    {
        decoder.setup(&lcd);
        mode = Recv;
        return;
    }
    // Send image starts when any key is pressed on CoreS3.
    if(M5.getBoard() == m5::board_t::board_M5StackCoreS3 &&
       (M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed()))
    {
        M5.In_I2C.release();
        esp_err_t err = esp_camera_init(&camera_config);
        if(err != ESP_OK)
        {
            lcd.clear(TFT_BLUE);
            M5_LOGE("Failed to init camera:%d", err);
            while(true) { delay(1000); }
        }
        lcd.clear(TFT_ORANGE);
        mode = Send;
        return;
        //        
    }
    unifiedButton.draw();
}

void loop()
{
    M5.update();
    unifiedButton.update();

    if(M5.BtnPWR.wasClicked())
    {
        M5_LOGI("%s", Communicator::instance().debugInfo().c_str());
    }

    switch(mode)
    {
    case Send: send_loop(); break;
    case Recv: recv_loop(); break;
    case Failed: break;
    default:   idle_loop(); break;
    }
}
