/*
  gob_esp_now example: file_transfer
  Transfer SD card files between 2 devices
*/
#include <SdFat.h>
#include <M5Unified.h>
#include <gob_esp_now.hpp>
#include <gob_unifiedButton.hpp> // for CoreS3
#include <WiFi.h>
#include <esp_wifi.h>
#include "transferTRX.hpp"

using namespace goblib::esp_now;

#if !defined(DEVICE_A)
#error Need define MACAddress to DEVICE_A
#endif
#if !defined(DEVICE_B)
#error Need define MACAddress to DEVICE_B
#endif

#ifndef TFCARD_CS_PIN
# define TFCARD_CS_PIN (4)
#endif

using File = FsFile;

SdFs sd;

namespace
{
auto& lcd = M5.Display;
goblib::UnifiedButton unifiedButton;

constexpr uint8_t APP_ID = 47;;
constexpr uint8_t TRANSCEIVER_ID = 47;

MACAddress devices[] =
{
    MACAddress(DEVICE_A),
    MACAddress(DEVICE_B),
};
MACAddress target;
TransferTRX transfer(TRANSCEIVER_ID);

template<typename ...Args> String formatString(const char* fmt, Args... args)
{
    size_t sz = snprintf(nullptr, 0U, fmt, args...); // calculate length
    char buf[sz + 1];
    snprintf(buf, sizeof(buf), fmt, args...);
    return String(buf, sz);
}

void ls(File dir, uint8_t indent = 0)
{
    String s;
    char fname[256];
    do
    {
        File f = dir.openNextFile();
        if(!f) { dir.rewindDirectory(); break; }

        f.getName(fname, sizeof(fname));
        //if(f.name()[0] == '.') { continue; }
        if(fname[0] == '.') { continue; }

        if(f.isDirectory())
        {
            //            s = formatString("%*c%s/", indent*2, ' ', f.name());
            s = formatString("%*c%s/", indent*2, ' ', fname);
            M5_LOGI("%s", s.c_str());
            ls(f, indent + 1);
            continue;
        }
        //        s = formatString("%*c%s : %lu", indent*2, ' ', f.name(), f.size());
        s = formatString("%*c%s : %lu", indent*2, ' ', fname, f.size());
        M5_LOGI("%s",s.c_str());
    }
    while(true);
}

//String filePath;
//String filePath = "/gmv/Sintel.gmv";
String filePath = "/wav/lupin_2_8k_16.wav";
//String filePath = "/bmp/item_003.bmp";
//String filePath = "/jpg/iyami.jpg";
//String filePath = "/png/pk16_2.png";


void find_largest_file(File dir, String dname = "", bool init = true)
{
    static file_size_t fileSize{};
    char fname[256];

    if(init) { filePath = ""; fileSize = 0; }
    do
    {
        File f = dir.openNextFile();
        if(!f) { dir.rewindDirectory(); break; }
        f.getName(fname, sizeof(fname));
        //if(f.name()[0] == '.') { continue; }
        if(fname[0] == '.') { continue; }

        if(f.isDirectory())
        {
            String dn = dname + fname;
            dn += '/';
            find_largest_file(f, dn, false);
            continue;
        }
        if(dname.length() + strlen(fname) < 128 && f.size() > fileSize) // Reject long pathname
        {
            fileSize = f.size();
            filePath = formatString("/%s%s", dname.c_str(), f.name());
        }
    }
    while(true);
}

TransferTRX::Status  old_state{TransferTRX::Status::None};
bool dirty{}, failed{};
//
}


void transfer_callback(void*)
{

}

void comm_callback(const Notify notify, const void* arg)
{
    MACAddress addr;
    switch(notify)
    {
    case Notify::ConnectionLost:
        addr = *(const MACAddress*)arg;
        failed = true;
        transfer.bus_lock([&addr]()
        {
            lcd.clear(0);
            lcd.setCursor(0,0);
            lcd.printf("CONNECTION LOST\n%s", addr.c_str(true));
        });
        break;
    default: break;
    }
    transfer.abort();
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
    comm.end();
}

// Drawing interval every second
void disp()
{
    lcd.startWrite();

    auto cur_state = transfer.status();
    if(old_state != cur_state)
    {
        switch(cur_state)
        {
            //case TransferTRX::Status::None: lcd.clear(TFT_DARKGREEN); break;
        case TransferTRX::Status::Recv:
            lcd.clear(TFT_CYAN);
            lcd.fillRect(0, lcd.height() - 64, lcd.width(), 32, TFT_WHITE);
            filePath = transfer.fname();
            break;
        case TransferTRX::Status::Send:
            lcd.clear(TFT_ORANGE);
            lcd.fillRect(0, lcd.height() - 64, lcd.width(), 32, TFT_WHITE);
            break;
        default: break;
        }
        old_state = cur_state;
    }
        
    if(!failed && (transfer.inProgress() || transfer.finished()))
    {
        String fname = filePath.substring(filePath.lastIndexOf('/') + 1);
        lcd.setCursor(0,0);
        lcd.printf("%s\n", fname.c_str());
        lcd.printf("%lu / %lu\n", transfer.transferedSize(), transfer.fileSize());
        auto rate = transfer.transferedRate();
        lcd.printf("%3.3f %%\n", rate * 100.f);
        auto pt = transfer.finished() ? transfer.timeRequired() : millis() - transfer.startTime();
        auto s = pt/1000;
        auto m = s/60;
        auto h = m/60;
        lcd.printf("%02ld:%02ld:%02ld\n", h, m % 60, s % 60);
        lcd.fillRect(0, lcd.height() - 64, lcd.width() * rate, 32, TFT_BLUE);
        lcd.printf("%lu\n", transfer.averageSpeed());
    }

    lcd.endWrite();
}

void disp_task(void*)
{
    for(;;)
    {
        // Exclusive control of card access and drawing
        // Because the card and Lcd buses are shared.
        transfer.bus_lock(disp);
        delay(1000);
    }
}

void setup()
{
    esp_log_level_set("*", ESP_LOG_DEBUG);

    M5_LOGI("Heap:%u", esp_get_free_heap_size());

    M5.begin();
    
    int retry = 10;
    bool mounted{};
    while(retry-- && !(mounted = sd.begin((unsigned)TFCARD_CS_PIN, SD_SCK_MHZ(25))) ) { delay(100); }
    if(!mounted) { M5_LOGE("Failed to mount %x", sd.sdErrorCode()); lcd.clear(TFT_RED); while(1) { delay(10000); }}
    
    unifiedButton.begin(&lcd);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);

    lcd.setFont(&fonts::Font4);
    unifiedButton.setFont(&fonts::Font4);

    // Find the largest file => trasfer target
    lcd.clear(TFT_DARKGRAY);
    lcd.printf("Please wait, Searching file...");

    //ls(sd.open("/"));
    //    find_largest_file(sd.open("/"));
    if(filePath.length())
    {
        M5_LOGI("%s", filePath.c_str());
        lcd.clear(TFT_DARKGREEN);
    }

    //
    auto& comm = Communicator::instance();

    M5_LOGI("  Self: %s", comm.address().toString().c_str());
    for(auto& addr : devices)
    {
        if(!addr || addr == comm.address()) { continue; }
        M5_LOGI("Target: %s", addr.toString().c_str());
        if(!comm.registerPeer(addr))
        {
            M5_LOGE("Failed to register target peer");
            lcd.clear(TFT_RED);
            for(;;) { delay(1000); }
        }
        target = addr;
    }
    comm.registerTransceiver(&transfer);
    comm.setRole(comm.address() == devices[0] ? Communicator::Role::Primary : Communicator::Role::Secondary);

    comm.registerNotifyCallback(comm_callback);
    
    auto cfg = comm.config();
    cfg.retransmissionTimeout = 200;
    cfg.retransmissionTimeout = 1000;
    //cfg.cumulativeAckTimeout = 100;
    cfg.maxRetrans = 4;
    //cfg.nullSegmentTimeout = 5000;
    cfg.nullSegmentTimeout = 0;
    comm.begin(APP_ID, cfg);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    M5_LOGI("%s", comm.debugInfo().c_str());
    
    //
    //    xTaskCreateUniversal(comm_task, "comm", 1024 * 8, nullptr, 1 /*priority */, nullptr, 0 /* core */);
    //    xTaskCreateUniversal(disp_task, "disp", 1024 * 8, nullptr, 1, nullptr, 1);
    //xTaskCreateUniversal(comm_task, "comm", 1024 * 8, nullptr, 1 /*priority */, nullptr, 1 /* core */);
    //xTaskCreateUniversal(disp_task, "disp", 1024 * 8, nullptr, 0, nullptr, 0);
    xTaskCreateUniversal(comm_task, "comm", 1024 * 12, nullptr, 2 /*priority */, nullptr, 1 /* core */);
    xTaskCreateUniversal(disp_task, "disp", 1024 * 12, nullptr, 1, nullptr, 0);

    M5_LOGI("Heap as end of setup:%u", esp_get_free_heap_size());
}

void loop()
{
    auto& comm = Communicator::instance();

    M5.update();
    if(filePath.length() && !transfer.inProgress() &&  M5.BtnA.wasClicked())
    {
        //UBaseType_t myPriority = uxTaskPriorityGet(NULL);
        //M5_LOGI("loop priority %d", myPriority); // 1
        dirty = true;
        failed = !transfer.send(target, filePath.c_str());
        if(failed)
        {
            M5_LOGE("Failed to send");
            return;
        }
    }
    if(M5.BtnC.wasClicked())
    {

        M5_LOGE("%s", comm.debugInfo().c_str());
    }
    if(M5.BtnB.wasClicked())
    {
        M5_LOGI("Heap:%u", esp_get_free_heap_size());
        transfer.bus_lock([&comm]()
        {
            lcd.startWrite();
            //bool b = lcd.drawBmpFile(sd, comm.isPrimary() ? filePath.c_str() :"/aaa.aaa");
            //bool b = lcd.drawJpgFile(sd, comm.isPrimary() ? filePath.c_str() :"/aaa.aaa");
            bool b = lcd.drawPngFile(sd, comm.isPrimary() ? filePath.c_str() :"/aaa.aaa");
            if(!b) { M5_LOGE("Failed to draw"); }
            lcd.endWrite();
            delay(3000);
            
        });
    }

#if 0
        if(!transfer.inProgress() &&  M5.BtnC.wasClicked())
    {
        ls(sd.open("/"));
    }
#endif
    if(dirty)
    {
        dirty = false;
        transfer.bus_lock(disp);
    }
}
