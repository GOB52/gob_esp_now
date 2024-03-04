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
    //MACAddress(DEVICE_B),
    MACAddress(DEVICE_C),
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
        if(!fname[0] || fname[0] == '.') { continue; }

        if(f.isDirectory())
        {
            s = formatString("%*c%s/", indent*2, ' ', fname);
            M5_LOGI("%s", s.c_str());
            ls(f, indent + 1);
            continue;
        }
        s = formatString("%*c%s : %lu", indent*2, ' ', fname, f.size());
        M5_LOGI("%s",s.c_str());
    }
    while(true);
}

void ls(const char* dir)
{
    File f = sd.open(dir);
    ls(f);
}

void dump(const uint8_t* buf, const size_t len)
{
    for(size_t i = 0; i < len; ++i)
    {
        M5.Log.printf("%02x%c", buf[i], ((i + 1) % 16) ? ' ' : '\n');
    }
    M5.Log.println();
}

// Prepare those files in SD
/*
 Make by terminal command
 cd tmp
 head -c 1250000 /dev/urandom > dummy_10Mbps.txt
          [size]                [filename]
*/
constexpr const char dir[] = "/tmp";
const char* files[] =
{
    "dummy_1Mbps.txt",   // 125000
    "dummy_512K.txt",    // 524288
    "dummy_1M.txt",      // 1048576
    "dummy_10Mbps.txt",  // 1250000
    "dummy_10M.txt",     // 10485760
};
int file_index{};
uint32_t file_crc{};

TransferTRX::Status  old_state{TransferTRX::Status::None};
bool failed{}, dirty{};

const char* status_string(const TransferTRX::Status s)
{
    static const char* tbl[] = { "None", "Send", "Recv" };
    return tbl[(uint8_t)s];
}
//
}

// Error callback
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
            lcd.printf("CONNECTION LOST\n%s", addr.toString(true).c_str());
        });
        break;
    default: break;
    }
    transfer.abort();
}

void disp()
{
    lcd.startWrite();
    if(!failed && (transfer.inProgress() || transfer.finished()))
    {
        auto cur_state = transfer.status();
        lcd.setCursor(0,0);
        lcd.printf("%s:%u\n", status_string(cur_state), esp_get_free_heap_size());
        lcd.printf("%s\n", transfer.fname());
        lcd.printf("%lu / %lu\n", transfer.transferedSize(), transfer.fileSize());
        auto rate = transfer.transferedRate();
        lcd.printf("%3.3f %%\n", rate * 100.f);
        auto pt = transfer.finished() ? transfer.timeRequired() : millis() - transfer.startTime();
        auto s = pt/1000;
        auto m = s/60;
        auto h = m/60;
        lcd.printf("%02ld:%02ld:%02ld:%03ld\n", h, m % 60, s % 60, pt % 1000);
        lcd.fillRect(0, lcd.height() - 48, lcd.width() * rate, 32, TFT_BLUE);
        lcd.printf("%lu\n", transfer.averageSpeed());
        lcd.printf("%s:%x", file_crc ? "FILE CRC" : "CRC", file_crc ? file_crc : transfer.crc32());
    }
    lcd.endWrite();
}

// Drawing interval every about 1 second
void disp_task(void*)
{
    for(;;)
    {
        // Exclusive control of card access and drawing
        // Because the card and Lcd buses are shared.
        transfer.bus_lock(disp);

        int wait = 1000; // 1 Sec
        while(wait--)
        {
            delay(1);
            if(dirty) { break; }
        }
        dirty = false;
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
    
    unifiedButton.begin(&lcd, goblib::UnifiedButton::appearance_t::transparent_all);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    lcd.setFont(&fonts::Font4);

    //
    auto before = esp_get_free_heap_size();
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
    comm.setRole(comm.address() == devices[0] ? Role::Primary : Role::Secondary);
    comm.registerNotifyCallback(comm_callback);
    
    auto cfg = comm.config();
    cfg.update_priority = 2;
    comm.begin(APP_ID, cfg);

    auto after = esp_get_free_heap_size();
    M5_LOGI("Library usage:%u : %uK", before - after, (before - after) / 1024);
    
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    M5_LOGI("%s", comm.debugInfo().c_str());

    xTaskCreateUniversal(disp_task, "disp", 1024 * 8, nullptr, 1, nullptr, 0);

    M5_LOGI("Heap at end of setup:%u", esp_get_free_heap_size());
    lcd.clear(TFT_DARKGREEN);
}

void loop()
{
    auto& comm = Communicator::instance();

    M5.update();
    unifiedButton.update();
    if(!transfer.inProgress() && M5.BtnA.wasClicked())
    {
        String path = formatString("%s/%s", dir, files[file_index]);
        file_index = (file_index + 1) % (sizeof(files)/sizeof(files[0]));
        failed = !transfer.send(target, path);
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

    auto cur_state = transfer.status();
    if(old_state != cur_state)
    {
        switch(cur_state)
        {
        case TransferTRX::Status::Recv:
            file_crc = 0;
            transfer.bus_lock([]()
            {
                lcd.startWrite();
                lcd.clear(TFT_CYAN);
                lcd.fillRect(0, lcd.height() - 48, lcd.width(), 32, TFT_WHITE);
                lcd.endWrite();
                dirty = true;
            });
            break;
        case TransferTRX::Status::Send:
            file_crc = 0;
            transfer.bus_lock([]()
            {
                lcd.startWrite();
                lcd.clear(TFT_ORANGE);
                lcd.fillRect(0, lcd.height() - 48, lcd.width(), 32, TFT_WHITE);
                lcd.endWrite();
                dirty = true;
            });
            break;
        default: break;
        }
        if(old_state == TransferTRX::Status::Recv && transfer.finished())
        {
            transfer.bus_lock([]()
            {
                file_crc = calculateCRC32(transfer.path());
                M5_LOGI("Received file [%s] CRC:%x", transfer.path(), file_crc);
            });
            dirty = true;
        }
        old_state = cur_state;
    }
}
