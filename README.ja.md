# gob_esp_now

Not yet published. Work in progress...

## 概要
[ESP-NOW](https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/espnow.html) をラップし、 [RUDP](https://datatracker.ietf.org/doc/html/draft-ietf-sigtran-reliable-udp-00) による通信順序と信頼性を確保したライブラリです。  
ESP-NOW の低層では信頼性確保の為の処理が行われているようですが、残念ながら完璧ではありません。  
当ライブラリは RUDP のように再送やハートビートを行うことで、信頼性の向上と状態検知が可能です。  
(なお全ての RUDP 機能は現時点でははサポートされていません)

## 導入
環境によって適切な方法でインストールしてください
* git clone や Zip ダウンロードからの展開
* platformio.ini
```ini
lib_deps = https://github.com/GOB52/gob_esp_now.git
```

## 構成

シングルトン(ひとつのインスタンスしか持たない)として存在するコミュニケータと、それにぶら下がるいくつかのトランシーバの形で送受信の管理が行われます。  
それぞれにはアプリケーション側でユニークな ID を設定します。2種の ID が合致するもの同士でのみ送受信が確立するようになっています。これによっていちいち受信パケットを解析して処理の振り分けをせず、ペイロードのデータのみを注視することができます。  
コミニュケータとトランシーバの ID の組み合わせは本物のトランシーバの周波数帯やチャンネルに相当するものです。


### 例

```cpp
#include <M5Unified.h>
#include <WiFi.h>
#include <gob_esp_now.hpp>
using namespace goblib::esp_now;
// Create your own TRX class for arbitrary send/receive
// In this example, an arbitrary string is assumed to be sent and received.
class MyTRX : public Transceiver
{
  public:
    explicit MyTRX(const uint8_t id) : Transceiver(id) {}
    MyTRX() = delete; // Delete default constructor

  protected:
    // Data received.
    virtual void onReceive(const MACAddress& addr, // Sender MAC address
                           const void* data,       // payload pointer
                           const uint8_t length) override // payload size
    {
        const char* str = (const char*)data;
        M5_LOGI("Recv:[%s]", str); // Log output of received string
    }
};
MACAddress target("11:22:33:44:55:66"); // Must be set destination address
auto& comm = Communicator::instance(); // Communicator instance (Singleton)
MyTRX trx(123 /* Any ID for TRX in the range 1-255 */);

void  setup()
{
    M5.begin();
    // Set mode and channels as appropriate for your environment.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    comm.addPeer(target); // Must be added destination MAC address.
    comm.registerTransceiver(&trx); // Must be registered your TRX.
    auto ret = comm.begin(52 /* Any ID for communicator in the range 1-255 */); // Initialize ESP-NOW and begin send/receive tasks.
    M5.Display.clear(ret ? TFT_DARKGREEN : TFT_RED); // If the screen is red, initialization failed.
}

void loop()
{
    M5.update();
    if(M5.BtnA.wasClicked()) 
    {
        char buf[128]{};
        snprintf(buf, sizeof(buf), "TEST string from <%s>", comm.address().toString().c_str());
        trx.postReliable(target, buf, strlen(buf) + 1 /* include terminate character */);  // Send string if BtnA was clicked.
        // When the communication arrives, it goes to the target device MyTRX::onReceive. 
    }
    
}
```





#### TODO
- Reliable / Unreliable
- c - > t,t,t    esp-now ttt-c の図
- 250 制限
- 詰まり解消のための task 制御 (config)
- パケット仕様
- RUDP 機能範囲


