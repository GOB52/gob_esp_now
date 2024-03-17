# Example racing

<img src="https://github.com/GOB52/gob_esp_now/assets/26270227/6236f83c-64e5-47cd-bbff-a705ff0740c5" width="33%" /><img src="https://github.com/GOB52/gob_esp_now/assets/26270227/0105238f-c99c-47b3-99ce-0f9790019470" width="33%" /><img src="https://github.com/GOB52/gob_esp_now/assets/26270227/c84d2d8a-14b5-433d-8dac-330209cfee64" width="33%" />


[日本語](#概要)

## Overview
This is a sample P2P racing game that can be played on two M5Stacks.  
It works only with M5Stack Core and Core2 (CoreS3 has no buttons).  
Unlike the other samples, the MAC address does not need to be registered in platformio.ini.

Original is M5Stack_CircuitRacer.ino by @Lovayn03 -san.  
I extracted and organized the M5Stack-specific parts from the original and modified it to enable P2P competition.

## WiFi
WiFi connection is made to adjust the time by NTP. (to adjust the game start timing)  
Save the credential information in advance or set the SSID and password to arguments of WiFi.begin().

* racing_main.cpp
```cpp
void configTime()
{
    // WiFi connect
    //WiFi.begin(); // Connect to credential in Hardware. (ESP32 saves the last WiFi connection)
    WiFi.begin("SSID", "PASS");
```

## How to play

1. press B button on the M5Stack side that will be the host side
1. communicate with the other side, and when the connection is established, the course selection is made.  
Return to the beginning in case of timeout
1. After selecting a course, each player moves to the state of preparation for the start of the game. 
1. After the countdown, the game starts.  
The opponent's car is indicated by a shadow of a black circle.

### Operation

### In select stage
|Buttton|Description|
|---|---|
|A C|Change stage|
|B|Decide|


#### In game
|Buttton|Description|
|---|---|
|A|Steering to left|
|C|Steering to right|

(The accelerator pedal is automatically kept pressed)

---

## 概要
二台の M5Stack で遊べる P2P レーシングゲームのサンプルです。  
M5Stack Core,Core2 のみでの動作となります(CoreS3 はボタンがないので)  
他のサンプルと違い MAC Address の platformio.ini への登録は必要あのません。

オリジナルは @Lovayn03 氏の M5Stack_CircuitRacer.ino です。  
M5Stack 固有部分を抽出して整理し、P2P対戦ができるように改造しました。

## WiFi
NTP による時刻合わせの為に WiFi 接続を行います。(ゲーム開始タイミングを合わせる為)  
事前にクレデンシャル情報を保存しておくか、ソースWiFi.begin()でSSIDとパスワードを設定してください。

* racing_main.cpp
```cpp
void configTime()
{
    // WiFi connect
    //WiFi.begin(); // Connect to credential in Hardware. (ESP32 saves the last WiFi connection)
    WiFi.begin("SSID", "PASS");
```

## 遊び方

1. ホスト側になる M5Stack 側の B ボタンを押下
1. 相手側と通信をし、接続状態になるとコース選択に移行  
タイムアウトした場合は最初に戻る
1. コースを選択すると、それぞれが開始準備状態へ移行
1. カウントダウン後、ゲーム開始。  
相手の車は黒丸の影で表示されます。

### 操作

### コース選択中
|Buttton|Description|
|---|---|
|A C|コース変更|
|B|決定|

#### ゲーム中
|Buttton|Description|
|---|---|
|A|ハンドルを左へ|
|C|~~インド人を右に!~~ ハンドルを右へ|

(アクセルは自動で踏みっぱなし)

