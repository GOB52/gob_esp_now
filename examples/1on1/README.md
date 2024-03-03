# Example 1on1

[English](#overview)


## 概要
2台の M5Stack 間でボタンA/B/Cの状態を相互に送信するサンプルです。  
画面には受信した相手のボタン状態が表示されます。  
どちらかのデバイスのボタンが押されたら送受信が開始されます。

```
A)P:0R:1H:0WC:0WDC:0
B)P:0R:1H:0WC:0WDC:0
C)P:0R:1H:0WC:0WDC:0
```
|Character|Description|
|---|---|
|P|isPressed?|
|R|isReleased?|
|H|isHolding?|
|WC|wasClicked?|
|WDC|wasDoubleClicked?|

## MAC Addressの指定
platfromio.ini の修正が必要です。
```ini
[devices]
; Set your devices MAC address for examples
build_flags = 
  -DDEVICE_A="\"30:c6:f7:1d:1c:cc\""
  -DDEVICE_B="\"a4:cf:12:6d:87:1c\""
```
それぞれに自分のM5Stackデバイスのアドレスを記入してください。

### MAC Address の調べ方
esptool.py がインストールされていれば以下のコマンドで知ることができます。(M5Stack を接続していること)

```sh
esptool.py read_mac
または
esptool.py -p PORT read_mac
```

出力例
```
esptool.py -p /dev/cu.SLAB_USBtoUART read_mac

esptool.py v4.7.0
Serial port /dev/cu.SLAB_USBtoUART
Connecting....
Detecting chip type... Unsupported detection protocol, switching and trying again...
Connecting.....
Detecting chip type... ESP32
Chip is ESP32-D0WDQ6-V3 (revision v3.0)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: 30:c6:f7:1d:1c:cc  <<<<<<<<<<<<<<<<<<<<<<<<<< ここ!
Uploading stub...
Running stub...
Stub running...
MAC: 30:c6:f7:1d:1c:cc
Hard resetting via RTS pin...
```

---

## Overview

This is a sample of transmission of button A/B/C status between two M5Stack devices.  
The screen displays the button status of the receiving device.  
When the button on either device is pressed, transmission and reception begin.

```
A)P:0R:1H:0WC:0WDC:0
B)P:0R:1H:0WC:0WDC:0
C)P:0R:1H:0WC:0WDC:0
```
|Character|Description|
|---|---|
|P|isPressed?|
|R|isReleased?|
|H|isHolding?|
|WC|wasClicked?|
|WDC|wasDoubleClicked?|

## MAC Address settings

Need modify platfromio.ini
```ini
[devices]
; Set your devices MAC address for examples
build_flags = 
  -DDEVICE_A="\"30:c6:f7:1d:1c:cc\""
  -DDEVICE_B="\"a4:cf:12:6d:87:1c\""
```
Fill in the address of your M5Stack device for each.


## How to detect MAC Address
If AAA is already installed, it can be detected with the following command. (M5Stack must be connected)

```sh
esptool.py read_mac
or
esptool.py -p PORT read_mac
```

e.g.
```
esptool.py -p /dev/cu.SLAB_USBtoUART read_mac

esptool.py v4.7.0
Serial port /dev/cu.SLAB_USBtoUART
Connecting....
Detecting chip type... Unsupported detection protocol, switching and trying again...
Connecting.....
Detecting chip type... ESP32
Chip is ESP32-D0WDQ6-V3 (revision v3.0)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: 30:c6:f7:1d:1c:cc  <<<<<<<<<<<<<<<<<<<<<<<<<< HERE!
Uploading stub...
Running stub...
Stub running...
MAC: 30:c6:f7:1d:1c:cc
Hard resetting via RTS pin...
```
