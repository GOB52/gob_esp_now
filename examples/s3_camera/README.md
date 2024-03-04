# Example s3_camera

[日本語](#概要)

## Overview
Sample of sending and displaying captured video from a CoreS3 camera to another M5Stack.  
The sending side must be CoreS3.  
Start by pressing any button on the CoreS3 side.

## MAC Address settings

Need modify platfromio.ini
```ini
[devices]
; Set your devices MAC address for examples
build_flags = 
  -DDEVICE_A="\"30:c6:f7:1d:1c:cc\""
  -DDEVICE_C="\"a4:cf:12:6d:87:1c\""
```
Fill in the address of your M5Stack device for each.  
M5Stacks with the addresses specified in DEVICE\_A and DEVICE\_C communicate with each other.

## How to detect MAC Address
If esptool.py is already installed, it can be detected with the following command. (M5Stack must be connected)

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

## Remarks
JPEG decoding uses the same decoding used by [M5Stack_FlipBookSD](https://github.com/GOB52/M5Stack_FlipBookSD).


---

## 概要
CoreS3 のカメラのキャプチャ映像を他のM5Stackへ送信し、表示するサンプルです。  
送信側は CoreS3 である必要があります。  
CoreS3 側の任意のボタン押下で開始します。

## MAC Addressの指定
platfromio.ini の修正が必要です。
```ini
[devices]
; Set your devices MAC address for examples
build_flags = 
  -DDEVICE_A="\"30:c6:f7:1d:1c:cc\""
  -DDEVICE_C="\"a4:cf:12:6d:87:1c\""
```
それぞれに自分の M5Stack デバイスのアドレスを記入してください。  
DEVICE\_A と DEVICE\_C で指定されたアドレスを持つ M5Stack 同士で通信を行います。

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

## 備考

JPEG のデコードには [M5Stack_FlipBookSD](https://github.com/GOB52/M5Stack_FlipBookSD) で使用している物を使用しています。
