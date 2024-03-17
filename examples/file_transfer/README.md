# Example file_transfer

<img src="https://github.com/GOB52/gob_esp_now/assets/26270227/ea78d6d8-5899-411c-9be8-4dd29bcaaa61" width="25%" />

[日本語](#概要)

## Overview

This sample transfers files on an SD card between two M5STack devices.  
Press the A button on the source M5Stack (for CoreS3, press the left 1/3 button on the screen) to start transfer.  
The transfer speed is about 50000 bytes/sec, depending on the communication status.  
The transferred file will be saved in the root of the receiver device's SD card.

## Prepare files
Prepare the following files in /tmp of each SD card.  
The size of the file can be arbitrary, but it is better not to make it too large (because it will take a long time to transfer the file).  
I used the following command to create the file.

```sh
head -c 1250000 /dev/urandom > /tmp/dummy_10Mbps.txt
         [size]                 [filename]
```
|Filename|Size|
|---|---|
|dummy\_1Mbps.txt  |   125000|
|dummy\_512K.txt   |   524288|
|dummy\_1M.txt     |  1048576|
|dummy\_10Mbps.txt |  1250000|
|dummy\_10M.txt    | 10485760|

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
M5Stacks with the addresses specified in DEVICE\_A and DEVICE\_B communicate with each other.

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

---

## 概要
二台の M5Stack 間で、 SD カード内のファイルを転送するサンプルです。  
転送元の M5Stack の A ボタン押下(CoreS3 の場合は画面左 1/3 押下) で転送を開始します。  
通信状態にもよりますが、50000 bytes/sec 程度の転送速度です。  
転送されたファイルは、相手の SD カードのルートに保存されます。

## ファイルの準備
それぞれの SD カードの /tmp に以下のファイルを用意します。  
ファイルサイズは適当でも構いませんが、あまり大きくしない方が良いでしょう(転送時間がかかるため)  
私は以下のコマンドにてファイルを作成しました。

```sh
head -c 1250000 /dev/urandom > /tmp/dummy_10Mbps.txt
         [size]                 [filename]
```
|Filename|Size|
|---|---|
|dummy\_1Mbps.txt  |   125000|
|dummy\_512K.txt   |   524288|
|dummy\_1M.txt     |  1048576|
|dummy\_10Mbps.txt |  1250000|
|dummy\_10M.txt    | 10485760|

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
DEVICE\_A と DEVICE\_B で指定されたアドレスを持つ M5Stack 同士で通信を行います。

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
