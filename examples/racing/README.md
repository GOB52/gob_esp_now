# Example racing

[日本語](#概要)

## Overview
This is a sample P2P racing game that can be played on two M5Stacks.  
It works only with M5Stack Core and Core2 (CoreS3 has no buttons).  
Unlike the other samples, the MAC address does not need to be registered in platformio.ini.

Original is M5Stack_CircuitRacer.ino by @Lovayn03 -san.  
I extracted and organized the M5Stack-specific parts from the original and modified it to enable P2P competition.

## How to play

1. press B button on the M5Stack side that will be the host side
1. communicate with the other side, and when the connection is established, the course selection is made. (Decied by B Button)
1. After selecting a course, each player moves to the state of preparation for the start of the game. 
1. After the countdown, the game starts.


### Operation

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

## 遊び方

1. ホスト側になる M5Stack 側の B ボタンを押下
1. 相手側と通信をし、接続状態になるとコース選択に移行 (B ボタンで決定)
1. コースを選択すると、それぞれが開始準備状態へ移行
1. カウントダウン後、ゲーム開始。

### 操作

|Buttton|Description|
|---|---|
|A|ハンドルを左へ|
|C|~~インド人を右に!~~ ハンドルを右へ|

(アクセルは自動で踏みっぱなし)

