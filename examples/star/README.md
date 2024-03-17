# Example star

<img src="https://github.com/GOB52/gob_esp_now/assets/26270227/048e7760-5086-4d16-8582-7f000dbd398f" width="25%" />


[日本語](#概要)

## Overview
This is a sample of a star connection between two or more M5Stacks.  
Clicking a button on the host M5Stack notifies other devices via broadcast that a connection is available,
The notified device attempts to connect to the host.  
Since this is a star connection, the host is connected to all other devices, and the other devices are connected only to the host.  

Once connected, the connection status and the button status of the connected device are displayed as shown in the image.


Translated with DeepL.com (free version)

---

## 概要
2台以上の M5Stack 間を星形接続するサンプルです。  
ホストとなる M5Stack のボタンをクリックすると、他のデバイスにブロードキャストで接続可能なことを通知し、
通知を受けたデバイスはホストに対して接続を試みます。  
星形接続なので、ホストは他の全てのデバイスと接続、他のデバイスはホストとのみ接続となります。  

接続されると画像のように、接続状況と接続先のボタン状態が表示されます。

