# M5StickS3 Mic

M5StickS3の内蔵マイクと外部I2Sマイクを使用して音声データを収集します。
Arduinoで実装し、BLEでデータを送信します。

## マイク

- 内蔵マイク: ES8311経由のI2S MEMSマイク
- 外部マイク: INMP441
- サンプリング周波数: 8kHz
- 音声データ: 16bit PCM

内蔵ES8311のI2Sピンは、使用するM5StickS3の基板リビジョンとM5Unifiedの
バージョンで差異があるため、スケッチではインストール済みM5Unifiedの
ボード設定をそのまま使用します。現在のM5Unified設定は以下です。

| ES8311 | ESP32-S3 |
| --- | --- |
| MCLK | GPIO18 |
| DOUT（マイクデータ） | GPIO14 |
| BCLK | GPIO17 |
| LRCK/WS | GPIO15 |
| DIN（再生データ） | GPIO16 |
| SCL | GPIO48 |
| SDA | GPIO47 |

なお、現在インストールされているM5Unifiedの`board_M5StickS3`設定は
マイクのI2S DATA INをGPIO16としているため、スケッチの内蔵マイクは
M5Unifiedの設定を優先しています。基板リビジョンに合わせてGPIO14を使う場合は、
M5Unifiedの更新版または対応するボード設定が必要です。

## INMP441の配線

M5StickS3のHat2-BusにあるGPIO5、GPIO6、GPIO7を外部I2S用に使用します。

| INMP441 | M5StickS3 |
| --- | --- |
| SCK/BCLK | GPIO5 |
| WS/LRCK | GPIO6 |
| SD/DATA | GPIO7 |
| L/R | GND |
| VDD | 3.3V |
| GND | GND |

L/RをGNDに接続すると左チャンネルになります。スケッチでは
`m5::input_only_left`を設定しています。

GPIOを変更する場合は、`M5StickS3Mic.ino`先頭の以下の定数を変更してください。

```cpp
constexpr int EXTERNAL_I2S_BCLK_PIN = 5;
constexpr int EXTERNAL_I2S_WS_PIN = 6;
constexpr int EXTERNAL_I2S_DATA_PIN = 7;
```

## 操作

- Key1（BtnA）: データ収集の開始/停止
- Key2（BtnB）: ES8311内蔵マイク / INMP441外部マイクの切替
- 画面: マイク種別、測定状態、音声波形を表示

## BLE出力

通知データは以下の形式です。

```text
millis,Out
```

BLEはNordic UART Service形式です。

- RX（書き込み）: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- TX（通知/読み取り）: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

音声データを受信するクライアントは、TX characteristicの通知を購読してください。
通知1回あたり最大10行をまとめて送信します。各行は改行で区切られ、
受信側ではカンマ区切りの1列目を`millis`、2列目を`Out`として扱ってください。

Arduinoスケッチは [M5StickS3Mic.ino](M5StickS3Mic.ino) です。