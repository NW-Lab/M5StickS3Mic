#include <M5Unified.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>

// ------------------------------------------------------------
// M5StickS3: microphone selection and recording sketch
// ------------------------------------------------------------
// README の要件を反映した、Arduino の雛形です。
// - 長押しで内蔵マイク / 外部 I2S マイク切替
// - ボタン短押しで収集開始 / 停止
// - 画面に状態表示、波形表示
// - 44.1kHz を想定したサンプル取得
// - BLE で "millis,Out" 形式のデータを送信
// ------------------------------------------------------------

constexpr uint32_t SAMPLE_RATE = 44100;
constexpr size_t MIC_BUFFER_SAMPLES = 128;
constexpr int WAVEFORM_DISPLAY_GAIN = 2;  //8
constexpr size_t WAVEFORM_DISPLAY_DECIMATION = 8;
constexpr size_t BLE_BATCH_LINES = 25;
constexpr size_t BLE_BATCH_BUFFER_SIZE = 768;

// M5StickS3 Hat2-BusのGPIOを使用します。
// INMP441: SCK -> BCLK, WS -> LRCK, SD -> DATA IN, L/R -> GND(左チャンネル)
constexpr int EXTERNAL_I2S_BCLK_PIN = 5;
constexpr int EXTERNAL_I2S_WS_PIN = 6;
constexpr int EXTERNAL_I2S_DATA_PIN = 7;

const char* DEVICE_NAME = "M5StickS3Mic";
constexpr char NUS_SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char NUS_RX_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char NUS_TX_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

enum class MicSource : uint8_t {
  Internal = 0,
  External = 1,
};

MicSource activeMic = MicSource::Internal;
bool isRecording = false;
volatile bool bleConnected = false;
volatile bool bleDisplayUpdatePending = false;
const char* statusText = "READY";
int16_t micBufferA[MIC_BUFFER_SAMPLES];
int16_t micBufferB[MIC_BUFFER_SAMPLES];
volatile bool micBufferPending = false;
volatile int16_t latestMicSample = 0;
volatile uint32_t micBufferCount = 0;
volatile int16_t* releasedBuffers[3] = { nullptr, nullptr, nullptr };
volatile size_t releasedLengths[3] = { 0, 0, 0 };
volatile uint8_t releasedHead = 0;
volatile uint8_t releasedTail = 0;
portMUX_TYPE releasedMux = portMUX_INITIALIZER_UNLOCKED;
bool audioDataSeen = false;
uint32_t nextQueueAttemptMs = 0;
uint32_t lastRecordFailureMs = 0;
uint32_t lastSampleAt = 0;
uint32_t lastBleTxMs = 0;
uint32_t lastPeakLogMs = 0;
char bleBatchBuffer[BLE_BATCH_BUFFER_SIZE];
size_t bleBatchLength = 0;
size_t bleBatchLines = 0;
int waveformX = 0;
int previousSample = 0;
M5Canvas waveformSprite(&M5.Display);

BLEServer* bleServer = nullptr;
BLECharacteristic* bleRxChar = nullptr;
BLECharacteristic* bleTxChar = nullptr;
m5::mic_config_t internalMicConfig;
m5::mic_config_t externalMicConfig;

void updateDisplay();

void onMicBufferReleased(void* args, void* data, size_t length) {
  if (data != nullptr && length > 0) {
    portENTER_CRITICAL(&releasedMux);
    uint8_t nextHead = (releasedHead + 1) % 3;
    if (nextHead != releasedTail) {
      releasedBuffers[releasedHead] = static_cast<int16_t*>(data);
      releasedLengths[releasedHead] = min(length, MIC_BUFFER_SAMPLES);
      releasedHead = nextHead;
      micBufferCount++;
    }
    portEXIT_CRITICAL(&releasedMux);

    if (micBufferCount == 1 || micBufferCount % 10 == 0) {
      Serial.print("Audio buffer received: ");
      Serial.println(micBufferCount);
    }
  }
}

void printStatus(const char* text) {
  statusText = text;
  updateDisplay();
}

void drawWaveform(int16_t sampleValue) {
  const int traceTop = 34;
  const int traceBottom = M5.Display.height() - 4;
  const int spriteHeight = traceBottom - traceTop + 1;
  const int halfH = spriteHeight / 2;

  int x = waveformX;
  int y = traceTop + halfH;
  int32_t amplifiedSample = static_cast<int32_t>(sampleValue) * WAVEFORM_DISPLAY_GAIN;
  amplifiedSample = constrain(amplifiedSample, -32768, 32767);
  int valueY = halfH - (amplifiedSample * halfH / 32768);
  valueY = constrain(valueY, 0, spriteHeight - 1);

  waveformSprite.drawLine(x, previousSample, x + 1, valueY, TFT_GREEN);
  previousSample = valueY;

  waveformX++;
  if (waveformX >= M5.Display.width()) {
    waveformX = 0;
    waveformSprite.fillSprite(TFT_BLACK);
    previousSample = halfH;
    waveformSprite.drawFastHLine(0, previousSample, M5.Display.width(), TFT_DARKGREY);
  }
}

void updateDisplay() {
  M5.Display.fillRect(0, 0, M5.Display.width(), 34, TFT_BLACK);

  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
  const char* micText = (activeMic == MicSource::Internal)
      ? "MIC: ES8311 I2S"
      : "MIC: INMP441 I2S";
  const char* stateText = isRecording ? "STATE: MEASURING" : "STATE: STOPPED";

  M5.Display.drawString(micText, 10, 6);
  M5.Display.drawString(stateText, 10, 20);

  M5.Display.setTextColor(bleConnected ? TFT_GREEN : TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.drawString(bleConnected ? "BLE: CONNECTED" : "BLE: WAITING", 125, 6);
}

void sendBleBatch() {
  if (bleTxChar == nullptr || !bleConnected || bleBatchLines == 0) {
    return;
  }

  bleTxChar->setValue((uint8_t*)bleBatchBuffer, bleBatchLength);
  bleTxChar->notify();
  bleBatchLength = 0;
  bleBatchLines = 0;
}

void appendBleData(uint32_t ms, int value) {
  int written = snprintf(
      bleBatchBuffer + bleBatchLength,
      BLE_BATCH_BUFFER_SIZE - bleBatchLength,
      "%lu,%d\n",
      static_cast<unsigned long>(ms),
      value);
  if (written <= 0 || static_cast<size_t>(written) >= BLE_BATCH_BUFFER_SIZE - bleBatchLength) {
    sendBleBatch();
    bleBatchLength = 0;
    bleBatchLines = 0;
    written = snprintf(
        bleBatchBuffer,
        BLE_BATCH_BUFFER_SIZE,
        "%lu,%d\n",
        static_cast<unsigned long>(ms),
        value);
  }
  if (written > 0) {
    bleBatchLength += static_cast<size_t>(written);
    bleBatchLines++;
    if (bleBatchLines >= BLE_BATCH_LINES) {
      sendBleBatch();
    }
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    bleDisplayUpdatePending = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    bleDisplayUpdatePending = true;
    BLEDevice::startAdvertising();
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String value = pChar->getValue();
    if (value.length() > 0) {
      if (value[0] == '1') {
        isRecording = true;
      } else if (value[0] == '0') {
        isRecording = false;
      }
      bleDisplayUpdatePending = true;
    }
  }
};

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());

    BLEService* service = bleServer->createService(NUS_SERVICE_UUID);
    bleRxChar = service->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE);
    bleRxChar->setCallbacks(new MyCharacteristicCallbacks());

    bleTxChar = service->createCharacteristic(
        NUS_TX_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY);

  bleTxChar->addDescriptor(new BLE2902());
  bleTxChar->setValue("READY");

  service->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  BLEDevice::startAdvertising();
  printStatus("BLE READY");
}

void toggleRecording() {
  isRecording = !isRecording;
  if (isRecording) {
    if (!M5.Mic.isRunning()) {
      M5.Mic.config(activeMic == MicSource::Internal ? internalMicConfig : externalMicConfig);
      if (!M5.Mic.begin()) {
        isRecording = false;
        statusText = "MIC START FAILED";
        Serial.println("Microphone start failed");
        updateDisplay();
        return;
      }
    }

    micBufferCount = 0;
    portENTER_CRITICAL(&releasedMux);
    releasedHead = 0;
    releasedTail = 0;
    portEXIT_CRITICAL(&releasedMux);
    nextQueueAttemptMs = 0;
    micBufferPending = M5.Mic.record(micBufferA, MIC_BUFFER_SAMPLES);
    bool secondBufferQueued = M5.Mic.record(micBufferB, MIC_BUFFER_SAMPLES);
    if (!micBufferPending || !secondBufferQueued) {
      isRecording = false;
      M5.Mic.end();
      statusText = "BUFFER QUEUE FAILED";
      Serial.println("Microphone buffer queue failed");
      updateDisplay();
      return;
    }
    audioDataSeen = false;
    statusText = "MEASURING...";
    Serial.println("START RECORD");
  } else {
    sendBleBatch();
    M5.Mic.end();
    micBufferPending = false;
    statusText = "STOP RECORD";
    Serial.println("Stop RECORD");
  }
  updateDisplay();
}

void toggleMicSource() {
  isRecording = false;
  micBufferPending = false;
  M5.Mic.end();

  if (activeMic == MicSource::Internal) {
    activeMic = MicSource::External;
    M5.Mic.config(externalMicConfig);
    statusText = M5.Mic.begin() ? "INMP441 READY" : "INMP441 FAILED";
    Serial.println("Mic: INMP441 I2S");
  } else {
    activeMic = MicSource::Internal;
    M5.Mic.config(internalMicConfig);
    statusText = M5.Mic.begin() ? "ES8311 READY" : "ES8311 FAILED";
    Serial.println("Mic: ES8311 I2S");
  }

  updateDisplay();
}

void setup() {
  auto cfg = M5.config();
  cfg.internal_mic = true;
  M5.begin(cfg);
  Serial.begin(115200);
  M5.Display.setRotation(1);  // 横向き表示
  M5.Display.setBrightness(128);
  M5.Display.setTextFont(&fonts::Font0);
  M5.Display.fillScreen(TFT_BLACK);

  const int traceTop = 34;
  const int traceBottom = M5.Display.height() - 4;
  if (!waveformSprite.createSprite(M5.Display.width(), traceBottom - traceTop + 1)) {
    Serial.println("Waveform sprite allocation failed");
  }
  waveformSprite.fillSprite(TFT_BLACK);
  previousSample = waveformSprite.height() / 2;
  waveformSprite.drawFastHLine(0, previousSample, waveformSprite.width(), TFT_DARKGREY);
  waveformSprite.pushSprite(0, traceTop);

  previousSample = waveformSprite.height() / 2;
  waveformX = 0;

  internalMicConfig = M5.Mic.config();
  internalMicConfig.sample_rate = SAMPLE_RATE;

  externalMicConfig = internalMicConfig;
  externalMicConfig.pin_data_in = EXTERNAL_I2S_DATA_PIN;
  externalMicConfig.pin_bck = EXTERNAL_I2S_BCLK_PIN;
  externalMicConfig.pin_ws = EXTERNAL_I2S_WS_PIN;
  externalMicConfig.pin_mck = I2S_PIN_NO_CHANGE;
  externalMicConfig.input_channel = m5::input_only_left;
  externalMicConfig.stereo = false;
  externalMicConfig.use_adc = false;
  M5.Mic.end();
  M5.Mic.config(internalMicConfig);
  bool micStarted = M5.Mic.begin();
  Serial.print("M5 board: ");
  Serial.println(static_cast<int>(M5.getBoard()));
  Serial.print("Mic enabled: ");
  Serial.println(M5.Mic.isEnabled() ? "yes" : "no");
  Serial.print("Mic running: ");
  Serial.println(M5.Mic.isRunning() ? "yes" : "no");
  Serial.print("I2S pins MCLK/BCLK/WS/DIN: ");
  Serial.print(internalMicConfig.pin_mck);
  Serial.print("/");
  Serial.print(internalMicConfig.pin_bck);
  Serial.print("/");
  Serial.print(internalMicConfig.pin_ws);
  Serial.print("/");
  Serial.println(internalMicConfig.pin_data_in);
  if (!micStarted || !M5.Mic.isRunning()) {
    statusText = "ES8311 INIT FAILED";
    Serial.println("ES8311 microphone initialization failed");
  } else {
    statusText = "ES8311 READY";
  }

  M5.Mic.setBufferReleaseCallback(nullptr, onMicBufferReleased);
  setupBle();
  statusText = (micStarted && M5.Mic.isRunning()) ? "ES8311 READY" : "ES8311 INIT FAILED";
  updateDisplay();

  delay(500);
  updateDisplay();
  Serial.println("Start");
}

void loop() {
  M5.update();

  if (bleDisplayUpdatePending) {
    bleDisplayUpdatePending = false;
    updateDisplay();
  }

  if (M5.BtnA.wasPressed()) {
    toggleRecording();
  }

  if (M5.BtnB.wasPressed()) {
    toggleMicSource();
  }

  int16_t* releasedBuffer = nullptr;
  size_t sampleCount = 0;
  portENTER_CRITICAL(&releasedMux);
  if (releasedTail != releasedHead) {
    releasedBuffer = const_cast<int16_t*>(releasedBuffers[releasedTail]);
    sampleCount = releasedLengths[releasedTail];
    releasedTail = (releasedTail + 1) % 3;
  }
  portEXIT_CRITICAL(&releasedMux);

  if (releasedBuffer != nullptr) {
    for (size_t index = 0; index < sampleCount; index += WAVEFORM_DISPLAY_DECIMATION) {
      int16_t displaySample = releasedBuffer[index];
      int displayPeak = abs(static_cast<int>(displaySample));
      size_t windowEnd = min(index + WAVEFORM_DISPLAY_DECIMATION, sampleCount);
      for (size_t windowIndex = index + 1; windowIndex < windowEnd; ++windowIndex) {
        int sampleMagnitude = abs(static_cast<int>(releasedBuffer[windowIndex]));
        if (sampleMagnitude > displayPeak) {
          displaySample = releasedBuffer[windowIndex];
          displayPeak = sampleMagnitude;
        }
      }
      drawWaveform(displaySample);
    }
    waveformSprite.pushSprite(0, 34);

    int value = 0;
    for (size_t index = 0; index < sampleCount; ++index) {
      value = max(value, abs(static_cast<int>(releasedBuffer[index])));
    }
    latestMicSample = value;

    if (millis() - lastPeakLogMs >= 500) {
      lastPeakLogMs = millis();
      // Serial.print("PCM peak: ");
      // Serial.println(value);
    }

    if (!audioDataSeen) {
      audioDataSeen = true;
      statusText = "AUDIO OK";
      Serial.print("Audio buffer received: ");
      Serial.println(micBufferCount);
      updateDisplay();
    }

    appendBleData(millis(), value);

    if (isRecording) {
      bool queued = M5.Mic.record(releasedBuffer, MIC_BUFFER_SAMPLES);
      if (queued) {
        micBufferPending = true;
        // Serial.println("Next buffer queue: OK");
        // Serial.print("M5.Mic.isRunning: ");
        // Serial.println(M5.Mic.isRunning() ? "yes" : "no");
      } else {
        statusText = "RECORD FAILED";
        Serial.println("Next buffer queue: FAILED");
        updateDisplay();
      }
    }

  }

  delay(1);
}
