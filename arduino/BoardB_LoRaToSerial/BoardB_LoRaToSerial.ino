#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"

// Board B: LoRa -> USB Serial.
// This first version validates fake PIPER packets and forwards them to Python.

#define RF_FREQUENCY 923200000
#define TX_OUTPUT_POWER 10
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

static const uint16_t MAX_PACKET_SIZE = 180;
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t STALE_TIMEOUT_MS = 1000;

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

static RadioEvents_t RadioEvents;
static char rxBuffer[MAX_PACKET_SIZE + 1];
static volatile bool rxDone = false;
static volatile uint16_t rxSize = 0;
static volatile int16_t rxRssi = 0;
static volatile int8_t rxSnr = 0;
static uint32_t lastValidPacketMs = 0;
static uint32_t lastStalePrintMs = 0;
static bool staleDisplayed = false;

static void enableExternalPower() {
  // Heltec WiFi LoRa 32 V4 powers the OLED through Vext. In the Heltec
  // library examples for this board, Vext is active-low.
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

static uint16_t rotateLeft5(uint16_t value) {
  return (uint16_t)((value << 5) | (value >> 11));
}

static uint16_t checksum16(const char *payload) {
  uint16_t c = 0x1234;
  while (*payload) {
    c = rotateLeft5(c);
    c ^= (uint8_t)(*payload);
    payload++;
  }
  return c;
}

static bool startsWithPiper(const char *line) {
  return strncmp(line, "PIPER,", 6) == 0;
}

static bool extractSeq(const char *line, long *seqOut) {
  if (!startsWithPiper(line)) {
    return false;
  }
  char *endPtr = nullptr;
  long seq = strtol(line + 6, &endPtr, 10);
  if (endPtr == line + 6 || *endPtr != ',') {
    return false;
  }
  *seqOut = seq;
  return true;
}

static bool validateChecksum(char *line) {
  char *lastComma = strrchr(line, ',');
  if (lastComma == nullptr) {
    return false;
  }

  char *checksumText = lastComma + 1;
  char *endPtr = nullptr;
  unsigned long received = strtoul(checksumText, &endPtr, 10);
  if (endPtr == checksumText || *endPtr != '\0' || received > 65535UL) {
    return false;
  }

  *lastComma = '\0';
  uint16_t expected = checksum16(line);
  *lastComma = ',';

  return expected == (uint16_t)received;
}

static void drawRxStatus(const char *line, int16_t rssi) {
  long seq = -1;
  extractSeq(line, &seq);

  oled.clear();
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Board B");
  oled.drawString(0, 12, "LoRa->Serial");
  if (seq >= 0) {
    oled.drawString(0, 24, "seq " + String(seq));
  } else {
    oled.drawString(0, 24, "seq ?");
  }
  oled.drawString(0, 36, "RX ok");
  oled.drawString(0, 48, "RSSI " + String(rssi));
  oled.display();
}

static void drawStaleStatus() {
  oled.clear();
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Board B");
  oled.drawString(0, 14, "STALE");
  oled.drawString(0, 28, "no LoRa >1s");
  oled.drawString(0, 42, "fake stop");
  oled.display();
}

static void startReceive() {
  Radio.Rx(0);
}

static void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uint16_t copySize = size;
  if (copySize > MAX_PACKET_SIZE) {
    copySize = MAX_PACKET_SIZE;
  }
  memcpy(rxBuffer, payload, copySize);
  rxBuffer[copySize] = '\0';
  rxSize = copySize;
  rxRssi = rssi;
  rxSnr = snr;
  rxDone = true;
  Radio.Sleep();
}

static void onRxTimeout(void) {
  Radio.Sleep();
  startReceive();
}

static void onRxError(void) {
  Serial.println("# LoRa RX error");
  Radio.Sleep();
  startReceive();
}

static void processReceivedPacket() {
  rxDone = false;

  // Trim a trailing newline if a sender included one.
  while (rxSize > 0 && (rxBuffer[rxSize - 1] == '\n' || rxBuffer[rxSize - 1] == '\r')) {
    rxBuffer[rxSize - 1] = '\0';
    rxSize--;
  }

  Serial.print("# LoRa packet received RSSI=");
  Serial.print(rxRssi);
  Serial.print(" SNR=");
  Serial.println(rxSnr);

  if (!startsWithPiper(rxBuffer)) {
    Serial.println("# Dropping non-PIPER packet");
    startReceive();
    return;
  }
  if (!validateChecksum(rxBuffer)) {
    Serial.print("# Dropping invalid checksum: ");
    Serial.println(rxBuffer);
    startReceive();
    return;
  }

  lastValidPacketMs = millis();
  staleDisplayed = false;
  drawRxStatus(rxBuffer, rxRssi);

  // This exact PIPER line is consumed by computer2_fake_receiver.py.
  Serial.println(rxBuffer);
  startReceive();
}

static void checkStale() {
  uint32_t now = millis();
  uint32_t age = now - lastValidPacketMs;
  if (age <= STALE_TIMEOUT_MS) {
    return;
  }

  if (!staleDisplayed) {
    drawStaleStatus();
    staleDisplayed = true;
  }

  if (lastStalePrintMs == 0 || now - lastStalePrintMs > 3000) {
    Serial.println("# STALE: no valid LoRa packet for >1s, fake slave would stop/freeze");
    lastStalePrintMs = now;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  enableExternalPower();
  delay(100);
  oled.init();
  drawStaleStatus();
  lastValidPacketMs = millis();

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  RadioEvents.RxDone = onRxDone;
  RadioEvents.RxTimeout = onRxTimeout;
  RadioEvents.RxError = onRxError;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.println("# Board B LoRa->Serial ready at 923.2 MHz");
  startReceive();
}

void loop() {
  Radio.IrqProcess();
  if (rxDone) {
    processReceivedPacket();
  }
  checkStale();
}
