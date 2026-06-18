#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"

// Board B: LoRa -> USB Serial for real Piper teleop packets.

#define RF_FREQUENCY 923200000
#define TX_OUTPUT_POWER 10
#define LORA_BANDWIDTH 1
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

static const uint16_t TELEOP_PACKET_SIZE = 47;
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t STALE_TIMEOUT_MS = 1000;
static const uint8_t TELEOP_MAGIC[4] = {'P', 'L', 'T', '1'};

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

static RadioEvents_t RadioEvents;
static uint8_t rxBuffer[TELEOP_PACKET_SIZE];
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

static uint16_t crc16Ccitt(const uint8_t *payload, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t index = 0; index < length; index++) {
    crc ^= (uint16_t)payload[index] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc & 0x8000) != 0) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

static uint16_t readLe16(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t readLeU32(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static bool hasMagic(const uint8_t *packet) {
  for (uint8_t index = 0; index < 4; index++) {
    if (packet[index] != TELEOP_MAGIC[index]) {
      return false;
    }
  }
  return true;
}

static bool validatePacket(const uint8_t *packet, uint16_t size) {
  if (size != TELEOP_PACKET_SIZE || !hasMagic(packet)) {
    return false;
  }
  uint16_t received = readLe16(packet + TELEOP_PACKET_SIZE - 2);
  uint16_t expected = crc16Ccitt(packet, TELEOP_PACKET_SIZE - 2);
  return received == expected;
}

static uint32_t packetSeq(const uint8_t *packet) {
  return readLeU32(packet + 5);
}

static void drawRxStatus(uint32_t seq, int16_t rssi) {

  oled.clear();
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Board B");
  oled.drawString(0, 12, "LoRa->Serial");
  oled.drawString(0, 24, "seq " + String(seq));
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
  oled.drawString(0, 42, "holding");
  oled.display();
}

static void startReceive() {
  Radio.Rx(0);
}

static void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uint16_t copySize = size;
  if (copySize > TELEOP_PACKET_SIZE) {
    copySize = TELEOP_PACKET_SIZE;
  }
  memcpy(rxBuffer, payload, copySize);
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

  if (!validatePacket(rxBuffer, rxSize)) {
    Serial.println("# Dropping invalid binary packet");
    startReceive();
    return;
  }

  uint32_t seq = packetSeq(rxBuffer);
  lastValidPacketMs = millis();
  staleDisplayed = false;
  Serial.write(rxBuffer, TELEOP_PACKET_SIZE);
  drawRxStatus(seq, rxRssi);

  Serial.print("# LoRa packet received seq=");
  Serial.print(seq);
  Serial.print(" RSSI=");
  Serial.print(rxRssi);
  Serial.print(" SNR=");
  Serial.print(rxSnr);
  Serial.print(" size=");
  Serial.println(rxSize);
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
    Serial.println("# STALE: no valid LoRa packet for >1s, receiver should hold last command");
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

  Serial.println("# Board B LoRa->Serial ready at 923.2 MHz BW250 SF7");
  startReceive();
}

void loop() {
  Radio.IrqProcess();
  if (rxDone) {
    processReceivedPacket();
  }
  checkStale();
}
