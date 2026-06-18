#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"

// Board A: USB Serial -> LoRa for real Piper teleop packets.

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
static const uint8_t TELEOP_MAGIC[4] = {'P', 'L', 'T', '1'};

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

static RadioEvents_t RadioEvents;
static uint8_t inputPacket[TELEOP_PACKET_SIZE];
static uint16_t inputPos = 0;
static bool txBusy = false;

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

static bool validatePacket(const uint8_t *packet) {
  if (!hasMagic(packet)) {
    return false;
  }
  uint16_t received = readLe16(packet + TELEOP_PACKET_SIZE - 2);
  uint16_t expected = crc16Ccitt(packet, TELEOP_PACKET_SIZE - 2);
  return received == expected;
}

static uint32_t packetSeq(const uint8_t *packet) {
  return readLeU32(packet + 5);
}

static void drawStatus(uint32_t seq, const char *status) {

  oled.clear();
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Board A");
  oled.drawString(0, 12, "Serial->LoRa");
  oled.drawString(0, 24, "seq " + String(seq));
  oled.drawString(0, 36, status);
  oled.drawString(0, 48, "923.2 MHz BW250");
  oled.display();
}

static void onTxDone(void) {
  txBusy = false;
  Radio.Sleep();
  Serial.println("TX done");
}

static void onTxTimeout(void) {
  txBusy = false;
  Radio.Sleep();
  Serial.println("TX timeout");
}

static void sendPacketOverLoRa(uint8_t *packet) {
  if (txBusy) {
    Serial.println("WARN: LoRa TX busy, dropping packet");
    return;
  }

  uint32_t seq = packetSeq(packet);
  txBusy = true;
  Radio.Send(packet, TELEOP_PACKET_SIZE);
  drawStatus(seq, "sent packet");
}

static bool currentPrefixMatches() {
  uint16_t prefixBytes = inputPos < 4 ? inputPos : 4;
  for (uint16_t index = 0; index < prefixBytes; index++) {
    if (inputPacket[index] != TELEOP_MAGIC[index]) {
      return false;
    }
  }
  return true;
}

static void resetParserWithByte(uint8_t value) {
  inputPos = 0;
  if (value == TELEOP_MAGIC[0]) {
    inputPacket[inputPos++] = value;
  }
}

static void handleSerialByte(uint8_t value) {
  if (inputPos >= TELEOP_PACKET_SIZE) {
    inputPos = 0;
  }

  inputPacket[inputPos++] = value;
  if (!currentPrefixMatches()) {
    resetParserWithByte(value);
    return;
  }

  if (inputPos < TELEOP_PACKET_SIZE) {
    return;
  }

  if (validatePacket(inputPacket)) {
    sendPacketOverLoRa(inputPacket);
  } else {
    Serial.println("WARN: dropping invalid binary packet");
  }
  inputPos = 0;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  enableExternalPower();
  delay(100);
  oled.init();
  drawStatus(0, "waiting serial");

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  RadioEvents.TxDone = onTxDone;
  RadioEvents.TxTimeout = onTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Serial.println("Board A Serial->LoRa ready at 923.2 MHz BW250 SF7");
}

void loop() {
  Radio.IrqProcess();
  while (Serial.available() > 0) {
    handleSerialByte((uint8_t)Serial.read());
  }
}
