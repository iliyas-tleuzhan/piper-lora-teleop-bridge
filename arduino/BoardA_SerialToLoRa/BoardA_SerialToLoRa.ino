#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"

// Board A: USB Serial -> LoRa.
// This first version is a fake transport test only. It does not use CAN.

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

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

static RadioEvents_t RadioEvents;
static char inputLine[MAX_PACKET_SIZE + 1];
static uint16_t inputPos = 0;
static bool inputOverflow = false;
static bool txBusy = false;

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

static void drawStatus(const char *line, const char *status) {
  long seq = -1;
  extractSeq(line, &seq);

  oled.clear();
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Board A");
  oled.drawString(0, 12, "Serial->LoRa");
  if (seq >= 0) {
    oled.drawString(0, 24, "seq " + String(seq));
  } else {
    oled.drawString(0, 24, "seq ?");
  }
  oled.drawString(0, 36, status);
  oled.drawString(0, 48, "923.2 MHz");
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

static void sendLineOverLoRa(char *line) {
  if (!startsWithPiper(line)) {
    Serial.println("WARN: dropping non-PIPER line");
    return;
  }
  if (!validateChecksum(line)) {
    Serial.print("WARN: dropping invalid checksum: ");
    Serial.println(line);
    return;
  }
  if (txBusy) {
    Serial.println("WARN: LoRa TX busy, dropping packet");
    return;
  }

  long seq = -1;
  extractSeq(line, &seq);
  Serial.print("SERIAL RX: ");
  Serial.println(line);
  Serial.print("LORA TX seq ");
  Serial.println(seq);

  drawStatus(line, "sent packet");
  txBusy = true;
  Radio.Send((uint8_t *)line, strlen(line));
}

static void handleSerialByte(char ch) {
  if (ch == '\r') {
    return;
  }

  if (ch == '\n') {
    if (inputOverflow) {
      Serial.println("WARN: input line too long, dropped");
    } else if (inputPos > 0) {
      inputLine[inputPos] = '\0';
      sendLineOverLoRa(inputLine);
    }
    inputPos = 0;
    inputOverflow = false;
    return;
  }

  if (inputPos >= MAX_PACKET_SIZE) {
    inputOverflow = true;
    return;
  }

  inputLine[inputPos++] = ch;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  enableExternalPower();
  delay(100);
  oled.init();
  drawStatus("", "waiting serial");

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  RadioEvents.TxDone = onTxDone;
  RadioEvents.TxTimeout = onTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Serial.println("Board A Serial->LoRa ready at 923.2 MHz");
}

void loop() {
  Radio.IrqProcess();
  while (Serial.available() > 0) {
    handleSerialByte((char)Serial.read());
  }
}
