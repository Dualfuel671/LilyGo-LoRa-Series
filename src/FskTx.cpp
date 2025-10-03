// FskTx.cpp - Dedicated low-bitrate FSK transmitter for T-Beam BPF (ESP32-S3 + SX1278)
// Generates structured ASCII payload frames with CRC-16 X.25.
// Frame format (extended):
// $BPF,<fc>,<upt_ms>,<vbat_mv>,<last_rssi>,<tempC>,Vhhhh,Ps*XXXX\n
//  - Vhhhh: 4-hex version tag (compile-time VERSION_TAG or derived)
//  - Ps: power source code (B=battery present, E=external/no battery, ?=unknown)
// CRC-16 X.25 over body between '$' and '*'.
// DISCLAIMER: Experimental beacon, not an AX.25 compatible on-air signal.

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <XPowersLib.h>

#ifndef T_BEAM_S3_BPF
#define T_BEAM_S3_BPF
#endif

// ---------------- Pin Mapping ----------------
#define I2C_SDA             (8)
#define I2C_SCL             (9)
#define RADIO_CS_PIN        (1)
#define RADIO_RST_PIN       (18)
#define RADIO_DIO0_PIN      (14)
#define RADIO_DIO1_PIN      (21)
#define RADIO_LDO_EN        (16)

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ 144.0f   // MHz (adjust as needed)
#endif

#ifndef CONFIG_FSK_INTERVAL_MS
#define CONFIG_FSK_INTERVAL_MS 60000UL
#endif

#ifndef VERSION_TAG
// Fallback version tag: simple hash of build date/time
static constexpr uint16_t fallbackVersionTag = \
  ((uint16_t)(__DATE__[4] + __TIME__[6] + __TIME__[7]) << 8) ^ (uint16_t)(__DATE__[0] + __TIME__[0]);
#define VERSION_TAG fallbackVersionTag
#endif

static const unsigned BEACON_INTERVAL_MS = CONFIG_FSK_INTERVAL_MS;
static uint32_t frameCounter = 0;
static unsigned long lastSend = 0;
static bool firstTx = false;
static RTC_DATA_ATTR uint32_t bootCounter = 0;

static U8G2 *u8g2ptr = nullptr;
static XPowersAXP2101 *pmu2101 = nullptr;
static XPowersAXP192  *pmu192  = nullptr;
static bool pmuReady = false;

// FSK radio
static Module fskModule(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
static SX1278 fskRadio(&fskModule);
static const float FSK_TARGET_BITRATE = 0.3f;   // kbps (300 bps attempt)
static const float FSK_FALLBACK_BITRATE = 0.6f; // kbps (fallback 600 bps)
static const float FSK_DEV_KHZ = 0.6f;
static const float FSK_RXBW_KHZ = 5.0f;
static uint16_t lastFSKRSSI = 0;
static uint16_t lastVBat_mV = 0;
static float fakeTempC = 0.0f; // placeholder for future real sensor
static float configuredKbps = FSK_TARGET_BITRATE; // record chosen bitrate

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 38
#endif

// CRC-16 X.25
static uint16_t crc_x25(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;++i){
    crc ^= data[i];
    for(int b=0;b<8;++b){
      if(crc & 1) crc = (crc >> 1) ^ 0x8408; else crc >>= 1;
    }
  }
  return ~crc;
}

static bool probeI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void oledLines(const char* l1,const char* l2="",const char* l3="",const char* l4=""){
  if(!u8g2ptr) return; auto &u=*u8g2ptr; u.clearBuffer(); u.setFont(u8g2_font_6x10_tr);
  int y=10; if(l1&&*l1){u.drawStr(0,y,l1); y+=12;} if(l2&&*l2){u.drawStr(0,y,l2); y+=12;}
  if(l3&&*l3){u.drawStr(0,y,l3); y+=12;} if(l4&&*l4){u.drawStr(0,y,l4);} u.sendBuffer();
}

static void initDisplay() {
  if(!probeI2C(0x3C)) return;
  u8g2ptr = new U8G2_SH1106_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
  u8g2ptr->begin();
  oledLines("FSK TX","Init...",""," ");
}

static uint16_t readBatteryMilliVolts(){
  if(pmu2101) return (uint16_t)pmu2101->getBattVoltage();
  if(pmu192)  return (uint16_t)pmu192->getBattVoltage();
  return 0;
}

static char detectPowerSourceCode(){
  if(pmu2101){
    bool battPresent = pmu2101->isBatteryConnect();
    if(battPresent) return 'B';
    return 'E';
  }
  if(pmu192){
    bool battPresent = pmu192->isBatteryConnect();
    if(battPresent) return 'B';
    return 'E';
  }
  return '?';
}

static size_t buildPayload(char *out,size_t maxLen,uint32_t fc,uint32_t uptime){
  lastVBat_mV = readBatteryMilliVolts();
  char body[192];
  char psrc = detectPowerSourceCode();
  snprintf(body,sizeof(body),"BPF,%lu,%lu,%u,%u,%.1f,V%04X,P%c",(unsigned long)fc,(unsigned long)uptime,
           (unsigned)lastVBat_mV,(unsigned)lastFSKRSSI,fakeTempC,(unsigned)VERSION_TAG, psrc);
  uint16_t c = crc_x25((const uint8_t*)body, strlen(body));
  char frame[224];
  snprintf(frame,sizeof(frame),"$%s*%04X\n",body,c & 0xFFFF);
  size_t flen = strlen(frame);
  if(flen+1>maxLen) return 0; memcpy(out,frame,flen+1); return flen;
}

static const char* resetReasonToStr(esp_reset_reason_t r){
  switch(r){
    case ESP_RST_POWERON: return "POWERON"; case ESP_RST_SW: return "SW"; case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT"; case ESP_RST_TASK_WDT: return "TASK_WDT"; case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP"; case ESP_RST_BROWNOUT: return "BROWNOUT"; case ESP_RST_SDIO: return "SDIO";
    default: return "OTHER";
  }
}

void setup(){
  Serial.begin(115200);
  delay(1500);
  bootCounter++;
  Serial.println(F("[BOOT] FSK TX Beacon"));
  Serial.printf("[RST] Reason=%s (%d)\n", resetReasonToStr(esp_reset_reason()), (int)esp_reset_reason());
  Serial.printf("[BUILD] %s %s\n", __DATE__, __TIME__);
  Serial.printf("[BOOT] Count=%lu\n", (unsigned long)bootCounter);

  // Power rails + I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  pmu2101 = new XPowersAXP2101(Wire);
  if(pmu2101 && pmu2101->init()){
    pmuReady = true; Serial.println(F("[PMU] AXP2101"));
    pmu2101->setALDO4Voltage(3300); pmu2101->enableALDO4();
    pmu2101->setALDO2Voltage(3300); pmu2101->enableALDO2();
    pmu2101->setDC3Voltage(3300);   pmu2101->enableDC3();
    pmu2101->setDC5Voltage(3300);   pmu2101->enableDC5();
    pmu2101->setALDO1Voltage(3300); pmu2101->enableALDO1();
    pmu2101->setALDO3Voltage(3300); pmu2101->enableALDO3();
  } else {
    delete pmu2101; pmu2101=nullptr; pmu192 = new XPowersAXP192(Wire);
    if(pmu192 && pmu192->init()){
      pmuReady = true; Serial.println(F("[PMU] AXP192"));
      pmu192->setLDO2Voltage(3300); pmu192->enableLDO2();
      pmu192->setLDO3Voltage(3300); pmu192->enableLDO3();
    }
  }
  pinMode(RADIO_LDO_EN, OUTPUT); digitalWrite(RADIO_LDO_EN, HIGH);
  delay(30);

  initDisplay();
  oledLines("FSK TX","Power", pmuReady?"PMU OK":"No PMU", "");

  // Init FSK radio
  Serial.println(F("[RADIO] Init FSK"));
  // RadioLib handles SPI internally; explicit SPI.begin() not required unless custom pins diverge.
  pinMode(STATUS_LED_PIN, OUTPUT); digitalWrite(STATUS_LED_PIN, LOW);

  configuredKbps = FSK_TARGET_BITRATE; // track what we end up using
  int state = fskRadio.beginFSK(CONFIG_RADIO_FREQ, FSK_TARGET_BITRATE, FSK_DEV_KHZ, FSK_RXBW_KHZ);
  if(state != RADIOLIB_ERR_NONE){
    Serial.printf("[RADIO] 300bps fail (%d) fallback 600bps\n", state);
    state = fskRadio.beginFSK(CONFIG_RADIO_FREQ, FSK_FALLBACK_BITRATE, FSK_DEV_KHZ, FSK_RXBW_KHZ);
    if(state == RADIOLIB_ERR_NONE) configuredKbps = FSK_FALLBACK_BITRATE;
  }
  if(state != RADIOLIB_ERR_NONE){
    Serial.printf("[ERR] FSK init failed code=%d\n", state);
    while(true){ delay(1000);} // halt
  }
  fskRadio.setPreambleLength(16);
  fskRadio.setCRC(true);
  Serial.printf("[RADIO] FSK ready @ %.3f MHz BR=%.0f bps\n", CONFIG_RADIO_FREQ, configuredKbps*1000.0f);
  char line3[22]; snprintf(line3,sizeof(line3),"BR=%.0f", configuredKbps*1000.0f);
  oledLines("FSK Ready","Freq OK", line3, "");

  // Immediate first frame
  frameCounter = 1;
  char payload[192];
  size_t plen = buildPayload(payload, sizeof(payload), frameCounter, 0);
    if(plen){
      digitalWrite(STATUS_LED_PIN, HIGH);
      int tx = fskRadio.transmit(payload);
      digitalWrite(STATUS_LED_PIN, LOW);
    if(tx == RADIOLIB_ERR_NONE){
      Serial.printf("[TX0] len=%u %s", (unsigned)plen, payload);
      oledLines("Sent0","FC:1", payload, "");
      firstTx = true; lastSend = millis();
    } else {
      Serial.printf("[ERR] TX0 code=%d\n", tx);
    }
  }
}

void loop(){
  unsigned long now = millis();
  if(now - lastSend >= BEACON_INTERVAL_MS){
    lastSend = now; if(!firstTx){ frameCounter = 0; firstTx = true; }
    frameCounter++;
    char payload[192];
    size_t plen = buildPayload(payload,sizeof(payload),frameCounter,now);
    if(plen){
      digitalWrite(STATUS_LED_PIN, HIGH);
      int tx = fskRadio.transmit(payload);
      digitalWrite(STATUS_LED_PIN, LOW);
      if(tx == RADIOLIB_ERR_NONE){
        Serial.printf("[TX] FC=%lu len=%u %s", (unsigned long)frameCounter, (unsigned)plen, payload);
        char l2[20]; snprintf(l2,sizeof(l2),"FC:%lu", (unsigned long)frameCounter);
        oledLines("FSK Sent", l2, payload, "");
      } else {
        Serial.printf("[ERR] TX code=%d\n", tx);
        oledLines("TX ERR","",""," ");
      }
    }
  }
  delay(10);
}
