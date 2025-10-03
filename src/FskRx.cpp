// FskRx.cpp - Dedicated low-bitrate FSK receiver for T-Beam BPF (ESP32-S3 + SX1278)
// Matches transmitter payload format and shows decoded fields.
// Extended Frame format (TX side will be updated accordingly):
// $BPF,<fc>,<upt_ms>,<vbat_mv>,<last_rssi>,<tempC>,V<hex>,P<pwr>*XXXX\n
//  - V<hex>: 4-hex build/version tag
//  - P<pwr>: power source code (B=battery present, E=external/no batt info, ?=unknown)
// CRC-16 X.25 over body (between '$' and '*').

#include <Arduino.h>
#include <RadioLib.h>
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
#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN      (38)   // choose a safe GPIO; adjust if needed
#endif

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ 144.0f
#endif

// Allow custom interval (mostly unused for RX except for status display refresh)
#ifndef CONFIG_FSK_INTERVAL_MS
#define CONFIG_FSK_INTERVAL_MS 10000UL
#endif

static U8G2 *u8g2ptr = nullptr;
static XPowersAXP2101 *pmu2101 = nullptr; 
static XPowersAXP192  *pmu192  = nullptr;
static bool pmuReady = false;
static uint32_t packetCount = 0;
static unsigned long lastStatus = 0;

static Module fskModule(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
static SX1278 fskRadio(&fskModule);
static const float FSK_TARGET_BITRATE = 0.3f;   // kbps
static const float FSK_FALLBACK_BITRATE = 0.6f; // kbps
static const float FSK_DEV_KHZ = 0.6f;
static const float FSK_RXBW_KHZ = 5.0f;
static float configuredKbps = FSK_TARGET_BITRATE;

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

static bool probeI2C(uint8_t addr) { Wire.beginTransmission(addr); return Wire.endTransmission()==0; }
static void oledLines(const char* l1,const char* l2="",const char* l3="",const char* l4=""){
  if(!u8g2ptr) return; auto &u=*u8g2ptr; u.clearBuffer(); u.setFont(u8g2_font_6x10_tr);
  int y=10; if(l1&&*l1){u.drawStr(0,y,l1); y+=12;} if(l2&&*l2){u.drawStr(0,y,l2); y+=12;}
  if(l3&&*l3){u.drawStr(0,y,l3); y+=12;} if(l4&&*l4){u.drawStr(0,y,l4);} u.sendBuffer();
}

static uint16_t readBatteryMilliVolts(){
  if(pmu2101) return (uint16_t)pmu2101->getBattVoltage();
  if(pmu192)  return (uint16_t)pmu192->getBattVoltage();
  return 0;
}

struct DecodedFrame {
  uint32_t fc=0; uint32_t uptime=0; uint16_t vbat=0; uint16_t lastRssi=0; float temp=0.0f;
  uint16_t version=0; char pwr='?'; uint16_t crcCalc=0; uint16_t crcField=0; bool ok=false; String raw;
};

static bool parseFrame(const String &rx, DecodedFrame &out){
  out = DecodedFrame(); out.raw = rx;
  if(rx.length()<12) return false;
  if(rx[0]!='$') return false;
  int star = rx.lastIndexOf('*'); if(star < 6 || star > (int)rx.length()-5) return false;
  // Extract CRC field
  if(star+5 > (int)rx.length()) return false;
  if(sscanf(rx.c_str()+star+1, "%4hx", &out.crcField) != 1) return false;
  String body = rx.substring(1, star);
  out.crcCalc = crc_x25((const uint8_t*)body.c_str(), body.length());
  if(out.crcCalc != out.crcField) return false;
  // Tokenize
  // Expect at minimum: BPF,fc,upt,vbat,last_rssi,temp plus optional Vxxxx,Px
  // We'll split by comma safely.
  int fieldIdx = 0; int start = 0; int len = body.length();
  String fields[10];
  for(int i=0;i<=len;i++){
    if(i==len || body[i]==','){
      if(fieldIdx < 10) fields[fieldIdx++] = body.substring(start,i);
      start = i+1;
    }
  }
  if(fieldIdx < 6) return false;
  if(fields[0] != "BPF") return false;
  out.fc = fields[1].toInt();
  out.uptime = fields[2].toInt();
  out.vbat = (uint16_t)fields[3].toInt();
  out.lastRssi = (uint16_t)fields[4].toInt();
  out.temp = fields[5].toFloat();
  // Optional extra fields
  for(int i=6;i<fieldIdx;i++){
    if(fields[i].length()>1 && fields[i][0]=='V'){
      out.version = (uint16_t)strtoul(fields[i].c_str()+1,nullptr,16);
    } else if(fields[i].length()>1 && fields[i][0]=='P'){
      out.pwr = fields[i][1];
    }
  }
  out.ok = true; return true;
}

static const char* resetReasonToStr(esp_reset_reason_t r){
  switch(r){
    case ESP_RST_POWERON: return "POWERON"; case ESP_RST_SW: return "SW"; case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT"; case ESP_RST_TASK_WDT: return "TASK_WDT"; case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP"; case ESP_RST_BROWNOUT: return "BROWNOUT"; case ESP_RST_SDIO: return "SDIO";
    default: return "OTHER"; }
}

void setup(){
  Serial.begin(115200);
  delay(1200);
  Serial.println(F("[BOOT] FSK RX Beacon"));
  Serial.printf("[RST] Reason=%s (%d)\n", resetReasonToStr(esp_reset_reason()), (int)esp_reset_reason());

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
  pinMode(STATUS_LED_PIN, OUTPUT); digitalWrite(STATUS_LED_PIN, LOW);
  delay(30);

  if(probeI2C(0x3C)){
    u8g2ptr = new U8G2_SH1106_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
    u8g2ptr->begin();
  }
  oledLines("FSK RX","Init", pmuReady?"PMU OK":"No PMU", "");

  int state = fskRadio.beginFSK(CONFIG_RADIO_FREQ, FSK_TARGET_BITRATE, FSK_DEV_KHZ, FSK_RXBW_KHZ);
  if(state != RADIOLIB_ERR_NONE){
    Serial.printf("[RADIO] 300bps fail (%d) fallback 600bps\n", state);
    state = fskRadio.beginFSK(CONFIG_RADIO_FREQ, FSK_FALLBACK_BITRATE, FSK_DEV_KHZ, FSK_RXBW_KHZ);
    if(state == RADIOLIB_ERR_NONE) configuredKbps = FSK_FALLBACK_BITRATE;
  }
  if(state != RADIOLIB_ERR_NONE){
    Serial.printf("[ERR] FSK init failed code=%d\n", state);
    while(true) { delay(1000);} }
  fskRadio.setPreambleLength(16); fskRadio.setCRC(true);
  Serial.printf("[RADIO] RX ready @ %.3f MHz BR=%.0f bps\n", CONFIG_RADIO_FREQ, configuredKbps*1000.0f);
  oledLines("RX Ready","Freq OK", "Listening", "");
  fskRadio.startReceive();
}

void loop(){
  int16_t state = fskRadio.readData(); // attempt buffered read; then fetch if ok
  if(state == RADIOLIB_ERR_NONE){
    String rx;
    fskRadio.readData(rx); // retrieve payload string
    digitalWrite(STATUS_LED_PIN, HIGH);
    DecodedFrame df; bool ok = parseFrame(rx, df);
    digitalWrite(STATUS_LED_PIN, LOW);
    if(ok){
      packetCount++;
      Serial.printf("[RX] OK FC=%lu UPT=%lu VBAT=%u RSSI=%d TEMP=%.1f VER=%04X PWR=%c CRC=%04X\n",
                    (unsigned long)df.fc,(unsigned long)df.uptime,df.vbat, -(int)fskRadio.getRSSI(), df.temp,
                    df.version, df.pwr, df.crcField);
      char l2[22]; snprintf(l2,sizeof(l2),"FC:%lu", (unsigned long)df.fc);
      char l3[22]; snprintf(l3,sizeof(l3),"VB:%u R:%d", df.vbat, (int)fskRadio.getRSSI());
      char l4[22]; snprintf(l4,sizeof(l4),"V%04X %c", df.version, df.pwr);
      oledLines("RX OK", l2, l3, l4);
    } else {
      Serial.print("[RX] BAD "); Serial.println(rx);
      oledLines("RX BAD","CRC/Parse",""," ");
    }
    fskRadio.startReceive();
  } else if(state != RADIOLIB_ERR_RX_TIMEOUT && state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RX] Err=%d restart\n", state);
    fskRadio.startReceive();
  }

  unsigned long now = millis();
  if(now - lastStatus > CONFIG_FSK_INTERVAL_MS){
    lastStatus = now;
    uint16_t vb = readBatteryMilliVolts();
    char l2[22]; snprintf(l2,sizeof(l2),"Pkts:%lu", (unsigned long)packetCount);
    char l3[22]; snprintf(l3,sizeof(l3),"VB:%u", vb);
    oledLines("Idle", l2, l3, pmuReady?"PMU":"NoPMU");
  }
}
