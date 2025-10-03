// Standalone LoRa pseudo-AX.25 Beacon for T-Beam BPF
// DISCLAIMER: This constructs AX.25-like UI frames (addresses + control + PID + CRC)
// but transmits them using LoRa modulation. Standard AX.25/APRS receivers expecting
// 1200 baud AFSK or 9600 direct FSK will NOT decode these over-the-air. This is for
// experimentation and debugging only. For real APRS/packet compatibility you must
// implement the proper physical layer.

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <RadioLib.h> // for FSK mode (SX127x)
#include <Wire.h>
#include <U8g2lib.h>
#include <XPowersLib.h> // PMU control for power rails (best effort)

#ifndef T_BEAM_S3_BPF
#define T_BEAM_S3_BPF
#endif

// ---------------- Pin Mapping (mirrored from examples/Factory/utilities.h under T_BEAM_S3_BPF) ----------------
#define I2C_SDA             (8)
#define I2C_SCL             (9)

#define RADIO_SCLK_PIN      (12)
#define RADIO_MISO_PIN      (13)
#define RADIO_MOSI_PIN      (11)
#define RADIO_CS_PIN        (1)
#define RADIO_RST_PIN       (18)
#define RADIO_DIO0_PIN      (14)
#define RADIO_DIO1_PIN      (21)

#define RADIO_LDO_EN        (16)
#define RADIO_CTRL          (39)   // LOW = TX path, HIGH = RX path per board design

// ---------------- Configuration Defaults ----------------
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ 145.10      // MHz
#endif

#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER 10  // dBm (adjust responsibly)
#endif

#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW 125.0         // kHz
#endif

// User adjustable beacon settings
static const char* MY_CALLSIGN = "KE8DCJ";        // Your callsign
static const uint8_t MY_SSID   = 0;               // Source SSID (0-15)
static const char* DEST_CALLSIGN = "BEACON";      // Destination callsign
static const uint8_t DEST_SSID   = 0;             // Destination SSID
// Optional digipeater path entries (leave empty for none)
struct Ax25PathEntry { const char* call; uint8_t ssid; };
static Ax25PathEntry DIGI_PATH[] = { /* {"WIDE1",1}, {"WIDE2",2} */ };
static const size_t DIGI_PATH_COUNT = sizeof(DIGI_PATH)/sizeof(DIGI_PATH[0]);

// ---------------- Mode Selection ----------------
// Define USE_FSK_300 to build a narrowband ~300 bps (fallback 600 bps) FSK beacon instead of LoRa CSS.
// Add to build flags: -DUSE_FSK_300
// Forward declare CRC used by both AX.25 and FSK framing before FSK helpers when compiled early.
static uint16_t ax25_crc(const uint8_t* data, size_t len); // actual definition appears later

#ifdef USE_FSK_300
#define MODE_NAME "FSK300"
#else
#define MODE_NAME "LoRa"
#endif

static const unsigned BEACON_INTERVAL_MS = 60000; // 60s main interval
// (Removed early keepalive heartbeat)
static uint32_t frameCounter = 0;
static unsigned long lastSend = 0;
static bool firstBeaconSent = false;
static RTC_DATA_ATTR uint32_t bootCounter = 0; // survives soft resets

// Forward decl
static void logResetInfo();
static bool pmuReady = false; // track PMU presence for display line

// Early forward declarations (will allocate later in setup). Define as raw pointers here.
class U8G2; // forward class
static U8G2 *u8g2ptr = nullptr; // will point to a created instance
static XPowersAXP2101 *pmu2101 = nullptr; // allocated in setup if detected
static XPowersAXP192  *pmu192  = nullptr; // fallback older PMU

#ifdef USE_FSK_300
// RadioLib FSK instance (NOTE: pin mapping adapted; DIO0 used for packet done)
// NSS, DIO0, RESET, DIO1
static Module fskModule(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
static SX1278 fskRadio(&fskModule);
// Target parameters (attempt 300 bps; may fall back to 600 if unsupported by RadioLib internals)
static const float FSK_TARGET_BITRATE = 0.3f;   // kbps (300 bps)
static const float FSK_FALLBACK_BITRATE = 0.6f; // kbps (600 bps)
static const float FSK_DEV_KHZ = 0.6f;          // kHz deviation (~600 Hz)
static const float FSK_RXBW_KHZ = 5.0f;         // kHz receiver bandwidth
static bool fskMode = false;

// Simple ASCII payload framing for FSK mode:
// $BPF,<fc>,<upt_ms>,<vbat_mv>,<rssi_prev>,<tempC if available>*XXXX\n
// Where *XXXX is 4 hex digits of CRC-16 X25 over everything after '$' up to but not including '*'.
// (Reuse ax25_crc for convenience.)
static uint16_t lastFSKRSSI = 0;
static float fakeTempC = 0; // placeholder for future sensor integration
static uint16_t lastVBat_mV = 0;

static uint16_t readBatteryMilliVolts() {
  // Best effort: if PMU present, query; else return 0 (not available)
  if (pmu2101) {
    // XPowersLib AXP2101 exposes getBattVoltage() in mV if battery detected; guard usage
    return (uint16_t)pmu2101->getBattVoltage();
  }
  if (pmu192) {
    return (uint16_t)pmu192->getBattVoltage();
  }
  return 0;
}

static size_t build_fsk_payload(char *out, size_t maxLen, uint32_t fc, uint32_t uptimeMs) {
  lastVBat_mV = readBatteryMilliVolts();
  // Compose body without leading '$' and without CRC first
  // Format: BPF,fc,upt,vbat,rssi,temp
  char body[160];
  snprintf(body, sizeof(body), "BPF,%lu,%lu,%u,%u,%.1f", (unsigned long)fc, (unsigned long)uptimeMs,
           (unsigned)lastVBat_mV, (unsigned)lastFSKRSSI, fakeTempC);
  uint16_t crc = ax25_crc((const uint8_t*)body, strlen(body));
  char frame[192];
  snprintf(frame, sizeof(frame), "$%s*%04X\n", body, crc & 0xFFFF);
  size_t flen = strlen(frame);
  if (flen + 1 > maxLen) return 0;
  memcpy(out, frame, flen + 1);
  return flen;
}

// Parse an incoming FSK payload according to the framing above.
// Returns true if CRC is valid and fields parsed.
static bool parse_fsk_payload(const char *in, uint32_t &fc, uint32_t &upt, uint16_t &vbat, uint16_t &rssi, float &temp, uint16_t &crcCalc, uint16_t &crcField) {
  fc = upt = vbat = rssi = 0; temp = 0; crcCalc = crcField = 0;
  size_t len = strlen(in);
  if (len < 10) return false; // too short
  if (in[0] != '$') return false;
  const char *star = strrchr(in, '*');
  if (!star || star - in < 5 || (len - (star - in) - 1) < 4) return false;
  // Extract CRC field
  if (sscanf(star + 1, "%4hx", &crcField) != 1) return false;
  // Body between '$' and '*'
  char body[170];
  size_t bodyLen = (size_t)(star - (in + 1));
  if (bodyLen >= sizeof(body)) return false;
  memcpy(body, in + 1, bodyLen); body[bodyLen] = '\0';
  crcCalc = ax25_crc((const uint8_t*)body, bodyLen);
  if (crcCalc != crcField) return false;
  // Split fields: BPF,fc,upt,vbat,rssi,temp
  // Use sscanf with defensive matching
  char tag[8];
  double tempD = 0.0;
  unsigned long fcUL=0, uptUL=0; unsigned vbatU=0, rssiU=0;
  int matched = sscanf(body, "%7[^,],%lu,%lu,%u,%u,%lf", tag, &fcUL, &uptUL, &vbatU, &rssiU, &tempD);
  if (matched < 6) return false;
  if (strcmp(tag, "BPF") != 0) return false;
  fc = (uint32_t)fcUL; upt = (uint32_t)uptUL; vbat = (uint16_t)vbatU; rssi = (uint16_t)rssiU; temp = (float)tempD;
  return true;
}
#endif

// (moved declarations above for visibility)

// Build flag to try SSD1306 first (or fallback). Usage: add -DTRY_SSD1306_FALLBACK=1
#ifndef TRY_SSD1306_FALLBACK
#define TRY_SSD1306_FALLBACK 1
#endif

// Build flag to enable timing instrumentation
#ifndef ENABLE_STAGE_TIMING
#define ENABLE_STAGE_TIMING 1
#endif

struct StageTimer { unsigned long t0; };
static StageTimer ST;
static inline unsigned long ms() { return millis(); }
static inline void stageStart() { ST.t0 = ms(); }
static inline void stageLog(const char *label) {
#if ENABLE_STAGE_TIMING
  Serial.printf("[T]+%lums %s\n", (unsigned long)(ms() - ST.t0), label);
#endif
}

static void drawTestPattern(U8G2 &d, const char *phase) {
  d.clearBuffer();
  // Checkerboard 8x8 blocks
  for (uint8_t y=0; y<64; y+=8) {
    for (uint8_t x=0; x<128; x+=8) {
      if ( ((x^y) & 0x08) ) d.setDrawColor(1); else d.setDrawColor(0);
      d.drawBox(x,y,8,8);
    }
  }
  d.setDrawColor(1);
  d.setFont(u8g2_font_6x10_tr);
  d.drawStr(0,10, phase);
  d.drawStr(0,22, "OLED TEST");
  d.sendBuffer();
}

static bool probeI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool initDisplaySequence() {
  unsigned long tStart = ms();
  Serial.println(F("[DISP] === Begin display init sequence ==="));
  bool found = probeI2C(0x3C);
  Serial.printf("[DISP] Probe 0x3C %s (t+%lums)\n", found?"OK":"FAIL", (unsigned long)(ms()-tStart));
  if (!found) return false;

  // Attempt SH1106 first (expected on this board). Use HW I2C default (no explicit SDA/SCL args)
  unsigned long tAlloc = ms();
  u8g2ptr = new U8G2_SH1106_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
  Serial.printf("[DISP] Alloc SH1106 obj dt=%lums\n", (unsigned long)(ms()-tAlloc));

  // Guard: if begin blocks, we log pre and post
  unsigned long tBegin = ms();
  Serial.println(F("[DISP] Calling u8g2->begin()"));
  u8g2ptr->begin();
  Serial.printf("[DISP] u8g2->begin() returned (dt=%lums, total=%lums)\n", (unsigned long)(ms()-tBegin), (unsigned long)(ms()-tStart));

  drawTestPattern(*u8g2ptr, "SH1106");
  Serial.printf("[DISP] Pattern drawn (total=%lums)\n", (unsigned long)(ms()-tStart));

#if TRY_SSD1306_FALLBACK
  // Instead of unconditional fallback, only try if a user build flag explicitly requests via second macro
  #ifdef FORCE_SSD1306_SECOND_PASS
    Serial.println(F("[DISP] FORCE_SSD1306_SECOND_PASS defined; trying SSD1306 alt driver"));
    unsigned long tAlt = ms();
    U8G2 *alt = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
    Serial.printf("[DISP] Alloc SSD1306 obj dt=%lums\n", (unsigned long)(ms()-tAlt));
    unsigned long tAltBegin = ms();
    alt->begin();
    Serial.printf("[DISP] SSD1306 begin dt=%lums\n", (unsigned long)(ms()-tAltBegin));
    drawTestPattern(*alt, "SSD1306");
    delete u8g2ptr; u8g2ptr = alt;
  #endif
#endif

  Serial.println(F("[DISP] === Display init sequence done ==="));
  return true;
}

static void oledLines(const char* l1, const char* l2="", const char* l3="", const char* l4="") {
  if (!u8g2ptr) return;
  U8G2 &u = *u8g2ptr;
  u.clearBuffer();
  u.setFont(u8g2_font_6x10_tr);
  int y = 10;
  if (l1 && *l1) { u.drawStr(0, y, l1); y += 12; }
  if (l2 && *l2) { u.drawStr(0, y, l2); y += 12; }
  if (l3 && *l3) { u.drawStr(0, y, l3); y += 12; }
  if (l4 && *l4) { u.drawStr(0, y, l4); }
  u.sendBuffer();
}

// ---------------- AX.25 helper functions ----------------
// Encode an AX.25 address (callsign padded to 6 upper-case chars, each shifted left 1),
// SSID byte with bit0 = 1 only for *last* address in the entire chain.
static void ax25_encode_address(uint8_t *out, const char* call, uint8_t ssid, bool last) {
  char buf[6];
  size_t len = strlen(call);
  for (size_t i = 0; i < 6; ++i) {
    char c = (i < len) ? call[i] : ' ';
    if (c >= 'a' && c <= 'z') c -= 32; // upper-case
    buf[i] = c;
  }
  for (int i = 0; i < 6; ++i) out[i] = ((uint8_t)buf[i]) << 1;
  // SSID byte format:  0b0110SSSID0 ; set bit0 if last address
  uint8_t ssidByte = 0b01100000 | ((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00);
  out[6] = ssidByte;
}

// CRC-16-IBM/X.25
static uint16_t ax25_crc(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x01) crc = (crc >> 1) ^ 0x8408; else crc >>= 1;
    }
  }
  return ~crc;
}

// Build AX.25 UI frame (no bit-stuffing / NRZI since we are not on a native AX.25 physical layer).
// Returns length or 0 on failure.
static size_t build_ax25_ui(uint8_t* out, size_t outSize, const String &info) {
  size_t needed = (2 + DIGI_PATH_COUNT) * 7 + 1 + 1 + info.length() + 2;
  if (needed > outSize) return 0;
  uint8_t *p = out;
  ax25_encode_address(p, DEST_CALLSIGN, DEST_SSID, false); p += 7;
  ax25_encode_address(p, MY_CALLSIGN,   MY_SSID,   DIGI_PATH_COUNT == 0); p += 7;
  for (size_t i = 0; i < DIGI_PATH_COUNT; ++i) {
    bool last = (i == DIGI_PATH_COUNT - 1);
    ax25_encode_address(p, DIGI_PATH[i].call, DIGI_PATH[i].ssid, last); p += 7;
  }
  *p++ = 0x03; // Control: UI
  *p++ = 0xF0; // PID: No layer 3
  for (size_t i = 0; i < info.length(); ++i) *p++ = (uint8_t)info[i];
  uint16_t crc = ax25_crc(out, p - out);
  *p++ = crc & 0xFF;
  *p++ = (crc >> 8) & 0xFF;
  return p - out;
}

// Removed original oledLines (replaced above with pointer-safe variant)

void setup() {
  Serial.begin(115200);
  stageStart();
  // Optional early heartbeat: configurable number of dots for bring-up diagnostics
  #ifndef HEARTBEAT_DOTS
  #define HEARTBEAT_DOTS 0   // Set >0 to emit that many '.' at boot
  #endif
  for (int i = 0; i < HEARTBEAT_DOTS; ++i) { Serial.write('.'); delay(2); }
  Serial.println();
  // ========================== BOOT BANNER ===========================
  // If you do not see the next line in the serial monitor, it likely
  // printed before the monitor attached. Open the monitor first, then
  // press the board's RESET button to catch it.
  Serial.print(F("[BOOT] T-Beam BPF Beacon Mode="));
  Serial.println(F(MODE_NAME));
  // ==================================================================
  bootCounter++;
  logResetInfo();
  Serial.printf("[BUILD] Compiled %s %s\n", __DATE__, __TIME__);
  Serial.printf("[BOOT] Count=%lu\n", (unsigned long)bootCounter);

  // Delay a moment to allow USB CDC to settle
  delay(1500);
  stageLog("Serial init done");

  // PMU / Power rails
  stageStart();
  Wire.begin(I2C_SDA, I2C_SCL); // temporary for PMU detection (AXP2101 also on this bus)
  Wire.setClock(400000);        // speed up I2C (panel handles 400kHz)
  // Detect and enable required rails before any radio/display heavy init
  pmu2101 = new XPowersAXP2101(Wire);
  if (pmu2101 && pmu2101->init()) {
    pmuReady = true;
    Serial.println(F("[PMU] AXP2101 detected"));
    pmu2101->setALDO4Voltage(3300); pmu2101->enableALDO4();
    pmu2101->setALDO2Voltage(3300); pmu2101->enableALDO2();
    pmu2101->setDC3Voltage(3300);   pmu2101->enableDC3();
    pmu2101->setDC5Voltage(3300);   pmu2101->enableDC5();
    pmu2101->setALDO1Voltage(3300); pmu2101->enableALDO1();
    pmu2101->setALDO3Voltage(3300); pmu2101->enableALDO3();
  } else {
    delete pmu2101; pmu2101 = nullptr;
    pmu192 = new XPowersAXP192(Wire);
    if (pmu192 && pmu192->init()) {
      pmuReady = true;
      Serial.println(F("[PMU] AXP192 detected"));
      pmu192->setLDO2Voltage(3300); pmu192->enableLDO2();
      pmu192->setLDO3Voltage(3300); pmu192->enableLDO3();
    } else {
      delete pmu192; pmu192 = nullptr;
    }
  }
  if (!pmuReady) Serial.println(F("[PMU] Not found (continuing)"));
  delay(50);
  stageLog("PMU + rails");

  // Initialize radio according to mode
#ifdef USE_FSK_300
  {
    Serial.println(F("[RADIO] Initializing FSK modem"));
    // Disable LoRa library usage; we rely on RadioLib for FSK
    int state = fskRadio.beginFSK(CONFIG_RADIO_FREQ, FSK_TARGET_BITRATE, FSK_DEV_KHZ, FSK_RXBW_KHZ);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("[RADIO] 300 bps attempt failed (code %d), trying 600 bps fallback\n", state);
      state = fskRadio.beginFSK(CONFIG_RADIO_FREQ, FSK_FALLBACK_BITRATE, FSK_DEV_KHZ, FSK_RXBW_KHZ);
    }
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("[ERR] FSK init failed code=%d\n", state);
      while(true) { delay(1000); }
    }
    fskMode = true;
    fskRadio.setPreambleLength(16); // bytes
    fskRadio.setCRC(true);
  // Whitening not exposed for SX1278 in RadioLib 7.1.2; relying on default settings.
    Serial.println(F("[RADIO] FSK ready"));
    oledLines("FSK Ready", "300/600bps", MY_CALLSIGN, "Int 60s");
    // Immediate first frame (simple text, not AX.25 framing yet)
    frameCounter = 1;
    #ifndef FSK_RECEIVER
      char payload[192];
      size_t plen = build_fsk_payload(payload, sizeof(payload), frameCounter, 0);
      if (plen) {
        int txState = fskRadio.transmit(payload);
        if (txState == RADIOLIB_ERR_NONE) {
          Serial.printf("[TX-IMMEDIATE] FSK payload len=%u\n", (unsigned)plen);
          Serial.print("[TX-IMMEDIATE] Payload: "); Serial.print(payload);
          oledLines("FSK Sent0", "FC:1", payload, "FSK CRC");
          firstBeaconSent = true; lastSend = millis();
        } else {
          Serial.printf("[ERR] Immediate FSK TX failed code=%d\n", txState);
        }
      } else {
        Serial.println(F("[ERR] FSK payload build failed"));
      }
    #else
      // Receiver mode: put radio into continuous receive
      Serial.println(F("[RADIO] Entering continuous FSK receive"));
      int rxState = fskRadio.startReceive();
      if (rxState != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERR] startReceive failed code=%d\n", rxState);
      }
      oledLines("FSK RX", "Waiting...", MY_CALLSIGN, "");
    #endif
  }
#else
  // LoRa path (original)
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
  if (!LoRa.begin((long)(CONFIG_RADIO_FREQ * 1000000))) {
    Serial.println(F("[ERR] LoRa init failed"));
    while (true) { delay(1000); }
  }
  LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);
  LoRa.setSignalBandwidth((long)(CONFIG_RADIO_BW * 1000));
  LoRa.setSpreadingFactor(9);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  Serial.printf("[INFO] Ready @ %.3f MHz BW=%.1f kHz Power=%d dBm\n",
                CONFIG_RADIO_FREQ, CONFIG_RADIO_BW, CONFIG_RADIO_OUTPUT_POWER);
  oledLines("Ready", "Freq 145.10", MY_CALLSIGN, "Int 60s");
  frameCounter = 1;
  {
    String info = String("FC=") + frameCounter + " UPT=0ms CALL=" + MY_CALLSIGN;
    uint8_t frame[255];
    size_t flen = build_ax25_ui(frame, sizeof(frame), info);
    if (flen) {
      LoRa.beginPacket(); LoRa.write(frame, flen); LoRa.endPacket();
      Serial.printf("[TX-IMMEDIATE] AX25 len=%u infoLen=%u\n", (unsigned)flen, (unsigned)info.length());
      Serial.print("[TX-IMMEDIATE] Info: "); Serial.println(info);
      oledLines("Sent0", "FC:1", info.c_str(), "LoRa AX25");
      firstBeaconSent = true; lastSend = millis();
    } else {
      Serial.println(F("[ERR] Immediate beacon build failed"));
    }
  }
#endif

  // I2C scan (full) after rails
  stageStart();
  Serial.println(F("[I2C] Scan after rails..."));
  uint8_t devs=0;
  for (uint8_t a=1;a<127;++a){
    Wire.beginTransmission(a);
    if (Wire.endTransmission()==0){
      Serial.printf("[I2C] 0x%02X\n", a); devs++; }
  }
  if (!devs) Serial.println(F("[I2C] No devices found"));
  stageLog("I2C scan complete");

  stageStart();
  bool dispInit = initDisplaySequence();
  stageLog("Display init attempt(s)");
  if (dispInit) {
    oledLines("Booting","Disp OK", pmuReady?"PMU OK":"No PMU", MY_CALLSIGN);
  } else {
    Serial.println(F("[DISP] No display ACK at 0x3C (continuing headless)"));
  }

  // (Countdown previously here was removed intentionally.)

  // (Radio SPI bus already active earlier for both modes)
}

void loop() {
  unsigned long now = millis();
#ifdef USE_FSK_300
  #ifndef FSK_RECEIVER
    // Periodic FSK payload transmission
    if (now - lastSend >= BEACON_INTERVAL_MS) {
      lastSend = now;
      if (!firstBeaconSent) {
        frameCounter = 0;
        firstBeaconSent = true;
      }
      frameCounter++;
      char payload[192];
      size_t plen = build_fsk_payload(payload, sizeof(payload), frameCounter, now);
      if (!plen) {
        Serial.println(F("[ERR] FSK payload build failed (size)"));
        oledLines("FSK build", "FAILED");
      } else {
        int txState = fskRadio.transmit(payload);
        if (txState == RADIOLIB_ERR_NONE) {
          Serial.printf("[TX] FSK len=%u\n", (unsigned)plen);
          Serial.print("[TX] Payload: "); Serial.print(payload);
          char l2[18]; snprintf(l2, sizeof(l2), "FC:%lu", (unsigned long)frameCounter);
          oledLines("FSK Sent", l2, payload, "");
        } else {
          Serial.printf("[ERR] FSK TX fail code=%d\n", txState);
          oledLines("FSK TX", "ERR");
        }
      }
    }
  #else
    // Receiver polling: check for received packet
    // RadioLib non-blocking approach: after startReceive(), poll available() using readData if needed.
    String rx;
    int16_t state = fskRadio.readData(rx);
    if (state == RADIOLIB_ERR_NONE) {
      // Validate framing & CRC
      uint32_t fc=0, upt=0; uint16_t vbat=0, rssi=0; float temp=0; uint16_t crcCalc=0, crcField=0;
      bool ok = parse_fsk_payload(rx.c_str(), fc, upt, vbat, rssi, temp, crcCalc, crcField);
      int16_t rssiNow = fskRadio.getRSSI();
      lastFSKRSSI = (uint16_t)(rssiNow < 0 ? -rssiNow : rssiNow);
      if (ok) {
        Serial.printf("[RX] OK FC=%lu UPT=%lu Vbat=%umV RSSI=%d Temp=%.1f CRC=%04X\n", (unsigned long)fc, (unsigned long)upt, vbat, (int)rssiNow, temp, crcField);
        char l2[22]; snprintf(l2, sizeof(l2), "FC:%lu RSSI:%d", (unsigned long)fc, (int)rssiNow);
        char l3[22]; snprintf(l3, sizeof(l3), "VBAT:%umV", vbat);
        oledLines("FSK RX OK", l2, l3, "");
      } else {
        Serial.print("[RX] BAD "); Serial.println(rx);
        oledLines("FSK RX BAD", "CRC/Parse", "", "");
      }
      // Restart receive
      fskRadio.startReceive();
    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
      // Unexpected error; attempt to restart
      if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[RX] readData state=%d restarting\n", state);
        fskRadio.startReceive();
      }
    }
  #endif
#else
  if (now - lastSend >= BEACON_INTERVAL_MS) {
    lastSend = now;
    if (!firstBeaconSent) {
      // Should not happen now, but safeguard
      frameCounter = 0;
      firstBeaconSent = true;
    }
    frameCounter++;
    String info = String("FC=") + frameCounter + " UPT=" + now + "ms CALL=" + MY_CALLSIGN;
    uint8_t frame[255];
    size_t flen = build_ax25_ui(frame, sizeof(frame), info);
    if (flen == 0) {
      Serial.println(F("[ERR] Frame build failed (size)"));
      oledLines("AX25 build", "FAILED");
    } else {
      Serial.printf("[TX] AX25 len=%u infoLen=%u\n", (unsigned)flen, (unsigned)info.length());
      Serial.print("[TX] Info: "); Serial.println(info);
      LoRa.beginPacket();
      LoRa.write(frame, flen);
      LoRa.endPacket();
      // Show truncated info on display
      char l2[18]; snprintf(l2, sizeof(l2), "FC:%lu", (unsigned long)frameCounter);
      char l3[18]; strncpy(l3, info.c_str(), 17); l3[17] = '\0';
      oledLines("Sent", l2, l3, "LoRa AX25");
    }
  }
  delay(10);
#endif // USE_FSK_300
}

static const char* resetReasonToStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "OTHER";
  }
}

static void logResetInfo() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("[RST] Reason=%s (%d)\n", resetReasonToStr(r), (int)r);
}