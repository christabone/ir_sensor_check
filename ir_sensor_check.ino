/***************************************************************************
 *  SEN0158 / PAJ7025R3  –  ESP32  Clean Diagnostic  v2.1
 *  Baud 115 200   •   I²C 100 kHz   •   Poll 20 Hz
 ***************************************************************************/
#include <Wire.h>

/* ---------- USER SETTINGS ---------- */
#define SDA_PIN        21
#define SCL_PIN        22
#define I2C_FREQ_HZ    100000          // you can try 400 000 later
#define SENSOR_ADDR    0x58
#define SERIAL_BAUD    115200
#define HEARTBEAT_LED  2               // set −1 to disable onboard LED blink
#define LOOP_DELAY_MS  50              // 20 Hz
/* ----------------------------------- */

struct IRPoint { uint16_t x, y; };

/* forward decls */
void i2cScan();
bool readProductID(uint16_t &pid);
bool runInitSequence();
bool readFrame(IRPoint p[4]);

/* ---------- SETUP ---------- */
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(2000);   // give you time to open Serial Monitor
  Serial.println("\n========  SEN0158 / PAJ7025R3  –  ESP32 Diagnostic  ========");

  if (HEARTBEAT_LED >= 0) pinMode(HEARTBEAT_LED, OUTPUT);

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQ_HZ);
  Serial.printf("I²C ready  SDA=%d  SCL=%d  @%lu Hz\n",
                SDA_PIN, SCL_PIN, (unsigned long)I2C_FREQ_HZ);

  i2cScan();

  uint16_t pid;
  bool pidOK  = readProductID(pid);
  bool initOK = runInitSequence();

  Serial.printf("\nSummary: PID %s  |  INIT %s\n",
                pidOK  ? "OK"   : "FAIL",
                initOK ? "OK"   : "FAIL");
  Serial.println("Polling 0x36 every 50 ms …");
}

/* ---------- LOOP ---------- */
void loop()
{
  static uint32_t t0 = 0;
  if (millis() - t0 >= LOOP_DELAY_MS) {
    t0 = millis();

    IRPoint pts[4];
    if (readFrame(pts)) {
      Serial.printf("%8lu ms  ",
                    (unsigned long)millis());
      for (int i = 0; i < 4; ++i)
        Serial.printf("P%d=(%4u,%4u)%c",
                      i, pts[i].x, pts[i].y, (i < 3 ? ' ' : '\n'));
    } else {
      Serial.println("Frame read error!");
    }

    if (HEARTBEAT_LED >= 0)
      digitalWrite(HEARTBEAT_LED, !digitalRead(HEARTBEAT_LED));
  }
}

/* ───────── HELPER FUNCTIONS ───────── */

void i2cScan()
{
  Serial.println("\n[SCAN] Detecting I²C devices …");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  • 0x%02X%s\n",
                    a, a == SENSOR_ADDR ? "  <– SEN0158" : "");
      ++found;
    }
  }
  if (!found) Serial.println("  (none)");
}

bool readProductID(uint16_t &pid)
{
  Serial.println("\n[PID] Reading 0x02/0x03 …");

  Wire.beginTransmission(SENSOR_ADDR);   // Bank‑0
  Wire.write(0xEF); Wire.write(0x00);
  if (Wire.endTransmission()) return false;

  Wire.beginTransmission(SENSOR_ADDR);   // pointer → 0x02
  Wire.write(0x02);
  if (Wire.endTransmission(false)) return false;

  if (Wire.requestFrom(SENSOR_ADDR, (uint8_t)2) != 2) return false;
  uint8_t lo = Wire.read(), hi = Wire.read();
  pid = (hi << 8) | lo;
  Serial.printf("  Product‑ID = 0x%04X\n", pid);
  return pid == 0x7025;
}

bool runInitSequence()
{
  Serial.println("\n[INIT] Sending 6‑byte sequence + extra 0x30‑08 …");
  const uint8_t seq[][2] = {
    {0x30,0x01}, {0x30,0x08}, {0x06,0x90},
    {0x08,0xC0}, {0x1A,0x40}, {0x33,0x33},
    {0x30,0x08}               // extra "go live"
  };
  bool ok = true;
  for (auto &cmd : seq) {
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(cmd[0]); Wire.write(cmd[1]);
    if (Wire.endTransmission()) ok = false;
    delay(10);
  }
  delay(120);
  return ok;
}

/* single stop‑start read – proven working */
bool readFrame(IRPoint p[4])
{
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x36);                    // data block start
  if (Wire.endTransmission()) return false;

  if (Wire.requestFrom(SENSOR_ADDR, (uint8_t)16) != 16) return false;
  uint8_t buf[16];  for (int i = 0; i < 16; ++i) buf[i] = Wire.read();

  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t n = 1 + i * 3;
    uint8_t s = buf[n + 2];
    p[i].x = ((s & 0x30) << 4) | buf[n];
    p[i].y = ((s & 0xC0) << 2) | buf[n + 1];
  }
  return true;
}