// ================= OLED DIAG (standalone, no other peripherals) ================
// Boot sequence:
//   1. Scan I2C bus on SDA=GPIO1/SCL=GPIO3 @100kHz
//   2. If empty: swap SDA/SCL (Wire 3,1) and scan again -> tells you if the
//      wires are swapped
//   3. If device found: init SSD1306 at that address, draw checkerboard 3s,
//      then "I2C OK!" text
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 d(128, 64, &Wire, -1, 400000);

static int scanBus(uint8_t *found, int maxN) {
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X ACK\n", a);
      if (n < maxN) found[n++] = a;
    }
  }
  return n;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== OLED DIAG (SSD1306 on GPIO1/3) ===");

  Serial.println("[1] Wire.begin(SDA=1, SCL=3), 100kHz scan:");
  Wire.begin(1, 3);
  Wire.setClock(100000);
  uint8_t found[8];
  int n = scanBus(found, 8);

  if (n == 0) {
    Serial.println("[2] bus empty -> trying SWAPPED pins (SDA=3, SCL=1)...");
    Wire.end();
    Wire.begin(3, 1);
    Wire.setClock(100000);
    n = scanBus(found, 8);
    if (n > 0) {
      Serial.println("  >>> YOUR SDA/SCL WIRES ARE SWAPPED! <<<");
      Serial.println("  >>> (firmware now uses swapped pins so the screen will work) <<<");
    }
  }

  if (n == 0) {
    Serial.println("[3] NOTHING on the bus either way. Checklist:");
    Serial.println("    - VDD = 3.3V? measure it with a meter");
    Serial.println("    - GND common with board?");
    Serial.println("    - CS->GND, DC->GND, RES->3.3V (the three fixed-level pins!)");
    Serial.println("    - R8 shorted / R3->R1 moved (SPI->I2C mod)");
    Serial.println("    - dupont wire continuity (very common failure!)");
    Serial.println("=== DIAG END ===");
    return;
  }

  uint8_t addr = 0x3C;
  bool addrKnown = false;
  for (int i = 0; i < n; i++) {
    if (found[i] == 0x3C || found[i] == 0x3D) { addr = found[i]; addrKnown = true; }
  }
  if (!addrKnown && n > 0) addr = found[0];
  Serial.printf("[3] device found, init SSD1306 @ 0x%02X...\n", addr);
  bool ok = d.begin(SSD1306_SWITCHCAPVCC, addr, true, false);
  Serial.printf("    begin() = %d\n", ok);

  for (int y = 0; y < 64; y++)
    for (int x = 0; x < 128; x++)
      if (((x >> 2) + (y >> 2)) & 1) d.drawPixel(x, y, SSD1306_WHITE);
  d.display();
  Serial.println("[4] checkerboard pattern shown for 3s...");
  delay(3000);

  d.clearDisplay();
  d.setTextSize(2);
  d.setTextColor(SSD1306_WHITE);
  d.setCursor(0, 0);
  d.println("I2C OK!");
  d.setTextSize(1);
  d.setCursor(0, 36);
  d.printf("addr 0x%02X\n", addr);
  d.display();
  Serial.println("[5] done - screen should show 'I2C OK!'");
  Serial.println("=== DIAG END ===");
}

void loop() {}
