#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define RELAY_ACTIVE_LEVEL   HIGH
#define RELAY_INACTIVE_LEVEL LOW

bool displayCommandActivity = false;
bool debugMode = false;

// Dance Sequencer
int dancePhase = 0; // 0=Idle, 1=Prep, 2=Left, 3=Right, 4=Re-Center, 5=Up, 6=Down, 7=CenterEL
unsigned long danceTimer = 0;
float danceStartAz = 0;
float danceStartEl = 0;

// —— OLED Setup ——
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Relay pins
//const int relayAzLeft  = 13;
//const int relayAzRight = 14;
//const int relayElDown  = 21;
//const int relayElUp    = 47;

// Relay pins AFTER REWIRE
const int relayAzLeft  = 47;
const int relayAzRight = 21;
const int relayElDown  = 14;
const int relayElUp    = 13;

// GS-232 state
double targetAz = NAN, targetEl = NAN;
const float tolAz = 2.0f, tolEl = 2.0f;
bool azMove = false, elMove = false;
bool gpredictConnected = false;
String inputCommand;
unsigned long lastUpdateMs = 0;
const unsigned long updateInterval = 500;

// Elevation smoothing
#define SMOOTH_WINDOW 5
float elHistory[SMOOTH_WINDOW] = {0};
int elIndex = 0;
void pushEl(float v) { elHistory[elIndex] = v; elIndex = (elIndex+1) % SMOOTH_WINDOW; }
float getSmoothedEl() { float sum = 0; for (int i = 0; i < SMOOTH_WINDOW; i++) sum += elHistory[i]; return sum / SMOOTH_WINDOW; }

// Directv Mount Elevation Offset
float elevationOffset = 42.0f;

// Convert BNO055 heading (-180..180) -> 0..360 + offset
float readHeading(const sensors_event_t &ev) {
  float h = ev.orientation.x;
  if (h < 0) h += 360;
  h += 90; // mount offset
  if (h >= 360) h -= 360;
  return h;
}

// ——— Hard-coded calibration (optional) ———
const bool USE_HARDCODED = false;
const adafruit_bno055_offsets_t defaultOffsets = { -33, -70, -29, 188, 160, -795, 0, 2, 0, 1000, 666 };
// -33, -70, -29, 188, 160, -795, 0, 2, 0, 1000, 666 
// -32, -59, -32, 216, -63, -654, -1, 2, 0, 1000, 746 

// ——— EEPROM calibration storage ———
const uint32_t EEPROM_MAGIC  = 0x424E4F43;
const int      MAGIC_ADDR    = 0;
const int      OFFSETS_ADDR  = MAGIC_ADDR + sizeof(EEPROM_MAGIC);

void applyOffsets(const adafruit_bno055_offsets_t &o) {
  bno.setSensorOffsets(o);
  EEPROM.put(MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(OFFSETS_ADDR, o);
  EEPROM.commit();
}

bool loadEepromOffsets(adafruit_bno055_offsets_t &o) {
  uint32_t m;
  EEPROM.get(MAGIC_ADDR, m);
  if (m != EEPROM_MAGIC) return false;
  EEPROM.get(OFFSETS_ADDR, o);
  bno.setSensorOffsets(o);
  return true;
}

void printCalStats() {
  adafruit_bno055_offsets_t o;
  bno.getSensorOffsets(o);
  Serial.println("=== CALIBRATION OFFSETS ===");
  Serial.print("Accel Off: "); Serial.print(o.accel_offset_x); Serial.print(','); Serial.print(o.accel_offset_y); Serial.print(','); Serial.println(o.accel_offset_z);
  Serial.print("Mag   Off: ");   Serial.print(o.mag_offset_x);   Serial.print(','); Serial.print(o.mag_offset_y);   Serial.print(','); Serial.println(o.mag_offset_z);
  Serial.print("Gyro  Off: ");  Serial.print(o.gyro_offset_x);  Serial.print(','); Serial.print(o.gyro_offset_y);  Serial.print(','); Serial.println(o.gyro_offset_z);
  Serial.print("Accel Rad: "); Serial.println(o.accel_radius);
  Serial.print("Mag   Rad: ");   Serial.println(o.mag_radius);
  Serial.println("===========================");
}

void calibrateAndStore() {
  Serial.println("🔄 CALIBRATION MODE — rotate until 3/3/3/3");
  uint8_t sys=0, gyr=0, acc=0, mag=0;

  while (sys < 3 || gyr < 3 || acc < 3 || mag < 3) {
    bno.getCalibration(&sys, &gyr, &acc, &mag);
    Serial.print("CALIB: SYS="); Serial.print(sys);
    Serial.print(" GYR=");    Serial.print(gyr);
    Serial.print(" ACC=");    Serial.print(acc);
    Serial.print(" MAG=");    Serial.println(mag);

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("CALIBRATION MODE");
    display.setCursor(0, 12);  display.print("SYS="); display.print(sys);
    display.setCursor(64,12);  display.print("GYR="); display.print(gyr);
    display.setCursor(0, 24);  display.print("ACC="); display.print(acc);
    display.setCursor(64,24);  display.print("MAG="); display.print(mag);
    display.display();

    delay(300);
  }

  Serial.println("🎉 Calibrated — fetching offsets");
  adafruit_bno055_offsets_t o;
  bno.getSensorOffsets(o);
  applyOffsets(o);
  Serial.println("💾 Offsets written to EEPROM");

  Serial.println("Use these for defaultOffsets:");
  Serial.print("const adafruit_bno055_offsets_t defaultOffsets = { ");
  Serial.print(o.accel_offset_x); Serial.print(", ");
  Serial.print(o.accel_offset_y); Serial.print(", ");
  Serial.print(o.accel_offset_z); Serial.print(", ");
  Serial.print(o.mag_offset_x);   Serial.print(", ");
  Serial.print(o.mag_offset_y);   Serial.print(", ");
  Serial.print(o.mag_offset_z);   Serial.print(", ");
  Serial.print(o.gyro_offset_x);  Serial.print(", ");
  Serial.print(o.gyro_offset_y);  Serial.print(", ");
  Serial.print(o.gyro_offset_z);  Serial.print(", ");
  Serial.print(o.accel_radius);   Serial.print(", ");
  Serial.print(o.mag_radius);     Serial.println(" };");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("CALIBRATION DONE!");
  display.display();
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  Wire.begin(10, 9);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();

  if (!bno.begin()) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("BNO055 FAIL!");
    display.display();
    while (1);
  }
  delay(100);

  if (USE_HARDCODED) {
    bno.setSensorOffsets(defaultOffsets);
  }
  else {
    adafruit_bno055_offsets_t eoffs;
    if (!loadEepromOffsets(eoffs)) {
      Serial.println("No EEPROM offsets → calibrating now");
      calibrateAndStore();
    }
  }

  delay(50);
  bno.setExtCrystalUse(true);
  delay(500);
  sensors_event_t discard; bno.getEvent(&discard);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("BNO055 NDOF");
  display.display();

  pinMode(relayAzLeft, OUTPUT);
  pinMode(relayAzRight, OUTPUT);
  pinMode(relayElUp, OUTPUT);
  pinMode(relayElDown, OUTPUT);
  digitalWrite(relayAzLeft, RELAY_INACTIVE_LEVEL);
  digitalWrite(relayAzRight, RELAY_INACTIVE_LEVEL);
  digitalWrite(relayElUp, RELAY_INACTIVE_LEVEL);
  digitalWrite(relayElDown, RELAY_INACTIVE_LEVEL);

  const int tms = 300;
  display.clearDisplay(); display.setCursor(0,0); display.println("UP");    display.display();
  digitalWrite(relayElUp, HIGH);    delay(tms); digitalWrite(relayElUp, LOW);    delay(100);
  display.clearDisplay(); display.setCursor(0,0); display.println("DOWN");  display.display();
  digitalWrite(relayElDown, HIGH);  delay(tms); digitalWrite(relayElDown, LOW);  delay(100);
  display.clearDisplay(); display.setCursor(0,0); display.println("LEFT");  display.display();
  digitalWrite(relayAzLeft, HIGH);  delay(tms); digitalWrite(relayAzLeft, LOW);  delay(100);
  display.clearDisplay(); display.setCursor(0,0); display.println("RIGHT"); display.display();
  digitalWrite(relayAzRight, HIGH); delay(tms); digitalWrite(relayAzRight, LOW); delay(100);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Ready");
  display.display();
  Serial.println("Ready! Type HELP for list of commands.");
  delay(150);
}

void updateMovement() {
  sensors_event_t ev; bno.getEvent(&ev);
  float currAz = readHeading(ev);
  float rawEl  = ev.orientation.y + elevationOffset;
  pushEl(rawEl);
  float currEl = getSmoothedEl();

  if (azMove) {
    float diff = targetAz - currAz;
    if (diff > 180) diff -= 360;
    else if (diff < -180) diff += 360;
    if (fabs(diff) > tolAz) {
      digitalWrite(relayAzRight, diff > 0 ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
      digitalWrite(relayAzLeft,  diff < 0 ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    } else {
      digitalWrite(relayAzLeft, RELAY_INACTIVE_LEVEL);
      digitalWrite(relayAzRight, RELAY_INACTIVE_LEVEL);
      azMove = false;
    }
  }

  if (elMove) {
    float diff2 = targetEl - currEl;
    if (fabs(diff2) > tolEl) {
      digitalWrite(relayElUp,   diff2 > 0 ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
      digitalWrite(relayElDown, diff2 < 0 ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    } else {
      digitalWrite(relayElUp,    RELAY_INACTIVE_LEVEL);
      digitalWrite(relayElDown,  RELAY_INACTIVE_LEVEL);
      elMove = false;
    }
  }
}

void updateOrientationDisplay() {
  static float lastTargetAz = NAN;
  static float lastTargetEl = NAN;

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  sensors_event_t ev; bno.getEvent(&ev);
  float az = readHeading(ev);
  pushEl(ev.orientation.y + elevationOffset);
  float el = getSmoothedEl();

  // Store last targets if move flags are active
  if (azMove) lastTargetAz = targetAz;
  if (elMove) lastTargetEl = targetEl;

  display.clearDisplay();
  display.setCursor(0, 0);   display.print("AZ:"); display.print(az,1);
  display.setCursor(0, 8);   display.print("EL:"); display.print(el,1);

  // Debug to Serial Monitor
  if (debugMode) {
    Serial.print("DEBUG | RawX:"); Serial.print(ev.orientation.x, 1);
    Serial.print(" | CalcAZ:"); Serial.print(az, 1);
    Serial.print(" | Calib[S:"); Serial.print(sys);
    Serial.print(" G:"); Serial.print(gyro);
    Serial.print(" A:"); Serial.print(accel);
    Serial.print(" M:"); Serial.print(mag);
    Serial.println("]");
  }

  // Spacer line
  display.setCursor(0, 16);  display.println("");

  // Show TAZ/TEL if movement is requested
  if (!isnan(lastTargetAz)) {
    display.setCursor(0, 24); display.print("TAZ:"); display.print(lastTargetAz,1);
  }
  if (!isnan(lastTargetEl)) {
    display.setCursor(0, 32); display.print("TEL:"); display.print(lastTargetEl,1);
  }

  // Calibration + Connection status
  display.setCursor(70, 0);  display.print("S:");  display.print(sys);
  display.setCursor(70, 8);  display.print("G:");  display.print(gyro);
  display.setCursor(70, 16); display.print("A:");  display.print(accel);
  display.setCursor(70, 24); display.print("M:");  display.print(mag);
  if (gpredictConnected) {
    display.setCursor(0, 56);
    display.print("Connected");
  }
  
  if (displayCommandActivity) {
    display.setCursor(SCREEN_WIDTH - 6, SCREEN_HEIGHT - 8);  // bottom right
    display.print("D");
    displayCommandActivity = false;  // clear after showing once
  }

  display.display();
  updateMovement();
}


void handleCommand(const String &cmd) {
  gpredictConnected = true;
  displayCommandActivity = true;

  if (cmd.startsWith("AZ")) {
    if (cmd.length() > 2 && (isDigit(cmd.charAt(2)) || cmd.charAt(2) == ' ')) {
      targetAz = cmd.substring(2).toFloat();
      azMove = true;
      Serial.print("RPRT 0\r");
    } else {
      sensors_event_t ev; bno.getEvent(&ev);
      char buf[16];
      snprintf(buf, sizeof(buf), "AZ %.1f", readHeading(ev));
      Serial.print(buf); Serial.print("\r");
    }
  }
    else if (cmd.equalsIgnoreCase("HELP")) {
    Serial.println("\n=== Available Commands ===");
    Serial.println("AZ         → Get current azimuth");
    Serial.println("AZxxx.x    → Set target azimuth (e.g. AZ123.4)");
    Serial.println("EL         → Get current elevation");
    Serial.println("ELxxx.x    → Set target elevation (e.g. EL45.6)");
    Serial.println("P          → Get current AZ and EL (position)");
    Serial.println("HOME       → Move to Home position (AZ180 EL45)");
    Serial.println("DANCE      → Perform calibration movements");
    Serial.println("DEBUG      → Toggle raw sensor debug output");
    Serial.println("CALIBRATE  → Start calibration and store offsets");
    Serial.println("CALSTATS   → Print current calibration values");
    Serial.println("SA         → Stop azimuth movement");
    Serial.println("SE         → Stop elevation movement");
    Serial.println("VE         → Get firmware version");
    Serial.println("Q or q     → Disconnect from Gpredict");
    Serial.println("HELP       → Show this help message");
    Serial.print("RPRT 0\r");
  }
  else if (cmd.equalsIgnoreCase("HOME")) {
    targetAz = 180.0;
    targetEl = 45.0;
    azMove = true;
    elMove = true;
    Serial.print("RPRT 0\r");
  }
  else if (cmd.equalsIgnoreCase("DANCE")) {
    dancePhase = 1; 
    Serial.println("Starting Dance... Type SA or SE to abort.");
    Serial.print("RPRT 0\r");
  }
  else if (cmd.equalsIgnoreCase("DEBUG")) {
    debugMode = !debugMode;
    Serial.print(debugMode ? "DEBUG ON\r" : "DEBUG OFF\r");
  }
  else if (cmd.startsWith("EL")) {
    if (cmd.length() > 2 && (isDigit(cmd.charAt(2)) || cmd.charAt(2) == ' ')) {
      targetEl = cmd.substring(2).toFloat();
      elMove = true;
      Serial.print("RPRT 0\r");
    } else {
      sensors_event_t ev; bno.getEvent(&ev);
      float raw = ev.orientation.y + elevationOffset; pushEl(raw);
      char buf[16];
      snprintf(buf, sizeof(buf), "EL %.1f", getSmoothedEl());
      Serial.print(buf); Serial.print("\r");
    }
  }
  else if (cmd.equalsIgnoreCase("P")) {
    sensors_event_t ev; bno.getEvent(&ev);
    float heading = readHeading(ev);
    float raw = ev.orientation.y + elevationOffset; pushEl(raw);
    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f %.1f", heading, getSmoothedEl());
    Serial.print(buf); Serial.print("\r");
  }
  else if (cmd.equalsIgnoreCase("CALSTATS")) {
    printCalStats();
    Serial.print("RPRT 0\r");
  }
  else if (cmd.equalsIgnoreCase("CALIBRATE")) {
    Serial.println("Re-running calibration...");
    calibrateAndStore();
    Serial.print("RPRT 0\r");
  }
  else if (cmd.equalsIgnoreCase("SA")) {
    digitalWrite(relayAzLeft, RELAY_INACTIVE_LEVEL);
    digitalWrite(relayAzRight, RELAY_INACTIVE_LEVEL);
    Serial.print("RPRT 0\r");
  }
  else if (cmd.equalsIgnoreCase("SE")) {
    digitalWrite(relayElUp, RELAY_INACTIVE_LEVEL);
    digitalWrite(relayElDown, RELAY_INACTIVE_LEVEL);
    Serial.print("RPRT 0\r");
  }
  else if (cmd.equalsIgnoreCase("VE")) {
    Serial.print("VEAdamESP32v1.0\r");
  }
  else if (cmd.equalsIgnoreCase("Q") || cmd.equalsIgnoreCase("q")) {
    gpredictConnected = false;
  }
  else {
    Serial.print("RPRT -1\r");
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\r' || c=='\n') {
      if (inputCommand.length()) {
        inputCommand.trim();
        handleCommand(inputCommand);
        inputCommand = "";
      }
    } else inputCommand += c;
  }

  if (millis() - lastUpdateMs >= updateInterval) {
    updateOrientationDisplay();
    lastUpdateMs = millis();
  }

  // --- Dance Sequencer ---
  if (dancePhase > 0) {
    // If we are currently moving, keep resetting the timer so it only starts counting once stopped
    if (azMove || elMove) {
      danceTimer = millis();
    }
    
    // Check if current move is done AND we have paused for 3 seconds
    if (!azMove && !elMove && millis() - danceTimer > 3000) {
       sensors_event_t ev; bno.getEvent(&ev);
       float curAz = readHeading(ev);
       float curEl = getSmoothedEl(); // use smoothed

       switch (dancePhase) {
         case 1: // Prep: Store start position
           danceStartAz = curAz;
           danceStartEl = curEl;
           Serial.println("Dance: Going Left -30");
           targetAz = danceStartAz - 30;
           if (targetAz < 0) targetAz += 360;
           azMove = true;
           dancePhase = 2;
           break;
           
         case 2: // Finished Left, Go Right (+30 from start)
           Serial.println("Dance: Going Right +30");
           targetAz = danceStartAz + 30;
           if (targetAz >= 360) targetAz -= 360;
           azMove = true;
           dancePhase = 3;
           break;

         case 3: // Finished Right, Return to Az Center
           Serial.println("Dance: Centering Az");
           targetAz = danceStartAz;
           azMove = true;
           dancePhase = 4;
           break;

         case 4: // Finished Az, Start El Up (+20)
           Serial.println("Dance: Going Up +20");
           targetEl = danceStartEl + 20;
           if (targetEl > 90) targetEl = 90;
           elMove = true;
           dancePhase = 5;
           break;
           
         case 5: // Finished Up, Go Down (-20 from start)
           Serial.println("Dance: Going Down -20");
           targetEl = danceStartEl - 20;
           if (targetEl < 0) targetEl = 0;
           elMove = true;
           dancePhase = 6;
           break;

         case 6: // Finished Down, Return to El Center
           Serial.println("Dance: Centering El");
           targetEl = danceStartEl;
           elMove = true;
           dancePhase = 7;
           break;

         case 7: // Done
           Serial.println("Dance Complete!");
           dancePhase = 0; 
           break;
       }
    }
  }
}
