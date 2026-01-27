#include <TinyGPS++.h>
#include <Wire.h>
// #include <Adafruit_MMC56x3.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>

/* ===================== FORWARD DECLARATIONS ===================== */
void readGPS();
void stopMotors();
void updateGpsLed();
void gpsLedBlink();
void setCurrentType();
void manualControl(int throttle, int yaw);
void autonomousHold();
void printCurrentDebug();
void driveMotors(int throttle, int yaw);
void updateCurrentEstimate(double lat, double lon, bool motorsStopped);
float readCompass();
float headingError(float desired, float current);

/* ===================== PIN DEFINITIONS (STM32) ===================== */

#define PPM_PIN           PA0     // Receiver connection PPM
#define ESC_PORT_PIN      PB6     // ESC connection for port - babord
#define ESC_STARBOARD_PIN PB7     // ESC connection for starboard - tribord
#define LED_GPS           PC13    // LED GPS status - no LED, no fix, blinking LED GPS working, LED steady GPS valid fix

#define GPS_SERIAL SerialGPS        // PA9 TX / PA10 RX

/* ===================== GLOBAL OBJECTS ===================== */
HardwareSerial SerialGPS(USART1);

/* ===================== CONSTANTS ===================== */

#define GPS_BAUD 115200           // valid for fast GPS, most GPS work at 9600 bauds
#define PPM_CHANNELS 8            // There is a total mas of 8 channels 0 to 7 in the array

#define RC_MIN      1000
#define RC_MAX      2000
#define RC_NEUTRAL  1500

#define SYNC_THRESHOLD   3000
#define MIN_VALID_PULSE   600
#define MAX_VALID_PULSE  2500

#define GPS_HDOP_MAX  1.0         // if HDOP is greater than 1, FIX is lousy

#define CURRENT_SAMPLE_TIME_MS 5000
#define CURRENT_LOCK_THRESHOLD 3.0

// === MAG CALIBRATION (FILL AFTER CALIBRATION) ===

float magOffsetX = -12.4;
float magOffsetY = 8.9;
float magScaleX  = 1.08;
float magScaleY  = 0.94;

// === LOCAL DECLINATION ===

#define MAG_DECLINATION_DEG 2.5  // degrees, positive for east declination for Paris area



/* ===================== OBJECTS ===================== */

TinyGPSPlus gps;
//Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
Adafruit_HMC5883_Unified mmc = Adafruit_HMC5883_Unified(12345);

Servo escPort;
Servo escStarboard;

/* ===================== GLOBALS ===================== */

volatile uint16_t ppm[PPM_CHANNELS];
volatile uint8_t ppmIndex = 0;

double targetLat = 0;
double targetLon = 0;

unsigned long ledTimer = 0;
bool ledState = false;

enum Mode { MODE_AUTONOMOUS, MODE_MANUAL };
Mode currentMode = MODE_AUTONOMOUS;

// Current estimation
double driftLat = 0, driftLon = 0;
unsigned long driftTimer = 0;
float currentDir = 0;
float currentStrength = 0;
bool currentValid = false;

// Control parameters
float HOLD_RADIUS  = 2.0;
float DIST_GAIN    = 1.5;
float YAW_GAIN     = 1.4;
float MAX_THRUST   = 320;
float MAX_YAW      = 180;
float MIN_THRUST   = 120;

enum Type { CALM_WATERS, LOW_CURRENT, STRONG_CURRENT };
Type currentType = CALM_WATERS;

/* ===================== PPM ISR ===================== */

void ppmISR() {
  static uint32_t lastPPM = 0;
  uint32_t now = micros();
  uint32_t diff = now - lastPPM;
  lastPPM = now;

  if (diff > 0x80000000UL) return;

  if (diff > SYNC_THRESHOLD) {
    ppmIndex = 0;
  } else if (ppmIndex < PPM_CHANNELS) {
    if (diff >= MIN_VALID_PULSE && diff <= MAX_VALID_PULSE) {
      ppm[ppmIndex++] = diff;
    }
  }
}

/* ===================== SETUP ===================== */

void setup() {
  pinMode(LED_GPS, OUTPUT);
  digitalWrite(LED_GPS, HIGH); // LED off (PC13 inversée)

  Serial.begin(115200);       // USB debug
  GPS_SERIAL.begin(GPS_BAUD); // GPS UART

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  analogWriteResolution(16);   // Timer resolution

  escPort.attach(ESC_PORT_PIN);
  escStarboard.attach(ESC_STARBOARD_PIN);
  stopMotors();

  Wire.begin();
  if (!mmc.begin()) {
    while (1);
  }

  // Wait for GPS fix
  while (!gps.location.isValid() || gps.hdop.hdop() > GPS_HDOP_MAX) {
    readGPS();
    gpsLedBlink();
  }

  digitalWrite(LED_GPS, LOW); // FIX OK

  targetLat = gps.location.lat();
  targetLon = gps.location.lng();
}

/* ===================== LOOP ===================== */

void loop() {
  readGPS();
  updateGpsLed();

  uint16_t ch1 = ppm[0]; // yaw
  uint16_t ch3 = ppm[2]; // throttle
  uint16_t ch5 = ppm[4]; // mode
  uint16_t ch6 = ppm[5]; // current type

  if (ch6 < 1300) currentType = CALM_WATERS;
  else if (ch6 > 1700) currentType = STRONG_CURRENT;
  else currentType = LOW_CURRENT;

  setCurrentType();

  currentMode = (ch5 > 1500) ? MODE_MANUAL : MODE_AUTONOMOUS;

  if (currentMode == MODE_MANUAL) {
    manualControl(ch3, ch1);
  } else {
    autonomousHold();
  }

  static unsigned long dbg = 0;
  if (millis() - dbg > 2000) {
    dbg = millis();
    printCurrentDebug();
  }
}

/* ===================== MODES ===================== */

void manualControl(int throttleRC, int yawRC) {
  int throttle = map(throttleRC, RC_MIN, RC_MAX, -500, 500);
  int yaw = map(yawRC, RC_MIN, RC_MAX, -500, 500);
  driveMotors(throttle, yaw);
}

void autonomousHold() {
  if (!gps.location.isValid()) {
    stopMotors();
    return;
  }

  double lat = gps.location.lat();
  double lon = gps.location.lng();

  float distance = TinyGPSPlus::distanceBetween(lat, lon, targetLat, targetLon);
  bool motorsStopped = (distance < HOLD_RADIUS);
  updateCurrentEstimate(lat, lon, motorsStopped);

  if (distance < HOLD_RADIUS) {
    stopMotors();
    return;
  }

  float bearing = TinyGPSPlus::courseTo(lat, lon, targetLat, targetLon);
  float heading = readCompass();
  float desiredBearing = bearing;

  if (currentValid) {
    float upstream = fmod(currentDir + 180.0, 360.0);
    float weight = constrain(currentStrength / 3.0, 0.0, 1.0);
    desiredBearing = (1.0 - weight) * bearing + weight * upstream;
  }

  float error = headingError(desiredBearing, heading);
  if (abs(error) < 5) error = 0;

  int yawCmd = constrain(error * YAW_GAIN, -MAX_YAW, MAX_YAW);
  int thrust = constrain(distance * DIST_GAIN * 80, MIN_THRUST, MAX_THRUST);

  if (currentValid && currentStrength > CURRENT_LOCK_THRESHOLD) {
    thrust = MAX_THRUST;
    error = headingError(currentDir + 180.0, heading);
    yawCmd = constrain(error * YAW_GAIN, -MAX_YAW, MAX_YAW);
  }

  driveMotors(thrust, yawCmd);
}

/* ===================== HELPERS ===================== */

void readGPS() {
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }
}


float readCompass() {
  sensors_event_t event;
  mmc.getEvent(&event);

  // Raw values
  float mx = event.magnetic.x;
  float my = event.magnetic.y;

  // ===== HARD-IRON OFFSET =====
  mx -= magOffsetX;
  my -= magOffsetY;

  // ===== SOFT-IRON SCALE =====
  mx *= magScaleX;
  my *= magScaleY;

  // ===== HEADING =====
  float heading = atan2(my, mx) * 180.0 / PI;

  // ===== DECLINATION =====
  heading += MAG_DECLINATION_DEG;

  // Normalize
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;

  return heading;
}


float headingError(float target, float current) {
  float e = target - current;
  while (e > 180) e -= 360;
  while (e < -180) e += 360;
  return e;
}

void driveMotors(int throttle, int yaw) {
  escPort.writeMicroseconds(constrain(RC_NEUTRAL + throttle + yaw, 1000, 2000));
  escStarboard.writeMicroseconds(constrain(RC_NEUTRAL + throttle - yaw, 1000, 2000));
}

void stopMotors() {
  escPort.writeMicroseconds(RC_NEUTRAL);
  escStarboard.writeMicroseconds(RC_NEUTRAL);
}

void updateCurrentEstimate(double lat, double lon, bool motorsStopped) {
  if (!motorsStopped) {
    driftTimer = 0;
    currentValid = false;
    return;
  }

  if (driftTimer == 0) {
    driftLat = lat;
    driftLon = lon;
    driftTimer = millis();
  }

  if (millis() - driftTimer >= CURRENT_SAMPLE_TIME_MS) {
    currentStrength = TinyGPSPlus::distanceBetween(driftLat, driftLon, lat, lon);
    currentDir = TinyGPSPlus::courseTo(driftLat, driftLon, lat, lon);
    currentValid = true;
    driftTimer = 0;
  }
}

void setCurrentType() {
  if (currentType == CALM_WATERS) {
    HOLD_RADIUS = 2.0; DIST_GAIN = 1.5; YAW_GAIN = 1.4; MAX_THRUST = 300; MIN_THRUST = 120;
  } else if (currentType == LOW_CURRENT) {
    HOLD_RADIUS = 3.0; DIST_GAIN = 1.8; YAW_GAIN = 1.2; MAX_THRUST = 360; MIN_THRUST = 100;
  } else {
    HOLD_RADIUS = 3.5; DIST_GAIN = 2.0; YAW_GAIN = 1.0; MAX_THRUST = 420; MIN_THRUST = 120;
  }
}

void printCurrentDebug() {
  if (!currentValid) return;
  Serial.print("CURRENT | ");
  Serial.print(currentStrength, 2);
  Serial.print(" m | Dir ");
  Serial.println(currentDir, 1);
}

/* ===================== GPS LED ===================== */

void updateGpsLed() {
  if (gps.location.isValid() && gps.hdop.hdop() <= GPS_HDOP_MAX) {
    digitalWrite(LED_GPS, LOW);
  } else {
    gpsLedBlink();
  }
}

void gpsLedBlink() {
  if (millis() - ledTimer > 500) {
    ledTimer = millis();
    ledState = !ledState;
    digitalWrite(LED_GPS, ledState ? LOW : HIGH);
  }
}
