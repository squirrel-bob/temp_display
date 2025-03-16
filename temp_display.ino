/*
 Awailable fonts: GLCD (font 1) and 2, 4, 6, 7, 8
*/
#include <SPI.h>
#include <TFT_eSPI.h>
#include <BluetoothSerial.h>

#define DEBUG false

#define TFT_ALMOSTBLACK 0x0861 /*  10,  10,  10 */
#define TFT_ERRORGRAY 0x3183   /*  50,  25,  25 */

#define READING_TIMEOUT_MS 50
#define GREY_OUT_DELAY_MS 5000

#define CMD_RESET 0x1E
#define CMD_REPEAT 0x07
#define DATA_MODE 0xD0
#define ACK 0x06
#define NAK 0x15

uint8_t temps[3] = { 0, 0, 0 };
uint8_t prevDisplayedTemps[3] = { 0, 0, 0 };

uint8_t tempsBuffer[3];

bool areValuesGreyedOut = true;
bool isBacklightEnabled = false;

unsigned long lastUpdate = millis();

typedef struct {
  bool isInitialized;
  float processNoiseCovariance;
  float measurementNoiseCovariance;
  float errorCovariance;
  float kalmanGain;
  float currentEstimate;
  unsigned long lastUpdate;
} KalmanFilter;

KalmanFilter kalmanFilters[3] = {};

TFT_eSPI tft = TFT_eSPI();

#if DEBUG
BluetoothSerial SerialBT;
#endif

void setup(void) {
  tft.init();
  toggleDisplayForce(0);
  tft.setRotation(2);  // portrait upside down
  tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);

  resetTemperatures();

  Serial.setTimeout(READING_TIMEOUT_MS);
  Serial.begin(9600);
#if DEBUG
  SerialBT.begin("ESP32_Debug");
#endif
}

void resetTemperatures() {
  resetKalmanFilters();
  for (uint8_t i = 0; i < 3; i++) {
    temps[i] = 0;
    prevDisplayedTemps[i] = 0;
  }
}

void prepareKalmanFilter(KalmanFilter* kf) {
  kf->isInitialized = false;
  kf->currentEstimate = 0.0;
  kf->errorCovariance = 1.0;
  kf->processNoiseCovariance = 0.5;
  kf->measurementNoiseCovariance = 0.5;
}

uint8_t updateKalmanFilter(KalmanFilter* kf, uint8_t currentTemp) {
  if (currentTemp == 0) {
    if (kf->isInitialized) {
      prepareKalmanFilter(kf);
    }
    return 0;
  }
  if (!kf->isInitialized) {
    kf->currentEstimate = currentTemp;
    kf->isInitialized = true;
    kf->lastUpdate = millis();
    return currentTemp;
  }

  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - kf->lastUpdate;
  float adjustedProcessNoiseCovariance = kf->processNoiseCovariance * (1 + timeDiff / 1000.0);

  kf->errorCovariance = kf->errorCovariance + adjustedProcessNoiseCovariance;
  kf->kalmanGain = kf->errorCovariance / (kf->errorCovariance + kf->measurementNoiseCovariance);
  kf->currentEstimate = kf->currentEstimate + kf->kalmanGain * ((float)currentTemp - kf->currentEstimate);
  kf->errorCovariance = (1 - kf->kalmanGain) * kf->errorCovariance;
  kf->lastUpdate = currentTime;
  return round(kf->currentEstimate);
}

void resetKalmanFilters() {
  for (uint8_t i = 0; i < 3; i++) {
    prepareKalmanFilter(&kalmanFilters[i]);
  }
}

void loop() {
  while (!Serial.available()) {
    if (isBacklightEnabled && !areValuesGreyedOut && (millis() - lastUpdate) >= GREY_OUT_DELAY_MS) {
      btPrintln("* [timeout]");
      areValuesGreyedOut = true;
      drawValues(prevDisplayedTemps);
    }
  }
  switch (Serial.read()) {
    case DATA_MODE:
      btPrintln("<- DATA_MODE");
      if (receiveTemperatures(temps)) {
        sendAck();
        update(temps);
      } else {
        sendNak();
      }
      break;
    case CMD_REPEAT:
      btPrintln("<- CMD_REPEAT");
      sendAck();
      update(temps);
      break;
    case CMD_RESET:
      btPrintln("<- CMD_RESET");
      sendAck();
      reset();
      break;
    default:
      sendNak();
  }
}

bool receiveTemperatures(uint8_t* targetTemps) {
  btPrintln("* [receive]");
  if (Serial.readBytes(tempsBuffer, 3) != 3) {
    return false;
  }
  btPrintln("<- 3 bytes");
  memcpy(targetTemps, tempsBuffer, 3);
  return true;
}

inline void sendAck() {
  btPrintln("---> ACK");
  writeByte(ACK);
}

inline void sendNak() {
  btPrintln("---> NAK");
  writeByte(NAK);
}

inline void writeByte(char byte) {
  discardSerialInput();
  Serial.write(byte);
  Serial.flush();
}

inline void discardSerialInput() {
  btPrintln("* [discarding input]");
  while (Serial.available()) Serial.read();
}

void update(uint8_t* inputTemps) {
  btPrintln("* [update]");
  uint8_t displayedTemps[3];
  for (uint8_t i = 0; i < 3; i++) {
    displayedTemps[i] = updateKalmanFilter(&kalmanFilters[i], inputTemps[i]);
  }
  memcpy(prevDisplayedTemps, displayedTemps, sizeof(displayedTemps));
  areValuesGreyedOut = false;
  drawValues(displayedTemps);
  lastUpdate = millis();
}

void drawValues(uint8_t* inputTemps) {
  btPrintln("* [draw]");
  // cpu
  drawSingleValue(inputTemps[0], 89, 0);
  // nvme
  drawSingleValue(inputTemps[1], 70, 1);
  // gpu
  drawSingleValue(inputTemps[2], 82, 2);

  toggleDisplay(1);
}

inline void drawSingleValue(uint8_t value, uint8_t threshold, uint8_t lineIndex) {
  uint16_t verticalPosition = 2 + 81 * lineIndex;
  if (value == 0) {
    tft.setTextColor(TFT_ALMOSTBLACK, TFT_BLACK);
    tft.drawString("00", 14, verticalPosition, 8);
    return;
  }
  char str[3];
  uint16_t color = (lineIndex % 2 == 0) ? TFT_LIGHTGREY : TFT_DARKGREY;
  if (areValuesGreyedOut) {
    color = TFT_ERRORGRAY;
  } else if (value >= threshold) {
    color = TFT_RED;
  }
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(itoa(value, str, 10), 14, verticalPosition, 8);
}

void reset() {
  discardSerialInput();
  toggleDisplayForce(0);
  tft.fillScreen(TFT_BLACK);
  resetTemperatures();
}

inline void toggleDisplay(bool value) {
  if (value != isBacklightEnabled) {
    toggleDisplayForce(value);
  }
}

inline void toggleDisplayForce(bool value) {
  digitalWrite(TFT_BL, value);
  isBacklightEnabled = value;
}

inline void btPrintln(String string) {
#if DEBUG
  SerialBT.println(string);
#endif
}
