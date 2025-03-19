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

uint8_t buffer[3];
uint8_t temps[3];
uint8_t prevTemps[3];
uint8_t tempsToDisplay[3];


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

void setup(void) {
  tft.init();
  toggleDisplay(0);
  tft.setRotation(2);  // portrait upside down
  tft.setTextSize(1);
  tft.setTextFont(8);
  tft.fillScreen(TFT_BLACK);

  resetTemperatures();

  Serial.setTimeout(READING_TIMEOUT_MS);
  Serial.begin(9600);
}

void resetTemperatures() {
  resetKalmanFilters();
  for (uint8_t i = 0; i < 3; i++) {
    temps[i] = 0;
    prevTemps[i] = 0;
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
  float adjustedProcessNoise = kf->processNoiseCovariance * (1 + timeDiff / 1000.0);
  float predictedErrorCovariance = kf->errorCovariance + adjustedProcessNoise;

  kf->kalmanGain = predictedErrorCovariance / (predictedErrorCovariance + kf->measurementNoiseCovariance);
  kf->currentEstimate += kf->kalmanGain * ((float)currentTemp - kf->currentEstimate);
  kf->errorCovariance = (1 - kf->kalmanGain) * predictedErrorCovariance;
  kf->lastUpdate = currentTime;
  return round(kf->currentEstimate);
}

void resetKalmanFilters() {
  for (uint8_t i = 0; i < 3; i++) {
    prepareKalmanFilter(&kalmanFilters[i]);
  }
}

bool updateNeeded = false;

void loop() {
  while (!Serial.available()) {
    if (isBacklightEnabled && !areValuesGreyedOut && (millis() - lastUpdate) >= GREY_OUT_DELAY_MS) {
      areValuesGreyedOut = true;
      drawValues();
    }
  }
  while (Serial.available()) {
    switch (Serial.read()) {
      case DATA_MODE:
        if (Serial.readBytes(buffer, 3) == 3) {
          memcpy(temps, buffer, 3);
          prepareUpdate();
          sendAck();
        } else {
          sendNak();
        }
        break;
      case CMD_REPEAT:
        prepareUpdate();
        sendAck();
        break;
      case CMD_RESET:
        reset();
        sendAck();
        break;
      default:
        sendNak();
    }
  }
  if (updateNeeded) {
    update();
  }
}

inline void sendAck() {
  writeByte(ACK);
}

inline void sendNak() {
  writeByte(NAK);
}

inline void writeByte(char byte) {
  Serial.write(byte);
  Serial.flush();
}

void update() {
  drawValues();
  updateNeeded = false;
}

void prepareUpdate() {
  for (uint8_t i = 0; i < 3; i++) {
    tempsToDisplay[i] = updateKalmanFilter(&kalmanFilters[i], temps[i]);
    prevTemps[i] = tempsToDisplay[i];
  }
  areValuesGreyedOut = false;
  updateNeeded = true;
  lastUpdate = millis();
}

void drawValues() {
  // cpu
  drawSingleValue(tempsToDisplay[0], 89, 0);
  // nvme
  drawSingleValue(tempsToDisplay[1], 70, 1);
  // gpu
  drawSingleValue(tempsToDisplay[2], 82, 2);

  if (!isBacklightEnabled) {
    toggleDisplay(1);
  }
}

inline void drawSingleValue(uint8_t value, uint8_t threshold, uint8_t lineIndex) {
  uint16_t verticalPosition = 2 + 81 * lineIndex;
  if (value == 0) {
    tft.setTextColor(TFT_ALMOSTBLACK, TFT_BLACK);
    tft.drawString("00", 14, verticalPosition);
    return;
  }

  uint16_t color;
  if (areValuesGreyedOut) {
    color = TFT_ERRORGRAY;
  } else if (value >= threshold) {
    color = TFT_RED;
  } else {
    color = (lineIndex & 1) ? TFT_DARKGREY : TFT_LIGHTGREY;
  }

  char str[3] = { 0 };

  if (value < 10) {
    str[0] = '0';
    str[1] = '0' + value;
  } else if (value < 100) {
    str[0] = '0' + (value / 10);
    str[1] = '0' + (value % 10);
  } else {
    str[0] = '0' + (value / 100);
    str[1] = '0' + ((value / 10) % 10);
    str[2] = '0' + (value % 10);
  }
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(str, 14, verticalPosition);
}

void reset() {
  toggleDisplay(0);
  tft.fillScreen(TFT_BLACK);
  resetTemperatures();
}

inline void toggleDisplay(bool value) {
  digitalWrite(TFT_BL, value);
  isBacklightEnabled = value;
}
