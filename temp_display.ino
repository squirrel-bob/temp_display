/*
 Awailable fonts: GLCD (font 1) and 2, 4, 6, 7, 8
*/
#include <SPI.h>
#include <TFT_eSPI.h>

#define READING_WINDOW_MS 1000
#define GREY_OUT_DELAY_MS 5000
#define TURN_OFF_DELAY_MS 15000
#define SERIAL_BUFFER_SIZE 10

TFT_eSPI tft = TFT_eSPI();

typedef struct {
  bool initialized;
  float Q;
  float R;
  float P;
  float K;
  float current_estimate;
} KalmanFilter;

KalmanFilter cpu_temp_kf;
KalmanFilter gpu_temp_kf;
KalmanFilter nvme_temp_kf;

void prepare_kf(KalmanFilter* kf) {
  kf->initialized = false;
  kf->current_estimate = 0.0;
  kf->P = 1.0;
  kf->Q = 0.5;
  kf->R = 0.5;
}

uint8_t update_kf(KalmanFilter* kf, uint8_t current_temp) {
  if (!kf->initialized) {
    kf->current_estimate = current_temp;
    kf->initialized = true;
  } else if (current_temp > kf->current_estimate) {
    kf->current_estimate = current_temp;
    kf->P += kf->Q;
  } else {
    kf->P = kf->P + kf->Q;
    kf->K = kf->P / (kf->P + kf->R);
    kf->current_estimate = kf->current_estimate + kf->K * ((float)current_temp - kf->current_estimate);
    kf->P = (1 - kf->K) * kf->P;
  }
  return round(kf->current_estimate);
}

void setup(void) {
  tft.init();
  toggle_display(0);
  tft.setRotation(2);
  tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);

  prepare_kf(&cpu_temp_kf);
  prepare_kf(&gpu_temp_kf);
  prepare_kf(&nvme_temp_kf);

  Serial.begin(115200);
}

uint8_t prev_cpu_temp = 0;
uint8_t prev_gpu_temp = 0;
uint8_t prev_nvme_temp = 0;
unsigned long last_update = millis();
bool grey_out = true;

void update(char* buffer) {
  char cpu_temp_char[4];
  char gpu_temp_char[4];
  char nvme_temp_char[4];

  strncpy(cpu_temp_char, buffer, 3);
  strncpy(gpu_temp_char, buffer + 3, 3);
  strncpy(nvme_temp_char, buffer + 6, 3);

  uint8_t cpu_temp = update_kf(&cpu_temp_kf, atoi(cpu_temp_char));
  uint8_t gpu_temp = update_kf(&gpu_temp_kf, atoi(gpu_temp_char));
  uint8_t nvme_temp = update_kf(&nvme_temp_kf, atoi(nvme_temp_char));

  grey_out = false;

  if (temperature_did_change(cpu_temp, gpu_temp, nvme_temp)) {
    draw_values(cpu_temp, gpu_temp, nvme_temp);
    prev_cpu_temp = cpu_temp;
    prev_gpu_temp = gpu_temp;
    prev_nvme_temp = nvme_temp;
  }

  toggle_display(1);
  last_update = millis();
}

inline bool temperature_did_change(uint8_t cpu_temp, uint8_t gpu_temp, uint8_t nvme_temp) {
  return cpu_temp != prev_cpu_temp || gpu_temp != prev_gpu_temp || nvme_temp != prev_nvme_temp;
}

char buffer[SERIAL_BUFFER_SIZE] = {};
unsigned long packet_receipt_start_time = 0;

void loop() {
  int8_t receive_index = -1;

  while (1) {
    if (Serial.available()) {
      char incomingByte = Serial.read();
      if (receive_index == -1) {
        if (is_data_packet_start_byte(incomingByte)) {
          packet_receipt_start_time = millis();
          receive_index = 0;
          continue;
        } else if (is_previous_packet_repeat_byte(incomingByte)) {
          receive_index = 1;
          break;
        } else if (is_stop_byte(incomingByte)) {
          stop();
          return;
        }
        break;
      }
      if (is_data_packet_end_byte(incomingByte) || receive_index >= SERIAL_BUFFER_SIZE - 1) {
        buffer[receive_index] = '\0';
        break;
      }
      // received next byte from data packet
      buffer[receive_index] = incomingByte;
      receive_index++;
    }

    if (reading_window_expired()) {
      // serial read timeout, starting over
      receive_index = -1;
      break;
    }
  }

  if (receive_index > 0) {
    send_ack();
    update(buffer);
  } else if (grey_out_delay_elapsed()) {
    grey_out = true;
  } else if (turn_off_delay_elapsed()) {
    stop();
  }
}

inline bool is_data_packet_start_byte(char byte) {
  return byte == 0x02;
}

inline bool is_previous_packet_repeat_byte(char byte) {
  return byte == 0x01;
}

inline bool is_stop_byte(char byte) {
  return byte == 0x04;
}

inline bool is_data_packet_end_byte(char byte) {
  return byte == 0x03;
}

inline bool reading_window_expired() {
  return (millis() - packet_receipt_start_time) > READING_WINDOW_MS;
}

inline bool grey_out_delay_elapsed() {
  return (millis() - last_update) >= GREY_OUT_DELAY_MS;
}

inline bool turn_off_delay_elapsed() {
  return (millis() - last_update) >= TURN_OFF_DELAY_MS;
}

inline void send_ack() {
  Serial.write(0x06);
}

void stop() {
  toggle_display(0);
  tft.fillScreen(TFT_BLACK);
}

void draw_values(uint8_t cpu_temp, uint8_t gpu_temp, uint8_t nvme_temp) {
  char buffer[SERIAL_BUFFER_SIZE];
  tft.setTextColor(grey_out ? TFT_DARKGREY : (cpu_temp >= 89 ? TFT_RED : TFT_LIGHTGREY), TFT_BLACK);
  tft.drawString(itoa(cpu_temp, buffer, 10), 14, 2, 8);

  tft.setTextColor(grey_out ? TFT_DARKGREY : (nvme_temp >= 70 ? TFT_GOLD : TFT_DARKGREY), TFT_BLACK);
  tft.drawString(itoa(gpu_temp, buffer, 10), 14, 83, 8);
  
  tft.setTextColor(grey_out ? TFT_DARKGREY : (gpu_temp >= 82 ? TFT_RED : TFT_LIGHTGREY), TFT_BLACK);
  tft.drawString(itoa(nvme_temp, buffer, 10), 14, 164, 8);
}

bool is_backlight_enabled = 1;

void toggle_display(uint8_t value) {
  if (value != is_backlight_enabled) {
    digitalWrite(TFT_BL, value);
    is_backlight_enabled = value;
  }
}
