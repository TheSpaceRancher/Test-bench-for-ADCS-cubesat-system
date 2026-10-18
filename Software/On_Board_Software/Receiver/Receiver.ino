#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr uint8_t WIFI_CHANNEL = 6;
constexpr float L298N_MAX_CURRENT_A = 2.0f;

struct TelemetryPacket {
  uint32_t sequence;
  uint32_t timeMs;
  float field[3];
  float fieldDerivative[3];
  float currentAverage[2];
  float duty[2];
};

bool packetReady = false;
TelemetryPacket pendingPacket = {};
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;

void onDataReceived(const esp_now_recv_info_t *, const uint8_t *data, int length) {
  if (data == nullptr || length != sizeof(TelemetryPacket)) {
    return;
  }
  portENTER_CRITICAL(&packetMux);
  memcpy(&pendingPacket, data, sizeof(pendingPacket));
  packetReady = true;
  portEXIT_CRITICAL(&packetMux);
}

void stopWithError(const char *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    stopWithError("ERROR: no se pudo configurar el canal Wi-Fi");
  }

  if (esp_now_init() != ESP_OK) {
    stopWithError("ERROR: no se pudo iniciar ESP-NOW");
  }
  esp_now_register_recv_cb(onDataReceived);

  Serial.printf("Receiver ESP-NOW listo en canal=%u. USB serial a 115200 baudios.\n",
                WIFI_CHANNEL);
}

void loop() {
  TelemetryPacket packet = {};
  bool hasPacket = false;
  portENTER_CRITICAL(&packetMux);
  if (packetReady) {
    packet = pendingPacket;
    packetReady = false;
    hasPacket = true;
  }
  portEXIT_CRITICAL(&packetMux);

  if (hasPacket) {
    const float currentX = constrain(packet.currentAverage[0],
                                     0.0f, L298N_MAX_CURRENT_A);
    const float currentY = constrain(packet.currentAverage[1],
                                     0.0f, L298N_MAX_CURRENT_A);
    const float dutyX = constrain(packet.duty[0], 0.0f, 1.0f);
    const float dutyY = constrain(packet.duty[1], 0.0f, 1.0f);
    Serial.printf(
      "DATOS:t_ms=%lu,Bx_uT=%.3f,By_uT=%.3f,Bz_uT=%.3f,"
      "dBx_uT_s=%.3f,dBy_uT_s=%.3f,dBz_uT_s=%.3f,"
      "I_X_est_prom_A=%.3f,I_Y_est_prom_A=%.3f,"
      "duty_X=%.4f,duty_Y=%.4f,seq=%lu\n",
      packet.timeMs, packet.field[0], packet.field[1], packet.field[2],
      packet.fieldDerivative[0], packet.fieldDerivative[1],
      packet.fieldDerivative[2], currentX, currentY, dutyX, dutyY,
      packet.sequence);
  }
}
