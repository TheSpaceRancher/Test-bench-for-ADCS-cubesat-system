#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr int PIN_SPI_SCK  = 18;
constexpr int PIN_SPI_MISO = 19;
constexpr int PIN_SPI_MOSI = 23;
constexpr int PIN_RM_CS    = 5;

constexpr uint32_t SPI_FREQ = 1000000;

constexpr int PIN_X_EN = 33;
constexpr int PIN_X_IN1 = 32;
constexpr int PIN_X_IN2 = 13;

constexpr int PIN_Y_EN = 25;
constexpr int PIN_Y_IN1 = 26;
constexpr int PIN_Y_IN2 = 27;

constexpr uint16_t CYCLE_COUNT = 200;
constexpr float LSB_PER_UT = 75.0f * CYCLE_COUNT / 200.0f;

constexpr uint32_t TS_MS = 100;
constexpr uint32_t SETTLE_MS = 10;
constexpr uint32_t PWM_FREQ = 2000;
constexpr uint8_t PWM_BITS = 8;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1u;

constexpr float K_GAIN = 0.10f;
constexpr float LPF_ALPHA = 0.5f;
constexpr float DEADBAND_UT_S = 0.05f;
constexpr bool BANG_BANG = false;

constexpr float COIL_SIGN_X = +1.0f;
constexpr float COIL_SIGN_Y = +1.0f;

constexpr float COIL_VOLTAGE = 11.2f;
constexpr float L298N_MAX_CURRENT_A = 2.0f;
constexpr float COIL_RESISTANCE_OHM = 0.435903f;
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 6;
constexpr uint8_t ESPNOW_BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

float maximumSafeDuty = 0.0f;
float coilCurrentOn = 0.0f;

struct TelemetryPacket {
  uint32_t sequence;
  uint32_t timeMs;
  float field[3];
  float fieldDerivative[3];
  float currentAverage[2];
  float duty[2];
};

constexpr uint8_t REG_POLL = 0x00;
constexpr uint8_t REG_CCX = 0x04;
constexpr uint8_t REG_MX2 = 0x24;
constexpr uint8_t REG_STATUS = 0x34;
constexpr uint8_t REG_REVID = 0x36;

uint8_t rmReadRegister(uint8_t reg) {
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_RM_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_RM_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void rmReadRegisters(uint8_t reg, uint8_t *data, size_t length) {
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_RM_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (size_t i = 0; i < length; ++i) {
    data[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_RM_CS, HIGH);
  SPI.endTransaction();
}

void rmWriteRegister(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_RM_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(PIN_RM_CS, HIGH);
  SPI.endTransaction();
}

void rmWriteRegisters(uint8_t reg, const uint8_t *data, size_t length) {
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_RM_CS, LOW);
  SPI.transfer(reg & 0x7F);
  for (size_t i = 0; i < length; ++i) {
    SPI.transfer(data[i]);
  }
  digitalWrite(PIN_RM_CS, HIGH);
  SPI.endTransaction();
}

bool rmInit() {
  const uint8_t rev = rmReadRegister(REG_REVID);
  Serial.printf("RM3100 REVID = 0x%02X\n", rev);
  if (rev != 0x22) {
    return false;
  }

  const uint8_t cc[] = {
    static_cast<uint8_t>(CYCLE_COUNT >> 8), static_cast<uint8_t>(CYCLE_COUNT),
    static_cast<uint8_t>(CYCLE_COUNT >> 8), static_cast<uint8_t>(CYCLE_COUNT),
    static_cast<uint8_t>(CYCLE_COUNT >> 8), static_cast<uint8_t>(CYCLE_COUNT)
  };
  rmWriteRegisters(REG_CCX, cc, sizeof(cc));
  return true;
}

int32_t decode24(const uint8_t *data) {
  int32_t value = (static_cast<int32_t>(data[0]) << 16) |
                  (static_cast<int32_t>(data[1]) << 8) |
                  static_cast<int32_t>(data[2]);
  if (value & 0x00800000) {
    value |= 0xFF000000;
  }
  return value;
}

bool rmRead(float &bx, float &by, float &bz) {
  rmWriteRegister(REG_POLL, 0x70);

  const uint32_t start = millis();
  while ((rmReadRegister(REG_STATUS) & 0x80) == 0) {
    if (millis() - start > 50) {
      return false;
    }
    delayMicroseconds(200);
  }

  uint8_t raw[9];
  rmReadRegisters(REG_MX2, raw, sizeof(raw));

  bx = decode24(&raw[0]) / LSB_PER_UT;
  by = decode24(&raw[3]) / LSB_PER_UT;
  bz = decode24(&raw[6]) / LSB_PER_UT;
  return true;
}

void driveCoil(int pinEn, int pinA, int pinB, float command) {
  command = constrain(command, -maximumSafeDuty, maximumSafeDuty);
  if (command > 0.0f) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (command < 0.0f) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
  ledcWrite(pinEn, static_cast<uint32_t>(fabsf(command) * PWM_MAX));
}

void coilsOff() {
  driveCoil(PIN_X_EN, PIN_X_IN1, PIN_X_IN2, 0.0f);
  driveCoil(PIN_Y_EN, PIN_Y_IN1, PIN_Y_IN2, 0.0f);
}

float shape(float bdot, float sign) {
  if (fabsf(bdot) < DEADBAND_UT_S) {
    return 0.0f;
  }
  const float command = BANG_BANG
    ? ((bdot > 0.0f) ? -1.0f : 1.0f)
    : (-K_GAIN * bdot);
  return sign * constrain(command, -1.0f, 1.0f);
}

float estimateCurrentAverage(float duty) {
  return min(duty * coilCurrentOn, L298N_MAX_CURRENT_A);
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ESPNOW_BROADCAST_ADDRESS, 6);
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;
  return esp_now_add_peer(&peerInfo) == ESP_OK;
}

void sendTelemetry(const TelemetryPacket &packet) {
  const esp_err_t result = esp_now_send(
    ESPNOW_BROADCAST_ADDRESS,
    reinterpret_cast<const uint8_t *>(&packet),
    sizeof(packet));
  if (result != ESP_OK) {
    Serial.printf("ERROR: envio ESP-NOW fallo, codigo=%d\n", result);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RM_CS, OUTPUT);
  digitalWrite(PIN_RM_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_RM_CS);

  const int outputs[] = {
    PIN_X_EN, PIN_X_IN1, PIN_X_IN2,
    PIN_Y_EN, PIN_Y_IN1, PIN_Y_IN2
  };
  for (int pin : outputs) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  ledcAttach(PIN_X_EN, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_Y_EN, PWM_FREQ, PWM_BITS);
  coilsOff();

  if (!initEspNow()) {
    Serial.println("ERROR: no se pudo iniciar ESP-NOW.");
    while (true) {
      delay(1000);
    }
  }

  if (!rmInit()) {
    Serial.println("ERROR: RM3100 no responde por SPI. Revise CS, SCK, MISO, MOSI, 3V3 y GND.");
    while (true) {
      delay(1000);
    }
  }

  coilCurrentOn = COIL_VOLTAGE / COIL_RESISTANCE_OHM;
  maximumSafeDuty = constrain(
    (L298N_MAX_CURRENT_A * COIL_RESISTANCE_OHM) / COIL_VOLTAGE,
    0.0f, 1.0f);

  Serial.println("DATOS: t_ms=tiempo_ms,Bx_uT=campo_X,By_uT=campo_Y,Bz_uT=campo_Z,"
                 "dBx_uT_s=derivada_X,dBy_uT_s=derivada_Y,dBz_uT_s=derivada_Z,"
                 "ux=comando_X,uy=comando_Y,PWM_X=cuenta_X,PWM_Y=cuenta_Y,"
                 "duty_X_pct=duty_X,duty_Y_pct=duty_Y,I_X_est_prom_A=corriente_media_estimada_X,"
                 "I_Y_est_prom_A=corriente_media_estimada_Y,"
                 "I_X_est_on_A=corriente_estimada_PWM_encendido_X,"
                 "I_Y_est_on_A=corriente_estimada_PWM_encendido_Y");
}

void loop() {
  static uint32_t nextT = millis();
  static float previousB[3] = {0.0f, 0.0f, 0.0f};
  static float filteredBdot[3] = {0.0f, 0.0f, 0.0f};
  static uint32_t previousTime = 0;
  static bool firstSample = true;

  nextT += TS_MS;
  coilsOff();
  delay(SETTLE_MS);

  float field[3];
  const uint32_t measurementTime = micros();
  if (!rmRead(field[0], field[1], field[2])) {
    Serial.println("ERROR: fallo de lectura del RM3100");
  } else if (firstSample) {
    for (int i = 0; i < 3; ++i) {
      previousB[i] = field[i];
    }
    previousTime = measurementTime;
    firstSample = false;
  } else {
    const float dt = (measurementTime - previousTime) * 1e-6f;
    previousTime = measurementTime;
    if (dt > 0.0f) {
      for (int i = 0; i < 3; ++i) {
        const float rawBdot = (field[i] - previousB[i]) / dt;
        filteredBdot[i] = LPF_ALPHA * rawBdot +
                          (1.0f - LPF_ALPHA) * filteredBdot[i];
        previousB[i] = field[i];
      }

      const float commandX = constrain(shape(filteredBdot[0], COIL_SIGN_X),
                                       -maximumSafeDuty, maximumSafeDuty);
      const float commandY = constrain(shape(filteredBdot[1], COIL_SIGN_Y),
                                       -maximumSafeDuty, maximumSafeDuty);
      driveCoil(PIN_X_EN, PIN_X_IN1, PIN_X_IN2, commandX);
      driveCoil(PIN_Y_EN, PIN_Y_IN1, PIN_Y_IN2, commandY);

      const uint32_t pwmX = static_cast<uint32_t>(fabsf(commandX) * PWM_MAX);
      const uint32_t pwmY = static_cast<uint32_t>(fabsf(commandY) * PWM_MAX);
      const float dutyX = static_cast<float>(pwmX) / PWM_MAX;
      const float dutyY = static_cast<float>(pwmY) / PWM_MAX;
      static uint32_t telemetrySequence = 0;
      TelemetryPacket packet = {};
      packet.sequence = telemetrySequence++;
      packet.timeMs = millis();
      packet.field[0] = field[0];
      packet.field[1] = field[1];
      packet.field[2] = field[2];
      packet.fieldDerivative[0] = filteredBdot[0];
      packet.fieldDerivative[1] = filteredBdot[1];
      packet.fieldDerivative[2] = filteredBdot[2];
      packet.currentAverage[0] = estimateCurrentAverage(dutyX);
      packet.currentAverage[1] = estimateCurrentAverage(dutyY);
      packet.duty[0] = dutyX;
      packet.duty[1] = dutyY;
      sendTelemetry(packet);
      Serial.printf("DATOS: t_ms=%lu,Bx_uT=%.3f,By_uT=%.3f,Bz_uT=%.3f,"
                    "dBx_uT_s=%.3f,dBy_uT_s=%.3f,dBz_uT_s=%.3f,ux=%.3f,uy=%.3f,"
                    "PWM_X=%lu,PWM_Y=%lu,duty_X_pct=%.1f,duty_Y_pct=%.1f,"
                    "I_X_est_prom_A=%.3f,I_Y_est_prom_A=%.3f,"
                    "I_X_est_on_A=%.3f,I_Y_est_on_A=%.3f\n",
                    millis(), field[0], field[1], field[2],
                    filteredBdot[0], filteredBdot[1], filteredBdot[2],
                    commandX, commandY, pwmX, pwmY, dutyX * 100.0f, dutyY * 100.0f,
                    estimateCurrentAverage(dutyX),
                    estimateCurrentAverage(dutyY),
                    coilCurrentOn, coilCurrentOn);
    }
  }

  const int32_t remaining = static_cast<int32_t>(nextT - millis());
  if (remaining > 0) {
    delay(remaining);
  }
}