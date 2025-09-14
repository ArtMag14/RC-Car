#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ---- Pins (your wiring) ----
const int PIN_IN1   = 27;  // IBT-4 IN1  (PWM forward)
const int PIN_IN2   = 14;  // IBT-4 IN2  (PWM reverse)
const int PIN_SERVO = 18;  // servo signal

Servo servoX;

// Joystick packet
typedef struct {
  int joyX;
  int joyY;
} struct_message;

// ===== Calibration (from your measurements) =====
const int X_LEFT   = 0;
const int X_CENTER = 2960;  // steering center
const int X_RIGHT  = 4095;
const int X_DB     = 60;    // servo deadband around center (tweak 40–120)

const int Y_DOWN   = 0;
const int Y_CENTER = 2850;  // throttle center
const int Y_UP     = 4095;
const int Y_DB     = 180;   // motor deadband (increase if it creeps)

// ---------- calibrated servo mapping (piecewise) ----------
int angleFromX(int x) {
  if (x < X_LEFT)  x = X_LEFT;
  if (x > X_RIGHT) x = X_RIGHT;

  // deadband => exact center
  if (abs(x - X_CENTER) <= X_DB) return 90;

  if (x > X_CENTER) {
    return map(x, X_CENTER + X_DB, X_RIGHT, 90, 180);
  } else {
    return map(x, X_LEFT, X_CENTER - X_DB, 0, 90);
  }
}

// ---------- calibrated throttle mapping (piecewise) ----------
void setMotorFromJoyY(int y) {
  if (y < Y_DOWN) y = Y_DOWN;
  if (y > Y_UP)   y = Y_UP;

  // Neutral zone → stop
  if (abs(y - Y_CENTER) <= Y_DB) {
    analogWrite(PIN_IN1, 0);
    analogWrite(PIN_IN2, 0);
    return;
  }

  // analogWrite default is 8-bit (0..255)
  if (y > Y_CENTER) {
    // Forward on IN1
    int duty = map(y, Y_CENTER + Y_DB, Y_UP, 0, 255);
    if (duty < 0) duty = 0; if (duty > 255) duty = 255;
    analogWrite(PIN_IN2, 0);
    analogWrite(PIN_IN1, duty);
  } else {
    // Reverse on IN2
    int duty = map(y, Y_DOWN, Y_CENTER - Y_DB, 255, 0);
    if (duty < 0) duty = 0; if (duty > 255) duty = 255;
    analogWrite(PIN_IN1, 0);
    analogWrite(PIN_IN2, duty);
  }
}

// NEW callback signature for ESP-NOW (core 3.x)
void onDataRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (len != sizeof(struct_message)) return;

  struct_message in;
  memcpy(&in, data, sizeof(in));

  // ----- Servo from X -----
  int angle = constrain(angleFromX(in.joyX), 0, 180);
  servoX.write(angle);

  // ----- Motor from Y -----
  setMotorFromJoyY(in.joyY);

  // Debug (optional)
  if (info) {
    const uint8_t* m = info->src_addr;
    Serial.printf("From %02X:%02X:%02X:%02X:%02X:%02X  X=%d -> angle=%d  Y=%d\n",
                  m[0], m[1], m[2], m[3], m[4], m[5],
                  in.joyX, angle, in.joyY);
  }
}

void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  analogWrite(PIN_IN1, 0);
  analogWrite(PIN_IN2, 0);

  // Servo
  servoX.attach(PIN_SERVO);
  servoX.write(90); // center on boot

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);

  // Hardware notes:
  // - Tie IBT-4 R_EN & L_EN to VCC (enable both halves)
  // - Share grounds (ESP32 GND ↔ IBT-4 GND ↔ Servo GND)
  // - Motor uses its own supply on PWR+/PWR- (not from ESP32)
}

void loop() {
  delay(20);
}
