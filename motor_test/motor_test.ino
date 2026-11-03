/*
 * motor_test.ino - Chay thu 42CME06 + CL42C bang Arduino UNO (muc logic 5 V, noi thang).
 *
 * Noi day (CL42C dat muc tin hieu 5 V; noi kieu "cathode chung"):
 *   UNO D9  -> PUL+        PUL- -> UNO GND
 *   UNO D8  -> DIR+        DIR- -> UNO GND
 *   ENA+/ENA- : de trong (driver enable mac dinh)
 *   Nguon 24 V -> +Vdc / GND cua CL42C. GND nguon KHONG can noi voi GND UNO (ngo vao cach ly quang).
 *
 * DIP CL42C: chon 3200 xung/vong theo bang in tren driver; SW6 = off (closed-loop); SW7 = off (PUL/DIR).
 *
 * Lenh qua Serial Monitor (115200, "Newline"):
 *   r 1.5   -> quay 1.5 vong/s (tang/giam toc tu tu)
 *   r 0     -> dung (giam toc)
 *   d       -> dao chieu (chi khi dang dung)
 *   ?       -> in trang thai
 * Xung tao bang tone() (song vuong 50%), du cho toc do 0.1 .. 10 vong/s o 3200 xung/vong.
 * CODE THU NGHIEM, CHUA CHAY TREN PHAN CUNG.
 */
const int PIN_PUL = 9;
const int PIN_DIR = 8;
const long PULSES_PER_REV = 3200;   // PHAI khop DIP SW1-SW4
const float ACCEL = 1.0;            // vong/s^2 (tang toc 1 vong/s moi giay)
const float MAX_RPS = 10.0;

float target = 0, current = 0;
bool dirCW = true;
unsigned long tLast = 0;

void applySpeed(float rps) {
  long f = (long)(rps * PULSES_PER_REV);
  if (f < 31) noTone(PIN_PUL);      // tone() khong tao duoc < 31 Hz tren UNO
  else tone(PIN_PUL, f);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);
  pinMode(PIN_PUL, OUTPUT);
  Serial.println(F("motor_test: lenh 'r <vong/s>', 'r 0', 'd', '?'"));
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s.startsWith("r")) {
      target = constrain(s.substring(1).toFloat(), 0, MAX_RPS);
      Serial.print(F("Dat toc do: ")); Serial.print(target); Serial.println(F(" vong/s"));
    } else if (s == "d") {
      if (current == 0) { dirCW = !dirCW; digitalWrite(PIN_DIR, dirCW ? HIGH : LOW);
                          Serial.println(dirCW ? F("Chieu 1") : F("Chieu 2")); }
      else Serial.println(F("Dung lai truoc khi dao chieu (r 0)"));
    } else if (s == "?") {
      Serial.print(F("Hien tai ")); Serial.print(current); Serial.print(F(" vong/s, dich "));
      Serial.print(target); Serial.print(F(", xung ")); Serial.print((long)(current * PULSES_PER_REV));
      Serial.println(F(" Hz"));
    }
  }
  // doc toc (ramp) moi 20 ms
  if (millis() - tLast >= 20) {
    float dt = (millis() - tLast) / 1000.0; tLast = millis();
    float step = ACCEL * dt;
    float prev = current;
    if (current < target) current = min(target, current + step);
    else if (current > target) current = max(target, current - step);
    if (current != prev) applySpeed(current);
  }
}
