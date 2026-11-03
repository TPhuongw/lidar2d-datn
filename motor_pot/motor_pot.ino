/*
 * motor_pot.ino - Chay 42CME06 + CL42C bang Arduino UNO, KHONG can laptop.
 * Toc do chinh bang bien tro, chieu quay bang cong tac gat.
 *
 * Noi day (giu nguyen phan driver nhu motor_test.ino):
 *   UNO D9  -> PUL+        PUL- -> UNO GND
 *   UNO D8  -> DIR+        DIR- -> UNO GND
 *   Driver CL42C: jumper muc tin hieu 5 V, DIP 3200 xung/vong, SW6 off (vong kin), SW7 off (PUL/DIR)
 *
 * Phan them:
 *   Bien tro 10k (chiet ap tuyen tinh, loai B10K):
 *     chan ngoai 1 -> UNO 5V,  chan ngoai 2 -> UNO GND,  chan GIUA -> UNO A0
 *   Cong tac gat chieu quay (tuy chon): mot chan -> D7, chan kia -> GND
 *     (khong lap cong tac thi D7 tu keo len, quay mot chieu co dinh)
 *   LED bao trang thai: dung LED san tren board (D13)
 *
 * Nguon cho UNO (chon 1):
 *   a) Cu sac dien thoai 5 V cam vao cong USB cua UNO (don gian nhat)
 *   b) Module ha ap 24 V -> 9 V, ra cam vao jack DC (hoac chan VIN) cua UNO
 *   KHONG cam thang 24 V vao jack DC / VIN: on ap tren UNO chi chiu 7-12 V.
 *
 * An toan khi bat nguon: neu bien tro KHONG o vi tri 0 luc bat, dong co se KHONG quay
 * cho toi khi van bien tro ve 0 mot lan (LED nhay nhanh bao dang cho).
 *
 * Van con nhan lenh Serial '?' (115200) neu cam laptop de xem trang thai.
 * CODE THU NGHIEM, CHUA CHAY TREN PHAN CUNG.
 */
const int PIN_PUL = 9;
const int PIN_DIR = 8;
const int PIN_SW_DIR = 7;
const int PIN_POT = A0;
const int PIN_LED = 13;

const long PULSES_PER_REV = 3200;  // PHAI khop DIP SW1-SW4 cua driver
const float MAX_RPS = 5.0;         // bien tro het co = 5 vong/s (he thong chi can 2-3)
const float MIN_RPS = 0.1;         // duoi muc nay coi nhu dung (vung chet quanh 0)
const float ACCEL = 1.0;           // vong/s^2: tang/giam toc tu tu de khong mat bam

float target = 0, current = 0, potFilt = 0;
bool dirCW = true;
bool armed = false;                 // chi chay sau khi bien tro da ve 0 it nhat 1 lan
unsigned long tLast = 0, tPrint = 0;

void applySpeed(float rps) {
  long f = (long)(rps * PULSES_PER_REV);
  if (f < 31) noTone(PIN_PUL);      // tone() tren UNO khong tao duoc duoi 31 Hz
  else tone(PIN_PUL, f);
}

float readPotRps() {
  // loc trung binh truot de bo nhieu ADC, roi lam tron buoc 0.05 vong/s cho on dinh
  potFilt += 0.1 * (analogRead(PIN_POT) - potFilt);
  float rps = potFilt / 1023.0 * MAX_RPS;
  rps = round(rps * 20) / 20.0;
  return (rps < MIN_RPS) ? 0 : rps;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SW_DIR, INPUT_PULLUP);
  dirCW = digitalRead(PIN_SW_DIR);
  digitalWrite(PIN_DIR, dirCW ? HIGH : LOW);
  potFilt = analogRead(PIN_POT);
  Serial.println(F("motor_pot: bien tro A0 = toc do, D7 = chieu. Lenh '?' xem trang thai."));
}

void loop() {
  unsigned long now = millis();
  if (now - tLast < 20) return;      // cap nhat moi 20 ms
  float dt = (now - tLast) / 1000.0;
  tLast = now;

  float potRps = readPotRps();
  if (!armed && potRps == 0) armed = true;
  bool wantCW = digitalRead(PIN_SW_DIR);

  // doi chieu: giam toc ve 0, dao DIR, roi tang toc lai
  if (wantCW != dirCW) {
    target = 0;
    if (current == 0) { dirCW = wantCW; digitalWrite(PIN_DIR, dirCW ? HIGH : LOW); }
  } else {
    target = armed ? potRps : 0;
  }

  float prev = current;
  float step = ACCEL * dt;
  if (current < target) current = min(target, current + step);
  else if (current > target) current = max(target, current - step);
  if (current != prev) applySpeed(current);

  // LED: sang = dang quay, tat = dung, nhay nhanh = cho van bien tro ve 0
  digitalWrite(PIN_LED, !armed ? ((now / 100) % 2) : (current > 0));

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '?') {
      Serial.print(F("bien tro ")); Serial.print(potRps);
      Serial.print(F(" | hien tai ")); Serial.print(current);
      Serial.print(F(" vong/s | xung ")); Serial.print((long)(current * PULSES_PER_REV));
      Serial.print(F(" Hz | chieu ")); Serial.print(dirCW ? 1 : 2);
      Serial.println(armed ? "" : " | CHO VAN BIEN TRO VE 0");
    }
  }
}
