/*
 * index_flag_test.ino - Thu moc 0 do bang LA CHAN (tam thiec) di qua cam bien khe, Arduino UNO.
 * Do do lap lai cua moc 0 do tinh theo SO BUOC dong co (khong phai theo thoi gian).
 *
 * Khac cac sketch truoc: xung buoc tao bang Timer1 + ngat, DEM DUOC tung buoc (tone() khong dem duoc).
 *
 * Noi day:
 *   Driver: D9 -> PUL+, D8 -> DIR+, PUL-/DIR- -> GND UNO (giu nguyen nhu cu)
 *   Cam bien khe: TIM -> 2x100 ohm noi tiep -> 5V, TRANG -> GND, XANH DUONG -> GND,
 *                 XANH LA -> D2 (keo len noi INPUT_PULLUP)
 *   La chan che tia -> transistor tat -> D2 len CAO -> canh LEN = mep truoc la chan = moc 0 do.
 *   Cam bien phai lui ra NGOAI mep dia 180 vach: tia chi bi la chan che, khong thay vach dia.
 *
 * Lenh Serial Monitor (115200, Newline):
 *   r 1     -> quay 1 vong/s (tang/giam toc tu tu), r 0 -> dung
 *   d       -> dao chieu (khi dung)
 *   z       -> xoa thong ke, lay lan qua moc ke tiep lam goc tham chieu
 *   ?       -> trang thai
 * Moi lan qua moc in: so buoc/vong (phai = 3200), do lech vi tri moc so voi tham chieu (buoc va do),
 * va khoang lech min..max tu luc 'z' = DO LAP LAI cua moc 0 do.
 * Moi dong MOC 0 co 't=<micros>' = thoi diem moc theo dong ho UNO (do chinh xac us),
 *   lidar_map_pc.py dung so nay de khu do tre USB.
 * CODE THU NGHIEM.
 */
const int PIN_PUL = 9;       // PB1
const int PIN_DIR = 8;
const int PIN_IDX = 2;       // INT0
const long STEPS_PER_REV = 3200;   // PHAI khop DIP driver
const float ACCEL = 1.0;           // vong/s^2
const float MAX_RPS = 5.0;

// ---- phat xung + dem buoc (Timer1, prescaler 8 -> 2 MHz) ----
volatile long stepAbs = 0;         // so buoc tuyet doi (co dau theo chieu)
volatile int8_t stepDir = 1;
volatile bool pulHigh = false;
volatile bool running = false;
volatile uint16_t nextOcr = 60000;

ISR(TIMER1_COMPA_vect) {
  if (!running) return;
  pulHigh = !pulHigh;
  if (pulHigh) { PORTB |= _BV(1); stepAbs += stepDir; }   // canh len = 1 buoc
  else         { PORTB &= ~_BV(1); }
  OCR1A = nextOcr;                 // doi toc do an toan ngay sau khi so sanh khop
}

// ---- moc 0 do ----
volatile unsigned long lastIdxUs = 0, lockUs = 2000000UL, revUs = 0;
volatile long lastIdxSteps = 0, idxSteps = 0, revSteps = 0;
volatile bool newIdx = false, haveRev = false;

void idxISR() {
  unsigned long t = micros();
  if (lastIdxUs && (t - lastIdxUs) < lockUs) return;     // bo canh rung o mep la chan
  long s = stepAbs;                                       // trong ngat: Timer1 khong chen vao duoc
  haveRev = (lastIdxUs != 0);
  if (haveRev) { revSteps = s - lastIdxSteps; revUs = t - lastIdxUs; }
  idxSteps = s; lastIdxSteps = s; lastIdxUs = t; newIdx = true;
}

float target = 0, current = 0;
bool dirCW = true;
unsigned long tLast = 0;
bool haveRef = false;
long refSteps = 0, devMin = 0, devMax = 0, revCount = 0;

void applySpeed(float rps) {
  float f = rps * STEPS_PER_REV;                 // xung/s
  if (f < 16) {                                  // duoi muc Timer1 tao duoc -> dung
    running = false; pulHigh = false; PORTB &= ~_BV(1);
    return;
  }
  uint16_t ocr = (uint16_t)(1e6 / f - 1);        // 2 MHz / (2 * f) - 1
  noInterrupts(); nextOcr = ocr; if (!running) { OCR1A = ocr; TCNT1 = 0; running = true; } interrupts();
  // khoa chong rung moc 0: nua vong
  unsigned long lk = (unsigned long)(0.5e6 / rps);
  if (lk > 2000000UL) lk = 2000000UL;
  noInterrupts(); lockUs = lk; interrupts();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUL, OUTPUT); digitalWrite(PIN_PUL, LOW);
  pinMode(PIN_DIR, OUTPUT); digitalWrite(PIN_DIR, HIGH);
  pinMode(PIN_IDX, INPUT_PULLUP);
  TCCR1A = 0; TCCR1B = _BV(WGM12) | _BV(CS11);   // CTC, chia 8
  OCR1A = 60000; TCNT1 = 0; TIMSK1 = _BV(OCIE1A);
  attachInterrupt(digitalPinToInterrupt(PIN_IDX), idxISR, RISING);
  Serial.println(F("index_flag_test | lenh: r <vong/s>, r 0, d, z (xoa thong ke), ?"));
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s.startsWith("r")) {
      target = constrain(s.substring(1).toFloat(), 0, MAX_RPS);
      Serial.print(F("Dat toc do ")); Serial.print(target); Serial.println(F(" vong/s"));
    } else if (s == "d") {
      if (current == 0) {
        dirCW = !dirCW; digitalWrite(PIN_DIR, dirCW ? HIGH : LOW);
        noInterrupts(); stepDir = dirCW ? 1 : -1; interrupts();
        haveRef = false;
        Serial.println(dirCW ? F("Chieu 1") : F("Chieu 2"));
      } else Serial.println(F("Dung lai truoc (r 0)"));
    } else if (s == "z") {
      haveRef = false; revCount = 0; Serial.println(F("Da xoa thong ke"));
    } else if (s == "?") {
      noInterrupts(); long sa = stepAbs; interrupts();
      Serial.print(F("toc do ")); Serial.print(current); Serial.print(F(" vong/s | buoc tuyet doi "));
      Serial.println(sa);
    }
  }

  // tang/giam toc moi 20 ms
  if (millis() - tLast >= 20) {
    float dt = (millis() - tLast) / 1000.0; tLast = millis();
    float prev = current, step = ACCEL * dt;
    if (current < target) current = min(target, current + step);
    else if (current > target) current = max(target, current - step);
    if (current != prev) applySpeed(current);
  }

  // moi lan qua moc 0 do
  if (newIdx) {
    noInterrupts(); long s = idxSteps, rs = revSteps; unsigned long ru = revUs, tu = lastIdxUs; bool hr = haveRev; newIdx = false; interrupts();
    // chi thong ke khi da quay deu (dat toc do)
    bool steady = (current == target) && current > 0;
    if (steady && !haveRef) { haveRef = true; refSteps = s; devMin = devMax = 0; revCount = 0; }
    if (haveRef) {
      long dev = (s - refSteps) % STEPS_PER_REV;                 // vi tri moc so voi tham chieu
      if (dev > STEPS_PER_REV / 2) dev -= STEPS_PER_REV;
      if (dev < -STEPS_PER_REV / 2) dev += STEPS_PER_REV;
      if (steady) { devMin = min(devMin, dev); devMax = max(devMax, dev); revCount++; }
      Serial.print(F("MOC 0 t=")); Serial.print(tu); Serial.print(F(" #")); Serial.print(revCount);
      if (hr) { Serial.print(F(" | buoc/vong ")); Serial.print(rs); Serial.print(F(" | ")); Serial.print(ru / 1000.0, 1); Serial.print(F(" ms")); }
      Serial.print(F(" | lech ")); Serial.print(dev); Serial.print(F(" buoc ("));
      Serial.print(dev * 360.0 / STEPS_PER_REV, 2); Serial.print(F(" do) | min..max "));
      Serial.print(devMin); Serial.print(F("..")); Serial.print(devMax); Serial.print(F(" buoc = "));
      Serial.print((devMax - devMin) * 360.0 / STEPS_PER_REV, 2); Serial.println(F(" do"));
    } else {
      Serial.print(F("MOC 0 t=")); Serial.print(tu); Serial.println(F(" (dang tang/giam toc, chua thong ke)"));
    }
  }
}
