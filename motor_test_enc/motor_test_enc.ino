/*
 * motor_test_enc.ino - Chay thu 42CME06 + CL42C bang UNO (lenh qua Serial, chua can bien tro)
 *   + dem xung cam bien khe doc dia encoder SICK 180 vach, in moi giay.
 *
 * Noi day (CL42C dat muc tin hieu 5 V; noi kieu "cathode chung"):
 *   UNO D9  -> PUL+        PUL- -> UNO GND
 *   UNO D8  -> DIR+        DIR- -> UNO GND
 *   ENA+/ENA- : de trong (driver enable mac dinh)
 *   Cam bien khe (mau day da noi dai):
 *     TIM -> 2 dien tro 100 ohm NOI TIEP (= 200 ohm) -> 5V      TRANG -> GND
 *     XANH LA -> D2 (khong can 10k: dung keo len noi INPUT_PULLUP)      XANH DUONG -> GND
 *   Nguon 24 V -> +Vdc / GND cua CL42C. GND nguon KHONG can noi voi GND UNO (ngo vao cach ly quang).
 *
 * DIP CL42C: chon 3200 xung/vong theo bang in tren driver; SW6 = off (closed-loop); SW7 = off (PUL/DIR).
 *
 * Lenh qua Serial Monitor (115200, "Newline"):
 *   r 1.5   -> quay 1.5 vong/s (tang/giam toc tu tu)
 *   r 0     -> dung (giam toc)
 *   d       -> dao chieu (chi khi dang dung)
 *   ?       -> in trang thai
 *   l       -> bat/tat dong in encoder moi giay
 *   f       -> bat/tat bo loc canh rung (mac dinh BAT)
 *   s 720   -> dat so vach/vong dung de tinh 'mong doi' va bo loc
 *   i       -> bat/tat dong in moi lan qua moc 0 do
 *   p       -> ghi 400 khoang cach giua cac canh tho roi in ra (xem hinh dang tin hieu)
 * Xung tao bang tone() (song vuong 50%), du cho toc do 0.1 .. 10 vong/s o 3200 xung/vong.
 * CODE THU NGHIEM, CHUA CHAY TREN PHAN CUNG.
 */
const int PIN_PUL = 9;
const int PIN_DIR = 8;
const long PULSES_PER_REV = 3200;   // PHAI khop DIP SW1-SW4
const float ACCEL = 1.0;            // vong/s^2 (tang toc 1 vong/s moi giay)
const float MAX_RPS = 10.0;
const int PIN_ENC = 2;               // INT0
int SLOTS_PER_REV = 180;            // 180 vach, 1 vach rong (nguoi dung dem tay). Doi bang lenh 's <so vach>'
volatile unsigned long encEdges = 0, tEdgeUs = 0, gapMaxUs = 0;
volatile unsigned long encRejected = 0;   // canh bi bo vi den qua som (rung/nhieu)
volatile unsigned long lockoutUs = 300;
volatile unsigned long dropUs = 1000000; // khoang trong > 3 chu ky vach = MAT TIN HIEU
volatile unsigned long dropCnt = 0, dropSumUs = 0;
// nhan moc 0 do: vach rong cho khoang cach ~2 lan binh thuong
volatile unsigned long avgGapUs = 0;      // trung binh truot khoang cach vach binh thuong
volatile unsigned long tIdxUs = 0, revUs = 0;
volatile int edgesSinceIdx = 0, revEdges = 0;
volatile bool newRev = false;
bool idxLog = true;   // canh moi phai cach canh truoc it nhat chung nay
volatile bool filterOn = true;      // bat: bo canh rung (moi vach dang bi dem ~4 lan)
// ghi lai khoang cach giua cac canh THO (chua loc) de xem hinh dang tin hieu qua 1 vong
const int CAP_N = 400;
volatile uint16_t capBuf[CAP_N];
volatile int capIdx = -1;                 // -1 = khong ghi
volatile unsigned long tRawUs = 0;
unsigned long tPrint = 0;
bool logOn = true;
void encISR() {
  unsigned long t = micros(), g = t - tEdgeUs;
  if (capIdx >= 0 && capIdx < CAP_N) {
    unsigned long gr = t - tRawUs; capBuf[capIdx++] = (gr > 65535UL) ? 65535 : gr;
  }
  tRawUs = t;
  if (filterOn && g < lockoutUs) { encRejected++; return; }   // canh "rung" sat ngay sau canh that
  tEdgeUs = t;
  if (g > dropUs) { dropCnt++; dropSumUs += g; }
  encEdges++;
  if (avgGapUs && g >= 3 * avgGapUs) {
    // mat tin hieu (thieu nhieu vach): khong cap nhat trung binh, huy dong bo moc 0
    tIdxUs = 0; edgesSinceIdx = 0;
  } else if (avgGapUs && g > avgGapUs + avgGapUs / 2
             && (tIdxUs == 0 || edgesSinceIdx > (SLOTS_PER_REV * 8) / 10)) {
    // 1.5..3 lan khoang binh thuong VA da qua it nhat 80% so vach -> vach rong = moc 0 do
    if (tIdxUs) { revUs = t - tIdxUs; revEdges = edgesSinceIdx + 1; newRev = true; }
    tIdxUs = t; edgesSinceIdx = 0;
  } else {
    edgesSinceIdx++;
    avgGapUs = avgGapUs ? (avgGapUs * 7 + g) / 8 : g;
  } if (g > gapMaxUs) gapMaxUs = g;
}

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
  pinMode(PIN_ENC, INPUT_PULLUP);    // keo len noi ~20-50k; co them 10k ngoai cung duoc
  attachInterrupt(digitalPinToInterrupt(PIN_ENC), encISR, FALLING);
  Serial.println(F("v6 | lenh: r <vong/s>, r 0, d, ?, l (in/giay), i (in moc 0), f (loc), s <so vach>, p (ghi 400 canh)"));
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
    } else if (s == "f") {
      filterOn = !filterOn; Serial.println(filterOn ? F("Loc: BAT") : F("Loc: TAT"));
    } else if (s.startsWith("s")) {
      int v = s.substring(1).toInt();
      if (v > 0) { SLOTS_PER_REV = v; Serial.print(F("So vach/vong = ")); Serial.println(v); }
    } else if (s == "p") {
      noInterrupts(); capIdx = 0; interrupts();
      Serial.println(F("Dang ghi 400 khoang cach canh tho..."));
    } else if (s == "i") {
      idxLog = !idxLog;
    } else if (s == "l") {
      logOn = !logOn;
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
    // bo loc: do thuc te (lenh p o 3 vong/s) cho thay chum canh rung chi keo dai <= ~160 us,
    // nen khoa co dinh 300 us (khong phu thuoc toc do), nhung khong qua 40% chu ky vach.
    float per = (current > 0.05) ? 1e6 / (current * SLOTS_PER_REV) : 50000;
    unsigned long lo = min(300UL, (unsigned long)(0.4 * per));
    noInterrupts(); lockoutUs = lo; dropUs = (unsigned long)(3 * per); interrupts();
  }
  // moi lan qua moc 0 do: in so canh va thoi gian cua vong vua xong
  if (newRev) {
    noInterrupts(); int e = revEdges; unsigned long T = revUs; newRev = false; interrupts();
    if (idxLog) {
      Serial.print(F("MOC 0: vong vua roi ")); Serial.print(e); Serial.print(F(" canh, "));
      Serial.print(T / 1000.0, 1); Serial.print(F(" ms = ")); Serial.print(1e6 / T, 3);
      Serial.print(F(" vong/s | xung buoc tuong ung ~")); Serial.println((long)(T * 1e-6 * current * PULSES_PER_REV));
    }
  }
  // in bang ghi khi du 400 canh
  if (capIdx >= CAP_N) {
    noInterrupts(); capIdx = -1; interrupts();
    Serial.println(F("--- khoang cach giua cac canh tho (us), 10 so/dong ---"));
    for (int i = 1; i < CAP_N; i++) {            // bo so dau (tinh tu truoc khi bat ghi)
      Serial.print(capBuf[i]); Serial.print((i % 10 == 0) ? '\n' : ' ');
    }
    Serial.println(F("\n--- het ---"));
  }
  // moi giay: so canh encoder dem duoc so voi mong doi
  if (millis() - tPrint >= 1000) {
    tPrint = millis();
    noInterrupts(); unsigned long n = encEdges; encEdges = 0; unsigned long gmax = gapMaxUs; gapMaxUs = 0;
    unsigned long rj = encRejected; encRejected = 0;
    unsigned long dc = dropCnt, ds = dropSumUs; dropCnt = 0; dropSumUs = 0; interrupts();
    if (logOn) {
      Serial.print(F("dat ")); Serial.print(current, 2);
      Serial.print(F(" vong/s | encoder ")); Serial.print(n);
      Serial.print(F(" canh/s (mong doi ~")); Serial.print((long)(current * SLOTS_PER_REV));
      Serial.print(F(") | bo ")); Serial.print(rj);
      Serial.print(F(" | mat tin hieu ")); Serial.print(dc); Serial.print(F(" lan, tong ")); Serial.print(ds / 1000);
      Serial.print(F(" ms | quang cach dai nhat ")); Serial.print(gmax); Serial.println(F(" us"));
    }
  }
}
