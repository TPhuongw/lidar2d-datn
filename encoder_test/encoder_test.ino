/*
 * encoder_test.ino - Doc vong encoder 180 vach (179 ranh + 1 rang rong) qua OPB871, ESP32.
 *
 * Phan cung (OPB871 da CO LAP khoi bo goc):
 *   5V --220R--> Anode LED (E)  ;  Cathode LED --> GND
 *   3V3 --10k--> Collector (S)  --> GPIO 27 (ENC_PIN)  ;  Emitter (S) --> GND
 *   Khe trong (anh sang qua)  -> transistor dan -> GPIO = LOW
 *   Rang che                  -> transistor tat -> GPIO = HIGH
 *
 * Nguyen ly: ngat ngoai o canh LEN (bat dau moi rang). Ghi thoi diem (us) moi canh.
 *   Binh thuong 2 canh cach nhau T (2 do). Rang rong -> khoang cach ~2T -> do la VACH INDEX (0 do).
 *   Goc = so canh ke tu index * 2 do (+ noi suy theo thoi gian giua 2 canh).
 *
 * Mo Serial Monitor 115200. Quay bang tay hoac bang dong co.
 * DAY LA CODE THU NGHIEM, CHUA CHAY TREN PHAN CUNG.
 */
#include <Arduino.h>

const int ENC_PIN = 27;
const float DEG_PER_EDGE = 2.0f;      // 180 vi tri/vong
const int EDGES_PER_REV = 179;        // 180 vi tri - 1 vi tri bi rang rong "nuot" (kiem tra lai!)
const float GAP_RATIO = 1.5f;         // khoang > 1.5 lan khoang truoc -> index

volatile uint32_t lastEdgeUs = 0;     // thoi diem canh gan nhat
volatile uint32_t lastPeriodUs = 0;   // khoang thoi gian giua 2 canh gan nhat
volatile int32_t edgeIdx = -1;        // so canh ke tu index (-1 = chua dong bo)
volatile uint32_t edgesThisRev = 0;   // dem canh trong vong hien tai
volatile uint32_t edgesLastRev = 0;   // so canh do duoc o vong truoc (kiem tra 179)
volatile uint32_t revPeriodUs = 0;    // thoi gian 1 vong
volatile uint32_t lastIndexUs = 0;
volatile uint32_t revCount = 0;
volatile uint32_t glitch = 0;         // canh qua sat nhau (nhieu)

void IRAM_ATTR onEdge() {
  uint32_t now = micros();
  uint32_t p = now - lastEdgeUs;
  if (lastPeriodUs && p < lastPeriodUs / 3) { glitch++; return; }   // loc nhieu canh doi
  if (lastPeriodUs && p > GAP_RATIO * lastPeriodUs && p < 3 * lastPeriodUs) {
    // gap rong -> index
    if (lastIndexUs) { revPeriodUs = now - lastIndexUs; edgesLastRev = edgesThisRev; revCount++; }
    lastIndexUs = now;
    edgesThisRev = 0;
    edgeIdx = 0;
  } else if (edgeIdx >= 0) {
    edgeIdx++;
  }
  edgesThisRev++;
  lastPeriodUs = (p < 2000000) ? p : 0;   // qua 2 s khong co canh -> coi nhu dung
  lastEdgeUs = now;
}

// Goc hien tai (do), -1 neu chua dong bo. Noi suy theo thoi gian ke tu canh cuoi.
float currentAngle() {
  noInterrupts();
  int32_t idx = edgeIdx; uint32_t t0 = lastEdgeUs, p = lastPeriodUs;
  interrupts();
  if (idx < 0 || p == 0) return -1;
  float frac = (float)(micros() - t0) / p;
  if (frac > 1.0f) frac = 1.0f;
  // Sau index, canh 0 o 0 do; vi tri bi nuot nam truoc index nen goc khong bi lech
  return fmodf((idx + frac) * DEG_PER_EDGE, 360.0f);
}

void setup() {
  Serial.begin(115200);
  pinMode(ENC_PIN, INPUT);              // da co dien tro keo len 10k ngoai
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), onEdge, RISING);
  Serial.println("encoder_test: quay vong encoder...");
}

void loop() {
  static uint32_t tPrint = 0;
  if (millis() - tPrint >= 200) {
    tPrint = millis();
    noInterrupts();
    uint32_t rp = revPeriodUs, el = edgesLastRev, rc = revCount, g = glitch;
    interrupts();
    float rps = rp ? 1e6f / rp : 0;
    Serial.printf("muc=%d  goc=%7.2f  vong=%lu  canh/vong=%lu  toc do=%.2f vong/s  nhieu=%lu\n",
                  digitalRead(ENC_PIN), currentAngle(), rc, el, rps, g);
  }
}
