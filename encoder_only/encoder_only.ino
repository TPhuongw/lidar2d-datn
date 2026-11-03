/*
 * encoder_only.ino - CHI test cam bien khe, KHONG dieu khien dong co. Arduino UNO.
 * Quay tay dia / la chan qua khe va xem ket qua.
 *
 * Noi day (module cam bien khe LM393):
 *   VCC -> 5V      GND -> GND      DO -> D2      (AO neu co: de trong)
 * (Neu van dung con OPB tran: TIM -> 2x100 ohm -> 5V, TRANG/XANH DUONG -> GND, XANH LA -> D2)
 *
 * LED tren board (D13) sang khi khe BI CHE (D2 = CAO), tat khi tia sang di qua.
 * Serial Monitor 115200: moi lan doi trang thai in 1 dong; moi giay in tong so lan che.
 * Gui 'z' de dat lai bo dem.
 * CODE THU NGHIEM, CHUA CHAY TREN PHAN CUNG.
 */
const int PIN_ENC = 2;
const int PIN_LED = 13;
const unsigned long LOCK_US = 300;   // bo canh rung < 300 us (do duoc o lan test truoc)

volatile unsigned long nBlock = 0, nRaw = 0, tLastUs = 0;

void encISR() {                      // canh LEN = bat dau bi che
  unsigned long t = micros();
  nRaw++;
  if (t - tLastUs < LOCK_US) return;
  tLastUs = t;
  nBlock++;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENC, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC), encISR, RISING);
  Serial.println(F("encoder_only: quay tay qua khe. 'z' = dat lai bo dem"));
}

void loop() {
  static int last = -1;
  static unsigned long tPrint = 0;
  int s = digitalRead(PIN_ENC);
  digitalWrite(PIN_LED, s);
  if (s != last) {
    last = s;
    noInterrupts(); unsigned long n = nBlock; interrupts();
    Serial.print(s ? F("BI CHE   ") : F("THONG    ")); Serial.print(F("| so lan che: ")); Serial.println(n);
  }
  if (Serial.available() && Serial.read() == 'z') {
    noInterrupts(); nBlock = 0; nRaw = 0; interrupts();
    Serial.println(F("--- dat lai ---"));
  }
  if (millis() - tPrint >= 1000) {
    tPrint = millis();
    noInterrupts(); unsigned long n = nBlock, r = nRaw; interrupts();
    Serial.print(F("[1s] so lan che ")); Serial.print(n);
    Serial.print(F(" | canh tho (ca rung) ")); Serial.println(r);
  }
}
