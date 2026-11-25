/*
 * ROBOT LAU NHÀ - ARDUINO UNO + HW-130 (Motor Shield V1 / L293D) + HC-05
 * BẢN TỐI ƯU: Parser nhẹ không sscanf, watchdog không spam shift-register,
 * hàng đợi TX Bluetooth chống khóa ngắt làm rớt gói UART từ ESP32.
 *
 * Thư viện cần cài: "Adafruit Motor Shield V1 Library" (AFMotor.h)
 *
 * Kênh động cơ trên HW-130 (giữ nguyên 100% đấu dây):
 *   M1 = 3 bánh bên TRÁI (song song)   M2 = 3 bánh bên PHẢI (song song)
 *   M3 = chổi lau 1                    M4 = chổi lau 2
 *
 * HC-05 : TX -> A0 | RX <- A1 (QUA cầu phân áp 1k/2k) | VCC 5V | GND (baud 9600)
 * ESP32 : ESP32 GPIO19 (TX) -> Arduino RX (chân 0)
 *         ESP32 GPIO18 (RX) <- Arduino TX (chân 1) QUA cầu phân áp 5V->3.3V (1k/2k)
 *         GND chung.
 *         *** Khi nạp code qua USB: bật "Nạp Arduino Uno (Nhả D0/D1)" trên web ESP32
 *             (giữ ESP32 có nguồn) hoặc rút dây chân 0/1 ***
 *
 * Giao thức UART 38400 bps (tương thích 100% với ESP32):
 *   Nhận từ ESP32: "M,<L>,<R>\n" (-255..255), "W,<mop>\n" (0..255), "T,<text>\n"
 *   Gửi lên ESP32: "C,<char>\n" (A, X, F, B, L, R, W, w, S)
 */

#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <string.h>

SoftwareSerial bt(A0, A1);   // RX, TX

AF_DCMotor mL(1, MOTOR12_1KHZ);
AF_DCMotor mR(2, MOTOR12_1KHZ);
AF_DCMotor mopA(3);
AF_DCMotor mopB(4);

// Đổi true nếu bánh chạy ngược chiều mong muốn
#define INVERT_LEFT    false
#define INVERT_RIGHT   false
#define MAX_DRIVE_PWM  210   // Giới hạn trần cứng bảo vệ L293D (3 motor/kênh)
#define CMD_TIMEOUT_MS 400

// Hàng đợi vòng TX cho HC-05: gửi tối đa 1 byte/vòng khi Serial rảnh
// để SoftwareSerial không khóa ngắt liên tục ~35 ms làm tràn buffer UART 38400
#define BT_TX_BUF_SIZE 64
char btTxBuf[BT_TX_BUF_SIZE];
uint8_t btTxHead = 0;
uint8_t btTxTail = 0;

unsigned long lastCmdMs = 0;
bool motorsStopped = true;
int curL = 999, curR = 999, curMop = 999;

void btQueueChar(char c) {
  uint8_t next = (uint8_t)((btTxHead + 1) % BT_TX_BUF_SIZE);
  if (next != btTxTail) {
    btTxBuf[btTxHead] = c;
    btTxHead = next;
  }
}

void btQueueLine(const char *s) {
  while (*s) btQueueChar(*s++);
  btQueueChar('\r');
  btQueueChar('\n');
}

void btPumpTx() {
  // Chỉ gửi 1 byte ra SoftwareSerial khi Hardware Serial đang rảnh
  if (btTxTail != btTxHead && Serial.available() == 0) {
    bt.write((uint8_t)btTxBuf[btTxTail]);
    btTxTail = (uint8_t)((btTxTail + 1) % BT_TX_BUF_SIZE);
  }
}

void setMotor(AF_DCMotor &m, int v, bool invert) {
  if (invert) v = -v;
  if (v > MAX_DRIVE_PWM)  v = MAX_DRIVE_PWM;
  if (v < -MAX_DRIVE_PWM) v = -MAX_DRIVE_PWM;
  if (v > 0)      { m.setSpeed((uint8_t)v);  m.run(FORWARD); }
  else if (v < 0) { m.setSpeed((uint8_t)-v); m.run(BACKWARD); }
  else            { m.setSpeed(0);           m.run(RELEASE); }
}

void drive(int l, int r) {
  if (l != curL) {
    setMotor(mL, l, INVERT_LEFT);
    curL = l;
  }
  if (r != curR) {
    setMotor(mR, r, INVERT_RIGHT);
    curR = r;
  }
  if (l != 0 || r != 0) motorsStopped = false;
}

void setMop(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  if (v == curMop) return;
  curMop = v;
  mopA.setSpeed((uint8_t)v);
  mopB.setSpeed((uint8_t)v);
  if (v > 0) {
    mopA.run(FORWARD);
    mopB.run(FORWARD);
    motorsStopped = false;
  } else {
    mopA.run(RELEASE);
    mopB.run(RELEASE);
  }
}

void stopAll() {
  drive(0, 0);
  setMop(0);
  motorsStopped = true;
}

// Parse nhanh "<int>,<int>" không dùng sscanf (nhẹ Flash & nhanh gấp ~10 lần)
bool parseTwoInts(const char *s, int &a, int &b) {
  char *end1 = NULL;
  long v1 = strtol(s, &end1, 10);
  if (end1 == s || *end1 != ',') return false;
  char *end2 = NULL;
  long v2 = strtol(end1 + 1, &end2, 10);
  if (end2 == end1 + 1) return false;
  a = (int)v1;
  b = (int)v2;
  return true;
}

// ---- Xử lý dòng nhận từ ESP32 ----
void handleLine(const char *s) {
  if (s[0] == 'M' && s[1] == ',') {
    int l = 0, r = 0;
    if (parseTwoInts(s + 2, l, r)) {
      drive(l, r);
      lastCmdMs = millis();
      static bool led = false;
      led = !led;
      digitalWrite(LED_BUILTIN, led);
    }
  } else if (s[0] == 'W' && s[1] == ',') {
    setMop(atoi(s + 2));
    lastCmdMs = millis();
  } else if (s[0] == 'T' && s[1] == ',') {
    btQueueLine(s + 2);   // Xếp hàng gửi dần sang điện thoại qua HC-05
  }
}

void setup() {
  Serial.begin(38400);    // Liên kết ESP32
  bt.begin(9600);         // HC-05
  pinMode(LED_BUILTIN, OUTPUT);
  stopAll();
  lastCmdMs = millis();
  bt.println(F("Robot san sang. Gui A=lau, X=dung."));
}

void loop() {
  // 1. Ưu tiên đọc toàn bộ dữ liệu từ ESP32 trước
  static char line[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line[n] = '\0';
      if (n > 0) handleLine(line);
      n = 0;
    } else if (c != '\r' && n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }

  // 2. Chuyển lệnh từ điện thoại (HC-05) -> ESP32
  while (bt.available()) {
    char c = (char)bt.read();
    if (strchr("AXFBLRWwS", c)) {
      Serial.print("C,");
      Serial.print(c);
      Serial.print('\n');
    }
  }

  // 3. Xả tối đa 1 byte telemetry sang HC-05 khi UART chính rảnh
  btPumpTx();

  // 4. Watchdog an toàn: chỉ gọi stopAll() đúng 1 lần khi mất kết nối quá 400 ms
  if (!motorsStopped && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    stopAll();
  }
}
