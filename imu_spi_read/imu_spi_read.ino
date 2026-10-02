#include <SPI.h>

// Định nghĩa các chân giao tiếp SPI (VSPI trên ESP32)
const int SCK_PIN = 18;
const int MISO_PIN = 19;
const int MOSI_PIN = 23;
const int CS_PIN = 5;

// --- ĐỊA CHỈ THANH GHI THƯỜNG GẶP ---
// (0x3B là địa chỉ thanh ghi bắt đầu đo gia tốc của dòng họ MPU9250 / ICM20689 phổ biến)
const uint8_t ACCEL_XOUT_H = 0x3B; 

void setup() {
  // 1. TỐC ĐỘ BAUD CHO MÀN HÌNH SERIAL ĐANG ĐẶT LÀ 115200
  Serial.begin(115200); 
  while (!Serial) { delay(10); }

  Serial.println("Khoi tao SPI de giao tiep voi IMU...");

  // Thiết lập chân CS
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Khởi tạo SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
}

// Hàm đọc nhiều byte liên tiếp từ IMU
void readRegisters(uint8_t reg, uint8_t count, uint8_t *dest) {
  // 2. TỐC ĐỘ CLOCK CỦA SPI ĐANG ĐẶT LÀ 1 MHz (1000000 Hz)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);

  // Gửi địa chỉ bắt đầu đọc (bit 7 = 1 để ra lệnh đọc)
  SPI.transfer(reg | 0x80);

  // Nhận liên tiếp các byte dữ liệu gửi về từ IMU
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = SPI.transfer(0x00);
  }

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void loop() {
  uint8_t rawData[6]; // Mảng chứa 6 byte dữ liệu (X_High, X_Low, Y_High, Y_Low, Z_High, Z_Low)

  // Đọc 6 byte dữ liệu gia tốc bắt đầu từ thanh ghi ACCEL_XOUT_H
  readRegisters(ACCEL_XOUT_H, 6, rawData);

  // Ghép 2 byte (High và Low) lại thành số nguyên 16-bit có dấu (đúng với định dạng dữ liệu của đa số IMU)
  int16_t accelX = (rawData[0] << 8) | rawData[1];
  int16_t accelY = (rawData[2] << 8) | rawData[3];
  int16_t accelZ = (rawData[4] << 8) | rawData[5];

  // In các giá trị đọc được ra màn hình Serial
  Serial.print("Accel X: "); Serial.print(accelX);
  Serial.print(" | Accel Y: "); Serial.print(accelY);
  Serial.print(" | Accel Z: "); Serial.println(accelZ);

  // Gửi giá trị về liên tục mỗi 50ms để vẽ đồ thị (Serial Plotter) hoặc xem trên Serial Monitor
  delay(50); 
}
