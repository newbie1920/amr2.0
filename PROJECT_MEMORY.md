# Dự án: TDTU Thesis Generator - AMR 2.0
**Mục tiêu**: Xây dựng báo cáo và hoàn thiện dự án theo chuẩn MauDATN_2021 của TDTU.

## Các "Sự thật" (Facts) đã thống nhất
### 1. Phân bổ INA3221 (Đã điều chỉnh do CH1 bị cháy R100)
Sử dụng cảm biến INA3221 (Địa chỉ I2C: 0x40) để giám sát công suất:
- **Kênh 1 (CH1)**: Bỏ qua (Đã cháy điện trở Shunt do cắm sai ban đầu).
- **Kênh 2 (CH2)**: Đo tổng Pin (Điện áp tổng và dòng điện tổng).
  - Kết nối: Nối tiếp tại cực dương của Pin chính.
  - Sử dụng trong code: `ina_busV[1]` (INA_CH_BATT) thay thế cho ADC.
- **Kênh 3 (CH3)**: Đo năng lượng tiêu thụ của động cơ.
  - Kết nối: Nối tiếp tại đầu vào 12V (VMOT) của mạch cầu H L298N.
  - Tác dụng: Cảnh báo kẹt bánh, đánh giá công suất khi xe có tải/leo dốc.
  - Dữ liệu Web: `motorV` và `motorA`.
- **Mạch 5V**: Tạm thời không giám sát qua INA3221.

### 2. Cấu hình phần cứng cốt lõi (ESP32-S3 N16R8)
- **Động cơ**: L298N (Trái: EN=8, IN1=9, IN2=10 | Phải: EN=11, IN3=12, IN4=13)
- **Encoder**: Trái (A=4, B=5), Phải (A=6, B=7)
- **I2C**: SDA=39, SCL=40 (Dùng chung cho MPU6050, OLED SSD1306, INA3221)
- **Lidar**: A1M8 (RX_ESP=46, TX_ESP=47, PWM=15)
- **Khác**: LED RGB Onboard=48

### 3. Quy ước làm việc
- Auto-Validation: Luôn tự kiểm tra lỗi sau khi sửa code.
- Surgical Operations: Giới hạn tìm kiếm và sửa code cục bộ.
- Chuẩn báo cáo: MauDATN_2021.
