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

### 4. Kiến trúc Điều hướng (Navigation) & Nav2
- **DWA Local Planner**: Không chạy trên ESP32 mà chạy trên **Main Thread của Trình duyệt Web**. Lấy dữ liệu global path (từ A*), mô phỏng quỹ đạo tránh vật cản (DWA), và liên tục gửi lệnh tốc độ `cmd_vel` (`v`, `w`) tần số cao (~10Hz) xuống ESP32 thông qua WebSocket (`VelocityMux`).
- **Path Smoothing**: Đầu ra của thuật toán A* gốc bị ziczac. Đã áp dụng `Gradient Descent Smoothing` kết hợp giới hạn `Ramer-Douglas-Peucker (RDP)` để mượt mà đường đi, giảm tải waypoint cho ESP32.
- **ESP32**: Chỉ chịu trách nhiệm chạy PID bám vận tốc `cmd_vel` hoặc bám đường thẳng cơ bản, nhường phần lớn AI và tính toán đường đi phức tạp cho phía Web-app (TDTU Thesis).

### 5. Tối ưu AMR Navigation Pipeline (Replan & Collision)
- **Đồng bộ Threshold**: PathFinder (A*) và DWA (Local) phải chung ngưỡng va chạm (`cost >= 120`). Việc chênh lệch threshold (vd DWA: 200, A*: 100) sẽ khiến DWA chủ động đi vào vùng đỏ/hồng của bản đồ gây đâm vật cản.
- **Rectangular Footprint**: Khi chạy mô phỏng (`simEngine`) hoặc check va chạm vật lý, phải dùng kích thước khung xe hình chữ nhật/vuông (VD: 30x30cm) với góc quay `theta` check đủ 8-point thay vì bán kính hình tròn `radius`.
- **Live Forward Scan (Anti-Jitter)**: Chỉ thắng gấp (brake) và Replan khi xe quét thấy vật cản thực sự ở ngưỡng nguy hiểm tuyệt đối (`cost >= 200` Inscribed/Lethal). Nếu dùng `cost >= 80` (vùng inflation xanh an toàn) sẽ gây hiện tượng false-positive, xe liên tục giật (jitter) nhảy qua lại giữa TRACK và RECOVERY. Check khoảng cách dừng (braking distance) động theo vận tốc `vel.v`.
- **Merge Path Smoothly**: Khi RECOVERY_REPLAN tìm ra đường đi mới, TUYỆT ĐỐI KHÔNG reset waypoint về index 0. Phải dò waypoint gần nhất phía trước vị trí hiện tại trên quỹ đạo mới và bắt đầu từ đó (`startIdx = bestIdx + 1`), giúp xe đi tiếp mượt mà không bị quay ngược đầu.
