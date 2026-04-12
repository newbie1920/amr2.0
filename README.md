# 🤖 AMR 2.0 - Hệ Thống Quản Lý Robot Vận Chuyển Tự Động (WMS & Fleet Management)

AMR 2.0 (Autonomous Mobile Robot 2.0) là dự án phần mềm quản lý và điều hướng hệ thống xe tự hành (AMR) phục vụ trong môi trường kho xưởng. Hệ thống có khả năng tự động xử lý hàng hóa (Nhập/Xuất), dẫn đường cho xe, theo dõi trạng thái di chuyển trực tiếp trên bản đồ 3D và kết nối chặt chẽ với hệ cơ sở dữ liệu kho bãi.

## ✨ Tính Năng Nổi Bật

1. **Giao Diện Bản Đồ 3D (3D Warehouse Map)**: 
   - Mô phỏng không gian nhà kho theo thời gian thực (Real-time).
   - Tự động hiển thị các vị trí của kệ hàng, trụ sạc (Charging Stations) và cổng giao dịch hàng.
2. **Hệ Thống Dẫn Đường Tự Động (Pathfinding)**: 
   - Tự động tính toán đường đi tối ưu (A* Algorithm) từ vị trí hiện tại đến kệ hàng hoặc cổng xuất/nhập.
   - Gửi quỹ đạo chuẩn xác (Trajetory & Waypoints) xuống xe qua giao thức kết nối siêu tốc WebSocket.
3. **Quản Lý Nhiệm Vụ Kho (WMS - Warehouse Management System)**:
   - Dễ dàng tạo phiếu Nhập và Xuất kho.
   - Quản lý Số lượng, SKU, và tự động liên kết dữ liệu mượt mà lên Supabase.
   - Giao việc tự động: Khi có phiếu mới, xe tự tính toán đường đi, làm nhiệm vụ và tự báo cáo.
4. **Điều Khiển Bằng Tay (Manual Override)**:
   - Hỗ trợ thao tác thủ công (WASD / Mũi tên) để gửi lệnh vận tốc cơ bản (Linear/Angular) giúp nhân sự dễ gỡ rối tình huống ngặt nghèo.
   - Nút Dừng Khẩn Cấp (E-STOP) giúp kích hoạt phanh trên toàn bộ phi đội xe ngay lập tức.
5. **Đồng Bộ Dữ Liệu Thời Gian Thực**:
   - Giao tiếp với Firmware (ESP32-S3) bằng WebSocket với tốc độ cực cao, giúp theo dõi Pin, thông số cảm biến, IMU, toạ độ liên tục.

## 🛠 Công Nghệ Sử Dụng

Phần Mềm Quản Trị (App Desktop & Web):
- **Framework**: Tauri, React 19, Vite
- **Ngôn ngữ**: JavaScript (JSX), CSS thuần túy.
- **Quản lý State**: Zustand
- **Hiển thị 3D Map**: Three.js, React Three Fiber & Drei
- **Backend / Database**: Supabase (PostgreSQL)

Mảng Firmware & Điều khiển nhúng (ESP32-S3):
- **Ngôn ngữ**: C++ (PlatformIO)
- **Giao thức**: WebSocket, WiFi
- Phụ trách vi điều khiển tốc độ, phản hồi PID, theo dõi Encoder và cập nhật vị trí tương đối.

## 🚀 Hướng Dẫn Cài Đặt (Local Development)

### Yêu Cầu Hệ Thống:
- [Node.js](https://nodejs.org/en/) & `npm`
- [Rust](https://www.rust-lang.org/) (nếu biên dịch Tauri Desktop App)

### Khởi Chạy Dự Án:
```bash
# 1. Cài đặt các gói phụ thuộc
npm install

# 2. Khởi chạy Web Server cục bộ (Giao diện chạy trên trình duyệt web)
npm run dev

# 3. Khởi chạy App Desktop (Tauri)
npm run tauri dev
```

---
*Phát triển bởi Lê Minh Đạt.*
