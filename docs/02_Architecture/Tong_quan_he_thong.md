# Tổng quan Hệ thống AMR 2.0

Dự án này sử dụng mô hình **PC-First Architecture**, trong đó:

1. **Firmware (ESP32-S3)**
   - Đọc dữ liệu từ Lidar, Encoder, IMU.
   - Điều khiển động cơ.
   - Truyền dữ liệu thô (raw data) lên PC qua WebSockets.

2. **PC / Cụm tính toán trung tâm (Brain)**
   - Nhận dữ liệu từ ESP32.
   - Xử lý SLAM (Tạo bản đồ), Path Planning (A*, DWA).
   - Đóng vai trò là ROS2 Node (dự kiến tích hợp Nav2).

3. **Giao diện điều khiển (Dashboard - ReactJS)**
   - Hiển thị bản đồ thời gian thực (Canvas 2D / WebGL).
   - Vẽ Costmap, Lidar Point Cloud.
   - Các nút điều khiển và debug.

### Liên kết liên quan:
- Để xem hướng dẫn xử lý Lidar, xem: [[LIDAR_MAPPING_GUIDE]]
- Để xem các lỗi đã giải quyết về Layout UI, xem: [[Ghi_chu_hang_ngay]]
