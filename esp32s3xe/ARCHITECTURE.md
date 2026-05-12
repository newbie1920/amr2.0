# Kiến trúc Firmware ESP32-S3 (AMR 2.0)

## Tổng quan dự án `esp32s3xe`
Dự án này là firmware điều khiển robot tự hành (Autonomous Mobile Robot - AMR) chạy trên vi điều khiển ESP32-S3. Nó tích hợp cả điều khiển phần cứng thực tế (Motor, Encoder, RPLidar A1, IMU MPU6050) và chế độ mô phỏng phần cứng (Hardware-in-the-Loop - HITL). Hệ thống sử dụng hệ điều hành thời gian thực FreeRTOS để chia luồng xử lý song song, tối ưu hoá hiệu suất cho SLAM và dẫn đường.

## Chức năng của các file mã nguồn (`.cpp` và `.h`)

### 1. Cốt lõi hệ thống & Cấu hình
* `main.cpp`: File khởi chạy chính (Entry point). Khởi tạo và định nghĩa các luồng (task) của FreeRTOS (`controlTask`, `lidarTask`, `pathfinderTask`, `explorationTask`). Phối hợp đọc cảm biến, vòng lặp điều khiển PID, SLAM, và giao tiếp mạng.
* `config.h`: Chứa toàn bộ cấu hình hệ thống: sơ đồ chân (Pin map), thông số PID, kích thước vật lý của robot (bán kính bánh xe, khoảng cách 2 bánh), cấu hình WiFi và công tắc bật/tắt chế độ mô phỏng (`SIMULATION_MODE`).

### 2. Mô phỏng (Hardware-in-the-Loop)
* `sim_engine.cpp / .h`: Động cơ vật lý (Physics Engine) cho chế độ mô phỏng. Giả lập động học của robot (kinematics), cập nhật vị trí, xử lý va chạm với tường và giả lập thêm nhiễu (noise) vào bánh xe để thử nghiệm thuật toán.
* `sim_world.cpp / .h`: Định nghĩa môi trường mô phỏng (Bản đồ nhà kho, phòng test) dưới dạng các đoạn thẳng chướng ngại vật (segments).
* `sim_lidar.cpp / .h`: Thuật toán bắn tia (Raycasting) giả lập cảm biến LiDAR. Nó sẽ tính toán các giao điểm giữa tia quét và thế giới `sim_world`.
* `sim_task.cpp / .h`: Luồng FreeRTOS chạy vòng lặp mô phỏng ở tần số 50Hz, giả lập luồng dữ liệu thay thế cho cảm biến thực để lập trình không cần lắp ráp phần cứng.

### 3. Cảm biến & Động cơ (Hardware Drivers)
* `imu_sensor.cpp / .h`: Đọc dữ liệu từ cảm biến gia tốc và con quay hồi chuyển MPU6050 qua I2C. Có logic tự động calibrate (hiệu chuẩn) khi bật nguồn.
* `odometry.cpp / .h`: Lưu trữ các biến trạng thái toàn cục của robot (vị trí `robotX, robotY, robotTheta`) và ma trận chuyển đổi (TF - Transform) giữa khung toạ độ Odom (của bánh xe) và Map (của SLAM).
* `wheel_pid.h`: Chứa logic bộ điều khiển PID kèm Feedforward (bù hệ số tĩnh) cho hai bánh độc lập, giúp bám chính xác vận tốc mục tiêu ngay cả khi điện áp pin thay đổi.
* `display_oled.cpp / .h`: Hiển thị thông số IP, trạng thái cảm biến, dung lượng pin lên màn hình OLED SSD1306.

### 4. Định vị và Xây dựng bản đồ (SLAM)
* `lidar_mapper.h`: Quản lý bản đồ lưới không gian 2D (Occupancy Grid Map). Áp dụng thuật toán tính log-odds để cập nhật xác suất khoảng trống/vật cản từ dữ liệu Lidar.
* `icp_matcher.h`: Thuật toán so khớp đám mây điểm (Iterative Closest Point). So sánh bản quét Lidar hiện tại với bản quét trước đó để sửa lỗi bánh xe trượt theo thời gian thực (rất nhanh).
* `csm_matcher.h`: Thuật toán so khớp tương quan (Correlative Scan Matching). Phân tích sự phù hợp của Lidar với toàn bộ bản đồ Grid toàn cục để sửa lỗi lũy kế, tăng độ chính xác của SLAM.
* `slam_diagnostics.h`: Thu thập log và đánh giá chất lượng SLAM (độ lệch toạ độ, thời gian tính toán).

### 5. Điều hướng & Tự hành (Navigation & Planning)
* `pathfinder.cpp / .h` & `pathfinder_types.h`: Thuật toán tìm đường toàn cục (Global Planner) dùng **A***. Hoạt động trên một luồng riêng biệt, không chặn motor, tìm quỹ đạo ngắn nhất trên Grid Map hiện tại.
* `dwa_planner.h`: Thuật toán dẫn đường cục bộ Dynamic Window Approach. Lấy quỹ đạo từ A* và mô phỏng trước các mẫu vận tốc cong (v, w) khác nhau. Tính điểm số (cost) dựa trên khoảng cách tới vật cản và mục tiêu, từ đó chọn ra tốc độ phù hợp nhất để né tránh linh hoạt.
* `navigator.h`: Máy trạng thái (State Machine) quản lý tiến trình di chuyển của robot (IDLE, TRACKING, DONE, RECOVERING). Có cơ chế recovery: khi bị kẹt sẽ tự động lùi hoặc xoay tại chỗ.
* `frontier_explorer.h`: Thuật toán tự khám phá. Quét trên Grid Map để tìm các điểm ranh giới giữa vùng đã biết và chưa biết (Frontier), tự chỉ định điểm đến để robot vẽ toàn bộ bản đồ mà không cần người điều khiển.

### 6. Giao tiếp mạng (Networking)
* `network_comm.cpp / .h`: Thiết lập HTTP Server và WebSockets. Có logic xử lý Captive Portal (chặn truy cập DNS, trả về địa chỉ IP của ESP32 để ép điện thoại tự mở pop-up đăng nhập WiFi). Gửi nhận tín hiệu điều khiển và đẩy gói dữ liệu Telemetry (Map, Pos, Lidar) liên tục cho trình duyệt hiển thị.

## Logic hoạt động chính yếu (Data Flow)
1. **Lấy mẫu liên tục**: Các tín hiệu Lidar (10Hz) và Encoder/IMU (50Hz) được đọc và ghi vào biến toàn cục.
2. **Ước lượng vị trí (Sensor Fusion)**: Bánh xe và IMU kết hợp (Complementary filter) đưa ra vị trí thô (Odometry).
3. **SLAM & Cập nhật Bản đồ**: SLAM dùng ICP và CSM để sửa lại toạ độ thô của Odometry cho khớp thực tế, rồi vẽ vật cản vào `gridMapper`.
4. **Phản xạ & Tìm đường (DWA/A*)**: Robot tính toán tìm đường bằng A*, dùng DWA bẻ lái tại chỗ với tần số cao để vừa đi tới đích vừa tránh vật cản động.
5. **Giao tiếp UI**: WebSockets đẩy Grid Map và trạng thái hệ thống cho App di động, App gửi lại lệnh tới toạ độ, hoặc lệnh joystick.
