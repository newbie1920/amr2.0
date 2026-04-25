## Tài liệu Hướng dẫn Thiết kếBộđiều khiển DDWMR Trajectory Tracking trên phần cứng ESP32 

## TS. Vũ Trí Viễn 

## Ngày 15 tháng 4 năm 2026 

## **Mục lục** 

|**1**|**Tổng quan Bài toán Thực tế**|**2**|
|---|---|---|
|**2**|**Nền tảng Toán học và Động học**|**2**|
||2.1<br>Mô hình Động học DDWMR<br>. . . . . . . . . . . . . . . . . . . . . . . .|2|
||2.2<br>Tạo Quỹđạo Tham chiếu<br>. . . . . . . . . . . . . . . . . . . . . . . . . .|2|
|**3**|**Lý thuyết Điều khiển Phân tầng**|**2**|
||3.1<br>Điều khiển Quỹđạo: Backstepping (High-level)<br>. . . . . . . . . . . . . .|2|
|**4**|**Điều khiển Động cơ (Low-level): Khâu PID**|**5**|
||4.1<br>Luật điều khiển PID . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|5|
||4.2<br>Chống bão hòa tích phân (Anti-windup) . . . . . . . . . . . . . . . . . .|5|
|**5**|**Định vịvà Dung hợp Cảm biến (Sensor Fusion)**|**5**|
||5.1<br>Tính toán Odometry (Định vịtương đối) . . . . . . . . . . . . . . . . . .|5|
||5.2<br>Dung hợp cảm biến với IMU MPU6050 . . . . . . . . . . . . . . . . . . .|6|
|**6**|**Thực thi Phần cứng trên ESP32**|**6**|
|**7**|**Kết luận và Lưu ý Triển khai**|**10**|



1 

## **1 Bài toán tế Tổng quan Thực** 

Bài toán đặt ra là điều khiển một Robot di động hai bánh chủđộng (Differential Drive Wheeled Mobile Robot - DDWMR) bám theo một quỹđạo cho trước. Trong môi trường thực tế, hệthống phải đối mặt với các vấn đề: 

- **Động học phi tuyến:** Mối quan hệgiữa vận tốc động cơ và tọa độkhông gian của robot là phi tuyến. 

- **Nhiễu hệthống và đo lường:** Cảm biến encoder có thểbịsai sốdo trượt bánh xe, trong khi động cơ chịu ảnh hưởng của ma sát và biến thiên tải trọng. 

Đểgiải quyết, tài liệu này đềxuất cấu trúc điều khiển phân tầng (Hierarchical Control): Tầng cao sửdụng thuật toán **Backstepping** đểtriệt tiêu sai sốquỹđạo; Tầng thấp sử dụng **LQG** đểđiều khiển momen động cơ; Đồng thời sửdụng **Odometry và EKF** kết hợp IMU đểđịnh vịchính xác. 

## **2 Nền tảng Toán học và Động học** 

## **2.1 Mô hình Động học DDWMR** 

Phương trình trạng thái của Robot được mô tảbằng vector _q_ = [ _x, y, θ_ ] _[T]_ : 

**==> picture [295 x 44] intentionally omitted <==**

Trong đó, _v_ và _ω_ lần lượt là vận tốc dài và vận tốc góc của robot. 

## **2.2 Tạo Quỹđạo Tham chiếu** 

Đểbộđiều khiển bám hoạt động mượt mà, cần cung cấp tọa độmong muốn ( _xd, yd_ ) cùng vận tốc tham chiếu tương ứng _vd, ωd_ . Các giá trịnày được tính từđạo hàm của phương trình quỹđạo: 

**==> picture [319 x 56] intentionally omitted <==**

Việc tính toán sẵn _ωd_ giúp hệthống triệt tiêu sai sốhướng _eθ_ nhanh hơn thông qua thuật ngữfeedforward. Tại các điểm vận tốc bằng 0, giá trị _ωd_ có thểbịbất định, do đó cần thêm một lượng nhỏ _ϵ_ vào mẫu sốkhi lập trình. 

## **3 Lý thuyết Điều khiển Phân tầng** 

## **3.1 Điều khiển Quỹđạo: Backstepping (High-level)** 

Sai sốbám trong hệtọa độđịa phương được định nghĩa là: 

**==> picture [333 x 44] intentionally omitted <==**

2 

Sửdụng hàm Lyapunov _V_ =[1] 2[(] _[e] x_[2][+] _[ e]_[2] _y_[) +][1] _[−]_[cos] _ky[ e][θ]_ , luật điều khiển đềxuất để _V_[˙] xác định âm là: 

**==> picture [299 x 30] intentionally omitted <==**

Chi tiết thiết kếbộđiều khiển như sau 

## **Bước 1: Mô hình hóa hệthống và quỹđạo tham chiếu** 

Đầu tiên, ta cần có phương trình toán học mô tảcách robot di chuyển và cách quỹđạo mẫu di chuyển. 

- **Mô hình động học của Robot:** 

**==> picture [276 x 135] intentionally omitted <==**

- **Quỹđạo tham chiếu (Reference):** (Ví dụnhư một con robot ảo chạy chuẩn trên đường) 

## **Bước 2: Định nghĩa sai sốbám (Tracking Error)** 

Ta không tính sai sốtrên hệtọa độtoàn cục (Global) mà phải nhân với ma trận quay (Rotation Matrix) đểchuyển sai sốvềhệtọa độcục bộgắn trên robot (Local Frame). Điều này giúp bộđiều khiển biết chính xác mục tiêu đang nằm ởhướng nào so với mũi robot. 

**==> picture [323 x 44] intentionally omitted <==**

## **Bước 3: Tìm động lực học của sai số(Error Dynamics)** 

Lấy đạo hàm hai vếcủa phương trình sai sốởBước 2 theo thời gian, kết hợp với các phương trình ởBước 1, ta thu được phương trình vi phân mô tảsựbiến thiên của sai số: 

**==> picture [299 x 51] intentionally omitted <==**

_Nhiệm vụbây giờlà phải tìm các giá trịđầu vào v và ω sao cho ex, ey, eθ tiến về0 khi thời gian t →∞._ 

3 

## **Bước 4: Lựa chọn hàm ứng viên Lyapunov** 

Đểhệthống ổn định, ta chọn một hàm năng lượng giảđịnh _V ≥_ 0. Hàm này thường được chọn dưới dạng: 

**==> picture [298 x 29] intentionally omitted <==**

Trong đó _ky >_ 0 là hệsốthiết kế. Lý do dùng (1 _−_ cos _eθ_ ) là vì khi _eθ_ = 0 thì phần này bằng 0, và nó luôn dương khi _eθ_ = 0 trong khoảng hẹp. 

**Bước 5: Thiết kếluật điều khiển đểép** _V_[˙] _≤_ 0 

Lấy đạo hàm của hàm _V_ theo thời gian: 

**==> picture [294 x 28] intentionally omitted <==**

Thay các phương trình động lực học sai số(ởBước 3) vào _V_[˙] : 

**==> picture [397 x 29] intentionally omitted <==**

Khai triển và triệt tiêu cặp _exωey − eyωex_ = 0, ta gom lại được: 

**==> picture [359 x 28] intentionally omitted <==**

Bây giờ, ta chọn luật điều khiển ( _v, ω_ ) sao cho triệt tiêu các thành phần gây mất ổn định và làm _V_[˙] mang dấu âm: **Chọn vận tốc dài** _v_ 

**==> picture [312 x 13] intentionally omitted <==**

Thay vào _V_[˙] , phần đầu tiên sẽbiến thành: _ex_ ( _−kxex_ ) = _−kxe_[2] _x_[.] **Chọn vận tốc góc** _ω_ Lúc này _V_[˙] còn lại: 

**==> picture [330 x 29] intentionally omitted <==**

Rút[sin] _ky[ e][θ]_ làm nhân tửchung cho 2 vếsau: 

**==> picture [322 x 29] intentionally omitted <==**

Đểtriệt tiêu cụm trong ngoặc và tạo ra sốâm, ta thiết kế: 

**==> picture [331 x 13] intentionally omitted <==**

**Kết luận** Thay lại luật điều khiển _ω_ vào _V_[˙] , ta được kết quảcuối cùng: 

**==> picture [297 x 29] intentionally omitted <==**

Vì _V_[˙] _≤_ 0 (Hàm bán xác định âm - Negative Semi-Definite), theo lý thuyết Lyapunov (và kết hợp BổđềBarbalat đểxửlý triệt để _ey_ ), hệthống sẽổn định tiệm cận toàn cục, nghĩa là sai số [ _ex, ey, eθ_ ] _[T] →_ 0 khi _t →∞_ . 

4 

## **4 Điều khiển Động cơ (Low-level): Khâu PID** 

Đểcác bánh xe bám theo vận tốc tham chiếu do tầng Backstepping yêu cầu, một bộđiều khiển PID (Proportional-Integral-Derivative) được sửdụng độc lập cho mỗi động cơ. 

## **4.1 Luật điều khiển PID** 

Tín hiệu điều khiển _u_ ( _t_ ) được tính dựa trên sai sốvận tốc _e_ ( _t_ ) = _ωref_ ( _t_ ) _− ωmeas_ ( _t_ ): 

**==> picture [330 x 30] intentionally omitted <==**

Dạng rời rạc được triển khai trên vi điều khiển: 

**==> picture [328 x 35] intentionally omitted <==**

## **4.2 Chống bão hòa tích phân (Anti-windup)** 

Trong thực tế, động cơ có giới hạn điện áp (ví dụ _±_ 12 _V_ ). Nếu sai sốduy trì lâu, khâu tích phân sẽtăng lên rất lớn (windup), làm hệthống mất ổn định. Đểkhắc phục, thành phần[�] _ei_ ∆ _t_ được kẹp (clamp) trong một khoảng giới hạn an toàn. 

1 class WheelPID { 2 private: 3 float Kp = 1.5, Ki = 0.2, Kd = 0.05; // ầCn tuning ựthc ết 4 float error_sum = 0; 5 float last_error = 0; 6 const float dt = 0.02; 7 8 public: 9 float update(float omega_meas, float omega_ref) { 10 float error = omega_ref - omega_meas; 11 12 error_sum += error * dt; 13 error_sum = constrain(error_sum, -50.0, 50.0); // Anti-windup 14 15 float d_error = (error - last_error) / dt; 16 float u_out = Kp * error + Ki * error_sum + Kd * d_error; 17 18 last_error = error; 19 return constrain(u_out, -12.0, 12.0); // Voltage saturation 20 } 21 }; 

Listing 1: Triển khai bộđiều khiển PID cho động cơ bánh xe 

## **5 Định vịvà Dung hợp Cảm biến (Sensor Fusion)** 

## **5.1 Tính toán Odometry (Định vịtương đối)** 

Quy trình tính toán tích phân quãng đường dựa trên sốxung encoder: 

5 

**==> picture [288 x 17] intentionally omitted <==**

**==> picture [244 x 15] intentionally omitted <==**

**==> picture [212 x 16] intentionally omitted <==**

Cập nhật trạng thái tại bước thời gian _k_ + 1: 

**==> picture [305 x 62] intentionally omitted <==**

**==> picture [304 x 13] intentionally omitted <==**

## **5.2 Dung hợp cảm biến với IMU MPU6050** 

Hạn chếlớn nhất của Odometry là sai sốtích lũy do trượt bánh xe. Gyroscope (IMU) không bịảnh hưởng bởi hiện tượng trượt nhưng lại bịtrôi (drift) theo thời gian. 

**Phương pháp 1: Complementary Filter.** Góc xoay _θ_ được cập nhật đơn giản qua bộlọc bù: 

**==> picture [327 x 15] intentionally omitted <==**

Với _α ∈_ (0 _._ 95 _,_ 1), phương án này loại trừnhiễu tần sốcao từencoder và độtrôi tần số thấp của gyroscope. 

**Phương pháp 2: Extended Kalman Filter (EKF).** Tuyến tính hóa mô hình phi tuyến thông qua Jacobian _Fk_ = _[∂] ∂x[f]_[và] _[H][k]_[=] _[∂h] ∂x_[.][EKF][dung][hợp][theo][2][bước:] 

- _Dựđoán (Encoder):_ Dựđoán trạng thái mới dựa trên dịch chuyển bánh xe. 

- _Cập nhật (IMU):_ Hiệu chỉnh góc xoay _θ_ sửdụng dữliệu gyroscope, qua đó thu nhỏ kích thước ma trận hiệp phương sai _P_ . 

## **6 Thực thi Phần cứng trên ESP32** 

Dưới đây là mã nguồn C++ thực thi trên ESP32. Các cấu trúc dữliệu đã được bổsung đầy đủ, và chú thích được viết bằng tiếng Anh đểđảm bảo chuẩn hóa lập trình nhúng. 

1 #include <Arduino.h> 2 #include <cmath> 

3 

4 // 1. Robot physical and control parameters 5 struct RobotConfig { 6 const double r = 0.05; // Wheel radius (meters) 7 const double b = 0.5; // Track width (meters) 8 const double Kx = 2.0; // Backstepping gain for X-error 9 const double Ky = 5.0; // Backstepping gain for Y-error 10 const double Kth = 1.5; // Backstepping gain for Thetaerror 11 const double dt = 0.02; // Sampling time (20ms) 12 }; 

13 

6 

14 struct State { double x, y, th; }; 15 RobotConfig cfg; 16 17 // 2. Trajectory Generation Struct 18 struct TrajectoryPoint { 19 double x, y, theta, v, omega; 20 double vx, vy, ax, ay; // Internal derivatives 21 }; 22 23 // Generate a figure-8 trajectory (Lemniscate of Gerono) 24 TrajectoryPoint getFigure8(double t) { 25 TrajectoryPoint p; 26 double A = 1.2; // Amplitude 27 double w = 0.4; // Orbital frequency 28 29 p.x = A * sin(w * t); 30 p.y = A * sin(w * t) * cos(w * t); 31 32 p.vx = A * w * cos(w * t); 33 p.vy = A * w * (cos(w * t) * cos(w * t) - sin(w * t) * sin(w * t)); 

34 35 36 37 38 39 

40 41 

42 43 44 45 46 47 48 49 50 51 52 

53 54 55 56 57 58 

59 

60 

= p.theta atan2(p.vy, p.vx); p.v = sqrt(p.vx * p.vx + p.vy * p.vy); return p; } 

// Generate a circular trajectory TrajectoryPoint generate_circular(double t, double R = 1.0, double w_orbit = 0.5) { TrajectoryPoint p; 

// Position: x = R*cos(w*t), y = R*sin(w*t) p.x = R * std::cos(w_orbit * t); p.y = R * std::sin(w_orbit * t); 

// Velocity derivatives: vx = -R*w*sin(w*t), vy = R*w*cos(w*t) p.vx = -R * w_orbit * std::sin(w_orbit * t); p.vy = R * w_orbit * std::cos(w_orbit * t); 

// Acceleration derivatives: ax = -R*w^2*cos(w*t), ay = -R*w ^2*sin(w*t) p.ax = -R * w_orbit * w_orbit * std::cos(w_orbit * t); p.ay = -R * w_orbit * w_orbit * std::sin(w_orbit * t); 

// Desired heading = p.theta std::atan2(p.vy, p.vx); 

// Linear velocity: vd = sqrt(vx^2 + vy^2) p.v = std::sqrt(p.vx * p.vx + p.vy * p.vy); 

61 

7 

62 // Angular velocity: wd = (vx*ay - vy*ax) / v^2 63 p.omega = (p.vx * p.ay - p.vy * p.ax) / (p.v * p.v + 1e-9); 

64 

65 return p; 66 } 

67 

68 // 3. Low-level Optimal Control 69 #include <Arduino.h> 70 // Class PID Controller cho ừtng bánh xe 71 class WheelPID { 72 private: 73 float Kp, Ki, Kd; 74 float error_sum = 0.0; 75 float last_error = 0.0; 76 float dt = 0.02; // Chu ỳk ấly ẫmu (20ms) 

77 

78 // ớGii ạhn tích phân (Anti-windup) 79 const float MAX_INTEGRAL = 50.0; 

80 

81 public: 82 WheelPID(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {} 

83 

84 float update(float omega_meas, float omega_ref) { 85 // 1. Error Calculation 86 float error = omega_ref - omega_meas; 

87 

88 // 2. Integral with Anti-windup 89 error_sum += error * dt; 90 if (error_sum > MAX_INTEGRAL) error_sum = MAX_INTEGRAL; 91 if (error_sum < -MAX_INTEGRAL) error_sum = -MAX_INTEGRAL; 

92 

93 // 3. Derivative 94 float d_error = (error - last_error) / dt; 

95 

96 // 4. Control signal 97 float u_out = (Kp * error) + (Ki * error_sum) + (Kd * d_error); 

98 

99 // Save error 100 last_error = error; 101 

102 // Voltage saturation -12V to 12V 103 return constrain(u_out, -12.0, 12.0); 

104 105 

} 

106 // Reset 107 void reset() { 108 error_sum = 0.0; 109 last_error = 0.0; 

110 

111 

} 

}; 

8 

112 113 // 4. Main Control Task (FreeRTOS Task) 114 void controlTask(void *pv) { 115 TickType_t xLastWakeTime = xTaskGetTickCount(); 

116 

117 //Initiallize the PID (Kp, Ki, Kd) 118 WheelPID leftMotorPID(1.5, 0.2, 0.05); // must be tuned 119 WheelPID rightMotorPID(1.5, 0.2, 0.05); 

- 120 

121 State current = {0, 0, 0}; 

- 122 

123 for (;;) { 

124 double t = millis() / 1000.0; 

125 TrajectoryPoint ref = getFigure8(t); // or generate_circular 

(t) 

126 

127 // --- High-level: Backstepping Control --128 double dx = ref.x - current.x; 129 double dy = ref.y - current.y; 

130 double eth = atan2(sin(ref.theta - current.th), cos(ref. theta - current.th)); 

131 

- 132 double ex = dx * cos(current.th) + dy * sin(current.th); 133 double ey = -dx * sin(current.th) + dy * cos(current.th); 

- 134 

- 135 double v_cmd = ref.v * cos(eth) + cfg.Kx * ex; 

136 double w_cmd = ref.omega + ref.v * (cfg.Ky * ey + cfg.Kth * sin(eth)); 

137 

- 138 // --- Inverse Kinematics --- 

- 139 double wL_ref = (v_cmd - (cfg.b/2)*w_cmd) / cfg.r; 140 double wR_ref = (v_cmd + (cfg.b/2)*w_cmd) / cfg.r; 

- 141 

142 

143 

- // --- Low-level: Execute LQG --- 

- // --- Low-level: Execute PID --- 

144 // Assume getEncoderVelocityL() to measure angular speed ( rad/s) 

145 

- 146 float omega_meas_L = getEncoderVelocityL(); 147 float omega_meas_R = getEncoderVelocityR(); 

- 148 

- 149 float uL = leftMotorPID.update(omega_meas_L, wL_ref); 150 float uR = rightMotorPID.update(omega_meas_R, wR_ref); 

151 

- 152 vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); 

- 153 

- 154 

} 

- } 

155 

156 void setup() { 157 xTaskCreate(controlTask, "ControlTask", 4096, NULL, 10, NULL); 158 } 

9 

159 160 void loop() {} 161 

Listing 2: ESP32 implementation for Backstepping, LQG, and Trajectory Generation 

## **7 Kết luận và Lưu ý Triển khai** 

Các lưu ý quan trọng trong quá trình lập trình phần cứng: 

- **Định thời (Timing):** Bắt buộc sửdụng RTOS (cụthểlà vTaskDelayUntil trong FreeRTOS) đểgiữchu kỳlấy mẫu _dt_ ổn định (20 _ms_ ). Bất kỳđộtrễ(jitter) nào cũng sẽlàm sai lệch bộlọc Kalman và Odometry. 

- **Chuẩn hóa đơn vịđo đạc:** Tất cảthông sốphải sửdụng chuẩn SI (mét, radian, giây). Xung encoder (ticks) cần được chuyển đổi sang _rad_ / _s_ trước khi đưa vào hàm tính toán của động cơ. 

- **Bão hòa ngõ ra (Actuator Saturation):** Tín hiệu điều khiển _uvolt_ phải luôn được kẹp giới hạn theo ngưỡng điện áp nguồn của driver động cơ, phòng tránh hiện tượng windup: 

   - _uout_ = max( _−_ 12 _,_ min(12 _, u_ )) (26) 

- **Hiệu chuẩn Cảm biến (Calibration):** Trạng thái đứng yên đầu tiên sau khi khởi động phải được dùng đểxác định sai sốtĩnh (offset) cho Gyroscope nhằm chống hiện tượng drift hệthống. 

10 

