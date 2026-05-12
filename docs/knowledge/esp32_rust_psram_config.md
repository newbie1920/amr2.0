# Cấu hình biên dịch Rust cho ESP32-S3 (Bật PSRAM & Tránh lỗi Windows Path)

**Ngày ghi nhận:** 06/05/2026
**Dự án:** AMR 2.0 (Navigation Firmware)
**Nền tảng:** ESP32-S3 (có PSRAM 8MB) + Rust (esp-idf-sys)

---

## 1. Vấn đề cốt lõi đã giải quyết
- **Lỗi tràn RAM (OOM - Out Of Memory):** Khi sử dụng thuật toán Theta* và mảng `OccupancyGrid` lớn (>640KB), ESP32-S3 bị khởi động lại liên tục. Nguyên nhân do bộ nhớ nội (SRAM) chỉ còn khoảng ~200KB sau khi chạy WiFi và FreeRTOS.
- **Lỗi không nhận cấu hình PSRAM:** Mặc dù bo mạch có 8MB PSRAM (SPIRAM), nhưng khi biên dịch, `esp-idf-sys` bỏ qua file `sdkconfig.defaults` do đường dẫn relative (`relative = true`) hoạt động sai trong môi trường Windows.
- **Lỗi đường dẫn quá dài (MAX_PATH):** Hệ thống Windows giới hạn 260 ký tự đường dẫn, làm crash tiến trình biên dịch CMake/Ninja của bộ ESP-IDF C-core.

## 2. Giải pháp & Cách cấu hình

### A. Fix lỗi MAX_PATH và thư mục Build
Luôn luôn định tuyến thư mục build của Rust ra ổ đĩa ngắn (vd: `C:\target`).
- **Lệnh biên dịch tiêu chuẩn:**
  ```powershell
  $env:CARGO_TARGET_DIR="C:\target"; cargo build --release
  ```

### B. Fix cấu hình `.cargo/config.toml` (Quan trọng nhất)
Để đảm bảo `embuild` (công cụ build ESP-IDF) luôn tìm thấy `sdkconfig.defaults` trên Windows, **phải dùng đường dẫn tuyệt đối (absolute path)** thay vì `relative = true`. Đồng thời, gán luôn port COM của ESP32 để tránh lỗi `not a terminal`.

```toml
[target.'cfg(target_os = "espidf")']
linker = "ldproxy"
# Gán sẵn port COM để `cargo run` có thể tự động flash & monitor
runner = "espflash flash --monitor --port COM3"
rustflags = [ "--cfg",  "espidf_time64"]

[env]
MCU = "esp32s3"
ESP_IDF_VERSION = "v5.5.3"
ESP_IDF_TOOLS_INSTALL_DIR = "workspace"

# BẮT BUỘC dùng đường dẫn tuyệt đối trên Windows để nhận diện file sdkconfig.defaults
ESP_IDF_SDKCONFIG_DEFAULTS = { value = "C:\\code2\\AMR2.0\\esp32s3xe-rust\\sdkconfig.defaults" }
CARGO_WORKSPACE_DIR = { value = "C:\\code2\\AMR2.0\\esp32s3xe-rust" }
```

### C. File `sdkconfig.defaults` cần thiết lập
Để bật tính năng sử dụng PSRAM trên module N16R8 hoặc tương đương:

```text
CONFIG_ESP32S3_SPIRAM_SUPPORT=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y

# Cho phép cấp phát tự động Malloc vào PSRAM
CONFIG_SPIRAM_USE_MALLOC=y
# Các vùng nhớ lớn hơn mức này (KB) sẽ bị đẩy qua PSRAM
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
```

## 3. Quy trình làm sạch và build lại (Full Clean)
Nếu sửa file `sdkconfig.defaults`, cache của `esp-idf-sys` phải bị xóa thì cấu hình mới mới có tác dụng.
```powershell
# 1. Xóa cache của thư viện esp-idf-sys
Remove-Item -Recurse -Force C:\target\xtensa-esp32s3-espidf\release\build\esp-idf-sys*

# 2. Xóa các tiến trình đang bị kẹt (nếu có)
Stop-Process -Name "build-script-build", "cargo", "ninja", "cmake" -Force -ErrorAction SilentlyContinue

# 3. Biên dịch lại từ đầu
$env:CARGO_TARGET_DIR="C:\target"; cargo run --release
```

## 4. Dấu hiệu nhận biết thành công trên Serial Monitor
Nếu PSRAM được kích hoạt thành công, log boot của ESP32 sẽ hiển thị:
```log
I (372) octal_psram: vendor id    : 0x0d (AP)
I (415) esp_psram: Found 8MB PSRAM device
I (921) esp_psram: Adding pool of 8192K of PSRAM memory to heap allocator
I (972) esp_psram: Reserving pool of 64K of internal memory for DMA/internal allocations
```
Sau đó, các mảng lớn (như `OccupancyGrid` ~640KB) trong Rust khi sử dụng `Vec::new()` sẽ tự động được hệ thống trỏ sang PSRAM, hệ thống không còn bị OOM panic.
