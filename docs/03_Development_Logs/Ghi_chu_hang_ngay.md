# Nhật ký Phát triển & Fix Bug

Sử dụng file này để ghi chép các lỗi gặp phải và cách giải quyết. Nó sẽ rất hữu ích khi viết phần "Khó khăn và Cách khắc phục" trong báo cáo.

## [2026-04-28] Fix lỗi Layout RViz Dashboard
- **Vấn đề**: Khi bấm spawn robot, phần `sidebar-left` bị chèn ép, các panel RViz và Warehouse map không full được chiều cao/rộng.
- **Giải pháp**: Đã refactor lại CSS sử dụng Flexbox/Grid, xóa các background thừa và chia tỷ lệ màn hình chuẩn xác cho chế độ Split-screen.

## [2026-04-25] Khắc phục nhiễu Serial Monitor trên ESP32
- **Vấn đề**: In ra toàn ký tự rác (junk) hoặc ô vuông khi debug ESP32-S3.
- **Giải pháp**: Xử lý lại Baudrate và cấu hình USB CDC.

## ... (Viết tiếp các bug ở đây) ...
