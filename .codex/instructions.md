# AI Unified Instructions (AMR2.0)

> **Lưu ý:** File này gộp chung các quy tắc của `.antigravityrules`, `.claude` và `.codex`. Các AI (Gemini, Claude, Codex) khi làm việc trong dự án này đều phải tuân thủ nghiêm ngặt các hướng dẫn dưới đây.

---

## PHẦN 1: ANTIGRAVITY GLOBAL RULES - SMART CODING AGENT

### 1. Lập Kế Hoạch & Tư Vấn Trước Khi Làm (Plan & Consult First)
- Với MỌI yêu cầu phức tạp (thêm tính năng, thay đổi kiến trúc, chọn công nghệ), PHẢI:
  a) Phân tích yêu cầu và liệt kê các PHƯƠNG ÁN khả thi (ít nhất 2-3 lựa chọn).
  b) Với MỖI phương án, giải thích rõ ràng ưu/nhược điểm khi áp dụng vào dự án của người dùng.
  c) Đề xuất phương án tốt nhất kèm lý do, nhưng để NGƯỜI DÙNG QUYẾT ĐỊNH.
  d) Chỉ bắt tay vào code SAU KHI người dùng chọn phương án.
- Với yêu cầu đơn giản (fix lỗi nhỏ, chỉnh CSS, thêm comment): làm luôn, không cần hỏi.

### 2. Lưu Tiến Độ, Kế Hoạch & Nhật Ký
- Khi bắt đầu task lớn, tự tạo file kế hoạch (`implementation_plan.md`) chứa checklist.
- Cập nhật `task.md` trong quá trình làm.
- **Ghi nhật ký hàng ngày (Journaling):** Sau mỗi khi nhận được yêu cầu hoặc hoàn thành công việc, tự động ghi vào `ai_journals/YYYY-MM-DD.md` chuẩn theo định dạng dòng thời gian chat tối giản (không sử dụng code HTML như font color để tránh rối mắt) như sau:
  ```markdown
  # Nhật ký DD/MM/YYYY

  **<Giờ User gửi>** `<Username>` : <Nội dung yêu cầu>
  **<Giờ AI phản hồi>** `<Tên Agent>` :
  > **Đã sửa file :** <Đường dẫn file tuyệt đối>
  > **Nội dung :** <Tóm tắt ngắn gọn những gì đã sửa>
  ```


### 3. Tích Hợp Obsidian (Second Brain)
- Luôn coi hệ thống Obsidian của người dùng là "Bộ não thứ hai". Hỏi người dùng đường dẫn Obsidian Vault để đọc tài liệu khi cần bối cảnh.
- Code xong logic khó → Đề xuất viết tài liệu Markdown + sơ đồ lưu thẳng vào Obsidian.

### 1.4 Hiểu Trước Khi Làm (Understand Before Act)
- Đọc hiểu cấu trúc (`README.md`, `package.json`, `platformio.ini`...) trước khi code.
- Debug: Tìm ROOT CAUSE (nguyên nhân gốc), không chữa triệu chứng.
- Bắt buộc search web hoặc GitHub để tham khảo khi gặp lỗi lạ/tính năng mới.
- Thao tác chính xác: Dùng `grep_search` tìm chính xác dòng code, không đọc cả file. Giữ nguyên comment cũ.

### 1.5 Test, Xác Minh & Tránh Lặp Lỗi
- Luôn chạy build/test/lint sau khi sửa. Lỗi tự sửa tối đa 2 lần.
- Không lặp lỗi: Sửa lỗi xong → Quét toàn codebase xem có lỗi tương tự không, sửa hết.
- Giao tiếp: Ngắn gọn, súc tích (Bullet point/Table). Báo cáo: Đã làm gì → Kết quả → Còn gì.
- Công cụ thông minh: Tận dụng MCP (GitHub, Supabase, DevTools), chạy song song lệnh cho nhanh.

---

## PHẦN 2: AMR2.0 PROJECT BRAIN & WORKFLOW

You are working in `C:\code2\AMR2.0`, a mixed robotics project:
- `esp32s3xe_v2/`: ESP32-S3 vehicle-brain firmware in C++/Arduino/PlatformIO.
- `src/`: React 19 + Vite + Tauri monitoring/control app.
- `docs/`: TDTU thesis, architecture notes, research notes.
- `scripts/` & `tests/`: automation, benchmarks, regression tests.

### Operating Rules
1. Preserve user work. The worktree is often dirty; never revert unrelated changes.
2. For firmware/controller changes, keep the safe baseline as default.
3. For UI state that must rerender, use React/Zustand hook selectors rather than `.getState()`.
4. For real-robot behavior, firmware telemetry is the source of truth.
5. For thesis/report artifacts, follow MauDATN_2021 style (Python 3.10+, C++ Arduino, JS ES6+).

### Required Checks & Verifications
- Run the narrowest meaningful verification after edits:
  - Frontend/app: `npm run test`, `npm run build`, or `npm run check`.
  - Trajectory benchmark: `npm run benchmark:trajectory`.
  - Firmware: `python -m platformio run` from `esp32s3xe_v2`.
- Docs/config-only: validate JSON/Markdown shape and run `git diff --check`.

### GitNexus & Code Intelligence
- **MUST run impact analysis before editing any symbol.**
- **MUST run `gitnexus_detect_changes()` before committing**.
- Warn the user if impact analysis returns HIGH or CRITICAL risk.
- NEVER rename symbols with find-and-replace — use `gitnexus_rename`.
- When exploring unfamiliar code, use `gitnexus_query`. When needing full context, use `gitnexus_context`.

## Important Local Docs
- `README.md`, `task.md`, `esp32s3xe_v2/ARCHITECTURE.md`
- `docs/02_Architecture/VEHICLE_BRAIN_ARCHITECTURE.md`
- `docs/01_TDTU_Thesis/PROJECT_REPORT.md`