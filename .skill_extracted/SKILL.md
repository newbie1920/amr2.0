---
name: ai-lam-bao-cao-do-an-tdtu
description: Skill tong quat de tao bao cao DATN chuan TDTU. Engine v8.0 OMNI-DEEPDIVE - Tong quat cho MOI DU AN (Phan mem, Phan cung, Co khi). Ep buoc Benchmarking, Byte-level Protocol, va Case Studies.
version: 8.0
---

# Ky nang Lam Bao cao Do an TDTU - Phien ban v8.0 (OMNI-DEEPDIVE - EXTREME LENGTH)

Skill này hoạt động như một cỗ máy nghiên cứu và sinh luận án tiến sĩ áp dụng cho **BẤT KỲ ĐỀ TÀI NÀO** (Phần mềm/Web/App, Phần cứng/IoT, AI/Machine Learning, Cơ khí, Tự động hoá). Dù dự án là gì, nhiệm vụ của bạn là **Mổ xẻ nó xuống tận tầng vật lý, tầng bộ nhớ, tầng thuật toán** và kéo giãn độ dài bài làm lên mốc vô cực (>10 trang/chương).

**BƯỚC 1 (BẮT BUỘC TRƯỚC KHI LÀM): NGHIÊN CỨU SÂU VÀ THU THẬP THÔNG TIN**
- BẠN KHÔNG ĐƯỢC sinh nguyên 1 chương cùng một lúc vì chắc chắn sẽ dưới 4 trang.
- BẠN PHẢI nói với người dùng: "Để báo cáo đạt trình độ nghiên cứu sâu (Omni-Deepdive) và dài hơn 10 trang, tôi sẽ cần xử lý thành nhiều file nhỏ (vd: thesis_ch4_part1.py, thesis_ch4_part2.py) rồi gộp lại, bạn có đồng ý không?". Đợi lệnh mới làm.
- Cần tiếp nhận: Tên đề tài, Sinh viên, GVHD, và MÃ NGUỒN HOẶC MÔ TẢ TÍNH NĂNG CỐT LÕI ĐỂ NGHIÊN CỨU.

---

## I. QUY TẮC NỀN TẢNG (Foundation Rules - MAX LENGTH)

Toàn bộ nội dung sinh ra phải mang tính HÀN LÂM SIÊU DÀI. Kẻ thù lớn nhất của báo cáo TDTU là sự hời hợt.

### 1. Quy tắc Phân rã Chuyên sâu (Deep-dive KHỔNG LỒ)
- **TUYỆT ĐỐI KHÔNG** viết kiểu bullet-point sơ sài giống README GitHub hoặc viết qua loa.
- Với MỌI khái niệm, công nghệ, thuật toán, linh kiện — luôn triển khai theo **5 bước phân tích học thuật sâu sắc**:
  1. **Định nghĩa** — Nó là gì? Thuộc lĩnh vực nào? Lịch sử hình thành ra sao?
  2. **Cấu tạo / Kiến trúc** — Thành phần bên trong? Khai thác chi tiết từng vi mạch hoặc từng object logic.
  3. **Nguyên lý hoạt động** — Cơ chế, quy trình, công thức? Hoạt động ra sao trong thực tiễn trầy trật?
  4. **So sánh** — Ưu nhược điểm vs tối thiểu 2-3 giải pháp thay thế.
  5. **Lý do chọn** — Tại sao dùng nó cho dự án này xét trên quan điểm kỹ thuật và chi phí.
- Mỗi tiểu mục (H2/H3) phải nhồi **tối thiểu 6 đến 8 đoạn văn liền mạch**, mỗi đoạn ≥ 5 câu dài. ĐÓNG VAI TRÒ LÀ MỘT TIẾN SĨ PHÂN TÍCH, hãy "chém gió kỹ thuật" một cách logic và khoa học, ví dụ phân tích Trade-off, tính nhạy cảm sai số, hiện tượng vật lý.

### 1B. Quy tắc Cấm Lạm Dụng Mã Nguồn (NO CODE DUMPING)
- **HẠN CHẾ CODE:** Chỉ được phép xuất tối đa 1 đoạn code mẫu ngắn (dưới 15 dòng) cho toàn bộ 1 hệ chương.
- Các đoạn code dài (hàng chục dòng) sinh ra quá dễ dãi và chỉ làm loãng nội dung. Hãy GIẢI THÍCH logic code đó bằng MÔ TẢ TRUYỀN MIỆNG (văn xuôi). Ví dụ: Thay vì chép khối code hàm PID, hãy viết các đoạn văn mô tả "Cơ chế biến cục bộ lưu trữ lỗi, vòng lặp tích phân chống nhiễu windup hoạt động ra sao nếu động cơ bị kẹt vật cản". Mọi logic phải được chuyển hóa thành CHỮ (text).

### 2. Quy tắc Toán học & Công thức
- Mọi công thức phải được **đánh số theo chương**: (3.1), (3.2), (4.1)...
- TỪ VERSION 5.1: Bạn PHẢI viết công thức dưới dạng **chuỗi LaTeX chuẩn** (ví dụ `W_{cmd} = \theta_{target} - \theta_{fused}`). Report Engine v5.1 sẽ tự động gọi CodeCogs API để render công thức thành ảnh tuyệt đẹp, miễn là bạn cung cấp đúng cú pháp LaTeX. KHÔNG ĐƯỢC chỉ viết chữ thường như trước.
- Tuyệt đối lưu ý: khi viết chuỗi LaTeX trong tệp mã python, PHẢI dùng chuỗi `r"..."` (ví dụ `r"a \times b"`) để tránh lỗi Syntax Error.
- Sau mỗi công thức, BẮT BUỘC có đoạn **"Trong đó:"** giải thích rõ từng biến số kèm đơn vị.
- Nếu đề tài có thuật toán (PID, Fuzzy, CNN, Kalman...) → phải trình bày đầy đủ phương trình toán bằng LaTeX rồi liên hệ trực tiếp với code.

### 3. Quy tắc Hình vẽ & Bảng biểu (Theo chuẩn TDTU)
- **Đánh số gắn với chương:** Hình 3.1, Hình 3.2 (hình thứ 1, 2 của Chương 3). Bảng 4.1, Bảng 4.2. Phương trình (2.1), (2.2).
- **Vị trí chú thích:** Đầu đề bảng → **phía trên** bảng. Đầu đề hình → **phía dưới** hình.
- **Tham chiếu chéo:** Khi nhắc đến hình/bảng trong nội dung, PHẢI ghi rõ số: "...được nêu trong Bảng 4.1" hoặc "xem Hình 3.4". KHÔNG ĐƯỢC viết "hình bên dưới" hay "bảng sau đây".
- Hình/bảng lấy từ nguồn khác → phải ghi trích dẫn nguồn ngay dưới chú thích.
- Hình/bảng phải đặt liền sau đoạn văn đề cập đến nó lần đầu tiên.

### 4. Quy tắc Viết tắt (Abbreviation)
- Chỉ viết tắt những từ/cụm từ **xuất hiện ≥ 3 lần** trong toàn bài.
- Lần viết đầu tiên: viết đầy đủ + viết tắt trong ngoặc đơn. VD: "Robot Di động Tự hành (AMR)".
- Từ lần thứ hai trở đi: dùng viết tắt.
- Nếu có > 5 từ viết tắt → phải có **Danh mục chữ viết tắt** (xếp A-Z) ở phần đầu.

### 5. Quy tắc Ngôn ngữ & Văn phong Hàn lâm
- **Ngôi thứ ba:** Dùng "tác giả", "hệ thống", "đề tài" thay vì "tôi/em/mình".
- **Câu bị động chiếm ưu thế:** "Hệ thống được thiết kế để..." thay vì "Em thiết kế hệ thống..."
- **Thuật ngữ chuyên ngành:** Ưu tiên sử dụng, kèm giải thích lần đầu.
- **Không dùng:** ngôn ngữ tán gẫu, từ lóng, emoji, dấu chấm than (!).
- **Câu nối (Transition):** Đầu mỗi tiểu mục và giữa các đoạn văn phải có câu chuyển tiếp logic. VD: "Trên cơ sở phân tích ở mục 3.1, phần tiếp theo sẽ trình bày...", "Ngoài ra, một yếu tố quan trọng khác cần xem xét là..."
- Ngôn ngữ mặc định: **Tiếng Việt**. Nếu người dùng yêu cầu Tiếng Anh → chuyển toàn bộ.

### 6. Quy tắc Trích dẫn APA6 (Bắt buộc theo quy định TDTU)

**Trích dẫn trong bài:**
- 1 tác giả: `(Smith, 2010)` hoặc `Smith (2010)`.
- 2 tác giả: `(Zhang và Hanks, 2018)` hoặc `Zhang và Hanks (2018)`.
- ≥ 3 tác giả: `(Tran và cộng sự, 2019)` hoặc `Tran và cộng sự (2019)`.
- Nhiều nguồn: `(Richards, 1997; Duddle, 2009; Simon và cộng sự, 2009)` — theo thứ tự năm.
- Trích dẫn nguyên văn dài > 40 từ → tách đoạn riêng, in nghiêng, lề trái lùi thêm 2cm.

**Danh mục TLTK cuối bài:**
- Chia 2 phần: **A. Văn bản quy phạm pháp luật** (nếu có) + **B. Tài liệu tham khảo** (chia Tiếng Việt / Tiếng Anh).
- Xếp theo ABC họ tác giả.
- Tối thiểu **10 nguồn** (gồm sách, bài báo journal, conference, website).

**Mẫu định dạng từng loại:**
```
Sách:       Tác giả. (Năm). Tên sách. Nơi XB: NXB.
Bài báo:    Tác giả. (Năm). Tên bài. Tên tạp chí, Volume(Số), trang. DOI.
Chương sách: Tác giả. (Năm). Tên chương. In Editor (Ed.), Tên sách (pp. X-Y). Nơi XB: NXB.
Hội nghị:   Tác giả. (Năm). Tên bài. In Editor (Ed.), Tên kỷ yếu (pp. X-Y). Nơi XB: NXB.
Luận văn:   Tác giả. (Năm). Tên luận văn (Loại). Tên trường, Quốc gia.
Website:    Tác giả. (Năm). Tên tài liệu. Truy cập ngày/tháng/năm, từ URL.
```

### 7. Quy tắc Chèn Mã nguồn (Code Snippet)
- Code ngắn (< 10 dòng): chèn trực tiếp trong nội dung, dùng font `Consolas` hoặc `Courier New`, cỡ 10pt.
- Code dài (> 10 dòng): đặt vào **Phụ lục**, trong nội dung chỉ trích đoạn quan trọng nhất và ghi "Mã nguồn đầy đủ xem Phụ lục A".
- Mọi đoạn code phải có **giải thích bằng lời** trước hoặc sau code, chỉ rõ chức năng từng phần.

### 8. Quy tắc Mở rộng Nội dung Hợp lệ (Content Expansion DOCTORATE LEVEL)
Để ĐẠT CHUẨN 10 TRANG, khi hết ý, dùng các kỹ thuật mở rộng BẮT BUỘC sau (KHÔNG chèn text rác/lặp lại):
- **Bảng so sánh đa chiều:** Ép buộc kẻ bảng từ suy luận của AI (VD: So sánh C++ vs uPython vs C ở góc độ Quản lý RAM, Tốc độ thực thi, Thời gian lập trình). Sau bảng phải có 3 đoạn văn nhận xét.
- **Phân tích Trade-off Engineering:** Đánh giá Chi phí – Hiệu năng – Năng lượng tiêu thụ - Tính khả thi bảo trì. Bất kể đó là module phần mềm hay linh kiện.
- **Ví dụ minh hoạ thực tế:** Tự "bịa" (mô phỏng logic) ra một ví dụ có con số thực. VD: "Giả sử băng thông mạng bị nghẽn 50%, hệ thống xử lý ra sao...".
- **Lịch sử phát triển & Xu hướng tiến hoá:** Công nghệ này ngày xưa ra sao, bây giờ thế nào, tại sao lại đổi mới.
- **Phân tích Khắc phục sự cố (Troubleshooting):** Có những lỗi (bugs) kinh điển nào thường gặp khi lập trình/thiết kế chức năng này và cách bạn vượt qua chúng.
- **Bàn luận Chuyên môn sâu (Discussion):** Góc nhìn đánh giá kỹ sư sau khi làm xong module đó. Mọi tiểu mục nhỏ đều có quyền tự chêm thêm "Bàn luận của tác giả".

### 9. Quy tắc Tiểu mục (Subsection)
- Tối đa **3 cấp**: 4.1 → 4.1.1 → 4.1.1.1. Không sâu hơn.
- Mỗi cấp phải có **ít nhất 2** tiểu mục. Không có 2.1.1 mà thiếu 2.1.2.

### 10. QUY TẮC BẮT BUỘC v5.0 (CRITICAL - KHÔNG ĐƯỢC VI PHẠM)

**A. Sơ đồ Mermaid - BỐ CỤC DÀN ĐỀU (CRITICAL) & CHỈ DÙNG TIẾNG ANH:**
- TẤT CẢ label, text, node name trong Mermaid code PHẢI viết bằng tiếng Anh ASCII (Mermaid.ink không hỗ trợ Tiếng Việt Unicode).
- **KHÔNG VẼ ĐƯỜNG THẲNG:** TUYỆT ĐỐI KHÔNG thiết kế chuỗi tuần tự quá dông dài (VD: `1->2->3->4->5...`) vì sơ đồ sẽ bị nén rất nhỏ trong mặt giấy.
- **KỸ THUẬT DÀN MẢNG 2D:** 
  1. Phải dàn đều các khối ra bề ngang/bề dọc bằng cách sử dụng các nhánh song song (VD: 1 process bắn ra 3 tiến trình con chạy song song).
  2. Sử dụng tích cực `subgraph` để gom nhóm logic (vd: `subgraph MCU`, `subgraph Sensors`), tạo hình khối vuông vức, giúp biểu đồ có tỷ lệ kích thước tối ưu mà **vẫn giữ nguyên được text chi tiết, đầy đủ không bị mất mát thông tin**.
  3. Linh hoạt kết hợp `direction LR` bên trong các module và `TD` cho luồng chính để sơ đồ đẹp như vẽ Visio.

**B. Chuỗi Ký tự Python (Raw String):**
- Khi sinh code Python `gen_report.py`, TOÀN BỘ các chuỗi có chứa backslash toán học LaTeX (`\theta`, `\Delta`, `\frac`, `\int`) hoặc newline (`\n`, `\t` của Mermaid) ĐỀU PHẢI dùng định dạng chuỗi thô của Python (vd: `r"W = \frac..."`). Nếu không sẽ gây ra SyntaxError hoặc UnicodeError khiến việc sinh báo cáo thất bại.

**B. Dau dau dong - DUNG DAU GACH NGANG:**
- Su dung ky tu `-` cho bullet list. TUYET DOI KHONG dung `●` hay dau cham.
- Ly do: ky tu Unicode `●` hien thi sai font thanh "dau cham to" trong Word.

**C. Moi linh kien/giao thuc/phuong phap PHAI co BANG SO SANH:**
- Khi gioi thieu bat ky linh kien (ESP32, MPU6050...), giao thuc (WebSocket, MQTT...) hay phuong phap (PID, Kalman...) nao:
  1. PHAI co BANG SO SANH voi it nhat 2 phuong an thay the.
  2. PHAI ghi RO ly do tai sao chon no thay vi cac cai khac.
  3. Tieu chi so sanh: gia thanh, hieu nang, do phuc tap, tinh tuong thich.

**D. Hinh anh dat TRONG chuong, KHONG chi o Phu luc (BẮT BUỘC):**
- Ảnh linh kiện, kết quả thi công, biểu đồ đo đạc BẮT BUỘC phải được rải đều xuyên suốt Chương 3, 4, 5.
- Khi tạo code Python (`gen_report.py`), đối với Chương 4, 5, 6, BẠN PHẢI gọi hàm `gen.add_placeholder_image(caption="Hinh...", width_inches=5.0)` ở TẤT CẢ các tiểu mục để chừa sẵn khung ảnh trống cho tác giả tự chèn ảnh thực tế vào sau. Kể cả sơ đồ khối, lưu đồ, ảnh mạch điện, ảnh sản phẩm thực tế đều phải gọi khung ảnh này. 

**E. Tai lieu tham khao (TLTK) PHAI CO THAT va KHOP SO THU TU:**
- Moi TLTK phai la nguon that, co the verify duoc (URL, ISBN, DOI).
- So thu tu [1], [2]... trong bai PHAI trung khop duoi danh muc.

**F. Độ dài và Chiều sâu luận điểm (TỐI QUAN TRỌNG - CHUNKING BẮT BUỘC):**
- BẠN KHÔNG THỂ viết nội dung Chương 4, 5, 6 dài 10 trang trong 1 lần prompt! Để làm được, khi được giao viết Chương 4, BẠN PHẢI nói: "Hãy cho phép tôi sinh nội dung làm 3 phần: thesis_ch4_4.1.py, thesis_ch4_4.2.py và thesis_ch4_4.3.py."
- Trong mỗi tiểu mục 4.x, BẠN PHẢI SINH RA HÀNG CH chục đoạn văn dài (sử dụng liên tục `gen.add_paragraph()`). Mỗi đoạn văn phải dài miên man.
- TUYỆT ĐỐI CẤM viêt lướt qua. Việc chèn Placeholder Image (Hình ảnh) không được dùng để thay thế lượng chữ. Bạn phải nhồi chữ bằng cách thực hiện triệt để Quy tắc số 8 (Content Expansion DOCTORATE LEVEL) trước khi chuyển sang luận điểm mới.

**G. Cấu hình Header và Footer TDTU (QUAN TRỌNG):**
- **Front Matter (trước Chương 1):** Header hoàn toàn bỏ trống. Footer chỉ đánh số trang bằng CHỮ SỐ LA MÃ (i, ii, iii) canh giữa. (Report Engine đã cấu hình sẵn trong `gen.start_front_matter()`).
- **Main Content (từ Chương 1):** Header chứa dòng "ĐỒ ÁN TỐT NGHIỆP/ TỔNG HỢP            Trang X" (Canh lề phải/giữa). Footer dưới cùng chứa "TÊN ĐỀ TÀI ĐỒ ÁN" (Canh lề trái/giữa tùy mẫu). (Report Engine đã làm tự động trong `gen.start_main_content()`). BẠN KHÔNG CẦN TỰ MÓC CODE CANH LỀ LẠI NHƯNG PHẢI NHỚ GỌI ĐÚNG 2 HÀM NÀY.

### 11. EXTREME ACADEMIC RIGOR (ĐỘC QUYỀN V8.0 - BẮT BUỘC MỖI DỰ ÁN BẤT KỲ)
Để biến một báo cáo tầm thường thành "Masterpiece", bạn PHẢI TỰ ĐỘNG áp dụng 3 kỹ thuật sau cho MỌI DỰ ÁN:
- **11.1 Phân tích Byte-Level & Protocol (Tầng Vật Lý):** Chống chỉ định miêu tả sơ sài. Ví dụ nếu dùng MPU6050 qua I2C, sinh viên thường chỉ ghi "đọc I2C", BẠN PHẢI vẽ cấu trúc Gói tin Payload: `[START][DevAddr 0x68][W][ACK][RegAddr 0x3B][ACK][R][Data 8-bit][NACK][STOP]`, rồi phân tích băng thông. Nếu làm Web, phân tích Header của HTTP Request/Response (Token JWT cấu tạo 3 phần Header.Payload.Signature ra sao).
- **11.2 Bảng Dữ liệu Đo kiểm Benchmarking (Mock Tinh Tế):** Nếu không được cấp số liệu, bạn hãy GIẢ ĐỊNH 1 kịch bản test mang tính kỹ thuật cao và MỞ BẢNG (add_table). Ví dụ: Đo độ nhiễu cảm biến Kalman qua 5 lần chạy (sai số ±0.02), Đo độ trễ (Latency) tải trang web qua 100/1000/5000 users. Lập bảng rõ ràng, rồi sau bảng là 1 trang A4 phân tích TẠI SAO có sai số.
- **11.3 Tình huống Ngoại lệ (Case Studies / Edge Cases):** Luôn dành khoảng 2 trang cuối mỗi chương cốt lõi để phân tích "Trường hợp sự cố". VD: "Xử lý kẹt động cơ phần cứng", "Mất kết nối Database", "Nhiễu điện từ EMI". Phân tích nguyên nhân và trình bày giải pháp phần mềm/phần cứng chống chịu (Fault Tolerance).

**H. KHÔNG Đánh số công thức (UPDATE):**
- Khi gọi hàm `gen.add_formula()`, TUYỆT ĐỐI KHÔNG truyền tham số đánh số (không dùng `formula_number=...`). Chỉ truyền nội dung toán học và text giải thích biến vì sinh viên không yêu cầu đánh số sau công thức nữa.

**I. NGHIÊM CẤM TẠO TRANG TRẮNG:**
- Hạn chế tối đa việc gọi thừa hàm `gen.add_page_break()`. Tuyệt đối không để rớt một trang hoàn toàn trống. Xin lưu ý: Hàm `gen.add_heading(title, level=1)` (Tạo Chương mới) ĐÃ TỰ ĐỘNG ngắt sang trang mới trong Engine, nên BẠN KHÔNG ĐƯỢC gọi `gen.add_page_break()` sát trước khi gọi tạo Chương 1, 2, 3... để tránh lỗi đẻ ra trang trắng liên tục.

---

## II. ĐỊNH DẠNG CHUẨN TDTU (Formatting Spec)

| Thông số | Giá trị | Ghi chú |
|---|---|---|
| Font chữ | Times New Roman (Unicode) | Cho cả nội dung lẫn heading |
| Cỡ chữ nội dung | **13pt** | Mật độ chữ bình thường, không nén/dãn |
| Heading 1 (Tên chương) | **16pt, Bold, IN HOA** | CHƯƠNG 1. GIỚI THIỆU ĐỀ TÀI |
| Heading 2 (Mục cấp 1) | **14pt, Bold** | 1.1 Đặt vấn đề |
| Heading 3 (Mục cấp 2) | **13pt, Bold** | 1.1.1 Mô tả chi tiết |
| Dãn dòng | **1.5 lines** | |
| Căn lề | **Justify** (Đều 2 bên) | |
| Lề trên | **3.5 cm** | |
| Lề dưới | **3.0 cm** | |
| Lề trái | **3.5 cm** | Để đóng gáy |
| Lề phải | **2.0 cm** | |
| Thụt đầu đoạn | **1 tab** (1.27 cm) | Dòng đầu mỗi đoạn văn |
| Đầu chương | **Ngắt trang mới** | Mỗi CHƯƠNG 1, 2... BẮT BUỘC phải nằm tách biệt ở một vùng trang MỚI (Page Break). |
| Danh sách (Bullet) | **Khoảng trắng** | TUYỆT ĐỐI không sử dụng ký tự gạch ngang '-' đầu câu như các văn bản thông thường. |
| Đánh số trang (Front Matter)| **Ở dưới, Canh giữa** | Từ Lời Cảm Ơn đến hết Danh mục: dùng **số La Mã** (i, ii...). **Header bỏ trống**. |
| Đánh số trang (Main) | **Ở trên, Header, Canh lề phải** | Từ Chương 1 trở đi: dùng **số Arabic** (1, 2...). |
| Header (Main) | Text xám in đậm | Ví dụ: `ĐỒ ÁN TỐT NGHIỆP/ TỔNG HỢP          Trang 1` |
| Footer (Main) | Tên đề tài in đậm | Canh giữa. Trang bìa KHÔNG hiện Footer. |
| Khổ giấy | **A4** (210 × 297mm), portrait, 1 mặt | |
| Chú thích hình | Phía **dưới** hình, căn giữa, italic | Hình 3.1. Tên hình |
| Tiêu đề bảng | Phía **trên** bảng, căn giữa, bold | Bảng 3.1: Tên bảng |
| Font code | Consolas hoặc Courier New, **10pt** | |
| Nội dung chính | **Tối thiểu 30 trang, tối đa 80 trang** | Trải dài đặc biệt nhiều ở Chương 4 và 5. Code sinh mỗi chương Python phải từ 100 tới 400 line code để bảm bảo tính Hàn Lâm siêu việt. |

> **Lưu ý Engine v5.0 (Cấu hình Header/Footer):** Bắt buộc phải chia 2 phase khi sinh report rạch ròi bằng `gen.start_front_matter()` (cho Lời cảm ơn, Mục lục...) và `gen.start_main_content()` (từ Chương 1 trở đi) để report engine kích hoạt tính năng Format Section. 
> -> `gen.start_front_matter()`: Canh số La Mã ở Footer giữa, Header trống rỗng.
> -> `gen.start_main_content()`: Canh chữ "ĐỒ ÁN" + Số trang ở Header, Footer là Tên đề tài môn học. Trang bìa tự động được khóa nhờ `different_first_page_header_footer`.
> **Quy định Khối Lượng Nội Dung (Content Depth Limits)**: Việc chia file Python (ví dụ: `thesis_ch1.py` đến `thesis_ch6.py`) không phải chỉ để chia nhỏ mà mục đích là để vi phân tích ĐỘ SÂU TOÁN HỌC / KIÊU KIẾN TRÚC. Từng file này ít nhất phải code 100 đến 400 dòng python gọi `add_paragraph` để viết lại phân tích chức năng thiết bị, quy trình thiết kế, cách sử dụng chi tiết cực độ.

---

## III. CẤU TRÚC BÁO CÁO BẮT BUỘC (Thesis Structure)

### A. Phần thủ tục (Front Matter)

```
 1. Trang bìa chính (Mẫu 1)
    ┌──────────────────────────────────────────────┐
    │  TỔNG LIÊN ĐOÀN LAO ĐỘNG VIỆT NAM (size 14) │
    │  TRƯỜNG ĐẠI HỌC TÔN ĐỨC THẮNG (Bold, 14)  │
    │  [TÊN KHOA] (Bold, 14)                      │
    │  [Logo TDTU]                                 │
    │  HỌ VÀ TÊN SINH VIÊN (Bold, 14)             │
    │  TÊN ĐỀ TÀI (Bold, 24)                      │
    │  ĐỒ ÁN TỐT NGHIỆP/TỔNG HỢP (Bold, 22)     │
    │  [TÊN NGÀNH] (Bold, 22)                      │
    │  TP.HCM, NĂM... (Bold, 14)                  │
    └──────────────────────────────────────────────┘
 2. Trang bìa phụ (Mẫu 2) — như bìa chính + thêm tên GVHD
 3. LỜI CẢM ƠN (Bold, 16) — ký tên tác giả ở cuối
 4. CÔNG TRÌNH ĐƯỢC HOÀN THÀNH TẠI TDTU — Lời cam đoan + ký tên
 5. Phiếu giao nhiệm vụ (do Khoa cấp)
 6. Lịch trình làm đồ án (bảng tuần/khối lượng/GVHD ký)
 7. TÓM TẮT (Bold, 16) — Tiếng Việt, 1-2 trang
 8. ABSTRACT (Bold, 16) — Tiếng Anh, 1-2 trang
 9. MỤC LỤC — Tự sinh từ Heading
10. DANH MỤC HÌNH VẼ
11. DANH MỤC BẢNG BIỂU
12. DANH MỤC CÁC CHỮ VIẾT TẮT (xếp A-Z)
```

### B. Phần nội dung chính (6 Chương)

```
CHƯƠNG 1. GIỚI THIỆU ĐỀ TÀI                          ≥ 3 trang
├── 1.1 Đặt vấn đề / Lý do chọn đề tài
│   └── Bối cảnh thực tế → Vấn đề tồn tại → Nhu cầu giải quyết
├── 1.2 Mục tiêu nghiên cứu
│   └── Mục tiêu tổng quát + Các mục tiêu cụ thể (liệt kê rõ ràng)
├── 1.3 Đối tượng và phạm vi nghiên cứu
│   └── Đối tượng: cái gì? | Phạm vi: giới hạn gì?
├── 1.4 Phương pháp nghiên cứu
│   └── PP lý thuyết + PP thực nghiệm + PP kiểm chứng
└── 1.5 Ý nghĩa khoa học và thực tiễn
    └── Đóng góp gì cho ngành? Ứng dụng thực tế?

CHƯƠNG 2. TỔNG QUAN TÀI LIỆU                         ≥ 5 trang
├── 2.1 Tình hình nghiên cứu trong nước
│   └── Phân tích ≥ 2 công trình/sản phẩm nội địa liên quan
├── 2.2 Tình hình nghiên cứu ngoài nước
│   └── Phân tích ≥ 3 công trình/paper quốc tế liên quan
├── 2.3 Các sản phẩm/giải pháp thương mại hiện có
│   └── Bảng so sánh tính năng + giá thành + hạn chế
└── 2.4 Đánh giá tổng hợp và hướng tiếp cận của đề tài
    └── Khoảng trống (gap) mà đề tài này sẽ lấp đầy

CHƯƠNG 3. CƠ SỞ LÝ THUYẾT                            ≥ 7 trang
├── 3.1 Lý thuyết chuyên ngành liên quan
│   └── Nền tảng học thuật cốt lõi của đề tài
├── 3.2 Nguyên lý hoạt động các linh kiện/cảm biến chính
│   └── Mỗi linh kiện: Hình ảnh + Datasheet + Bảng thông số
├── 3.3 Mô hình toán học và thuật toán
│   └── Phương trình, chứng minh, suy luận chi tiết
└── 3.4 Các công nghệ phần mềm/framework sử dụng
    └── Kiến trúc, luồng dữ liệu, ưu nhược điểm

CHƯƠNG 4. TÍNH TOÁN VÀ THIẾT KẾ HỆ THỐNG              ≥ 10 trang
├── 4.1 Yêu cầu thiết kế tổng quan
│   └── Bảng yêu cầu kỹ thuật (Specification)
├── 4.2 Thiết kế phần cứng
│   ├── 4.2.1 Sơ đồ khối + giải thích từng khối
│   ├── 4.2.2 Tính toán lựa chọn linh kiện (dòng, áp, công suất)
│   ├── 4.2.3 Sơ đồ nguyên lý mạch điện (Schematic)
│   └── 4.2.4 Bảng Pinout / Bảng kết nối
├── 4.3 Thiết kế phần mềm
│   ├── 4.3.1 Kiến trúc phần mềm tổng quan
│   ├── 4.3.2 Lưu đồ thuật toán (Flowchart)
│   ├── 4.3.3 Mảnh ghép học thuật: Tác giả đã học được cách triển khai thuật toán gì vào thực tế? (Đưa vào đây những đúc kết cá nhân)
│   └── 4.3.4 Giải thích code cốt lõi (trích đoạn + diễn giải chức năng từng hàm)
├── 4.4 Thiết kế hệ thống điều khiển / phần mềm ứng dụng
│   └── Phân tích việc ứng dụng các framework hoặc kỹ thuật điều khiển. Những kỹ năng lập trình / config nào tác giả đã thu hoạch được qua thao tác này?
└── 4.5 Tổng kết chương 4
    └── Những yếu tố cốt lõi nào đã làm nên một thiết kế thành công? Sự phù hợp giữa lý thuyết và thiết kế thực tế như thế nào?

CHƯƠNG 5. KẾT QUẢ VÀ BÀN LUẬN                        ≥ 6 trang
├── 5.1 Sản phẩm thực tế đã đạt được
│   └── Mô tả chức năng sản phẩm hoàn thiện. Sản phẩm này có thể áp dụng vào đời sống / công nghiệp cụ thể như thế nào?
├── 5.2 Kịch bản thực nghiệm 1: [Tên kịch bản]
│   └── Mục đích → Thiết lập → Bảng số liệu → Biểu đồ LaTeX → Đánh giá khách quan (Sai số là bao nhiêu? Tại sao lại có dải sai số này?)
├── 5.3 Kịch bản thực nghiệm 2: [Tên kịch bản]
│   └── Phân tích sâu: Qua thí nghiệm này, bản thân sinh viên đã điều chỉnh / can thiệp (tune) thông số như thế nào để hệ thống chạy ổn định?
├── 5.4 Kịch bản thực nghiệm 3: [Tên kịch bản]
│   └── (tương tự)
└── 5.5 Đánh giá tổng quát
    └── Đạt / Chưa đạt mục tiêu ban đầu? So sánh trực tiếp những gì làm được với mục tiêu đề ra ở Chương 1. Tác giả đã gặt hái được những kinh nghiệm xương máu nào trong khâu lắp ráp/vận hành?

CHƯƠNG 6. KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN                ≥ 2 trang
├── 6.1 Kết luận chung
│   ├── Tóm tắt 3-5 kết quả kỹ thuật ấn tượng nhất đã làm được.
│   └── Những kỹ năng cứng (phần cứng, phần mềm, hàn xì) và kỹ năng mềm mà nhóm/sinh viên đã đúc kết được qua toàn bộ quá trình chạy dự án.
├── 6.2 Hướng phát triển và khả năng thương mại hoá
│   └── Đề xuất cụ thể các update tương lai. Liệu sản phẩm có thể startup / thương mại hoá không? Cần thêm công nghệ gì để đạt mức sản xuất hàng loạt?
└── 6.3 Lời kết
    └── Cảm nhận thực tế của người thực hiện về trải nghiệm làm đồ án.
```

### C. Phần cuối (Back Matter)

```
TÀI LIỆU THAM KHẢO — Chuẩn APA6, chia Tiếng Việt / Tiếng Anh
PHỤ LỤC A — Mã nguồn (nếu có)
PHỤ LỤC B — Bản vẽ / PCB (nếu có)
PHỤ LỤC C — Datasheet (nếu có)
Lưu ý: Phụ lục KHÔNG ĐƯỢC dày hơn phần nội dung chính
```

---

## IV. QUY TRÌNH THỰC HIỆN (Workflow)

### Bước 1: Khảo sát Dự án (Project Deep Scan)

Trước khi viết BẤT KỲ nội dung nào, BẮT BUỘC quét toàn bộ folder mã nguồn:

| Mục tiêu | Cách tìm | Ví dụ |
|---|---|---|
| Xác định đề tài/lĩnh vực | Đọc README, tên project | Robot, IoT, AI, FPGA... |
| Xác định vi điều khiển | `platformio.ini`, `*.ino` | ESP32, STM32, Arduino... |
| Xác định cảm biến/actuator | Đọc file `.h`, `#include` | MPU6050, DHT22, Servo... |
| Xác định framework SW | `package.json`, `requirements.txt`, `Cargo.toml`, `CMakeLists.txt` | ROS2, React, Flask, TensorFlow... |
| Xác định thuật toán cốt lõi | Đọc logic code chính | PID, FFT, SLAM, CNN, Fuzzy... |
| Xác định giao thức truyền thông | Đọc hàm init/send/receive | UART, SPI, MQTT, WebSocket, BLE... |
| Xác định cấu trúc dự án | Quét cây thư mục | firmware/, app/, server/... |

**Output của bước này:** Bảng tóm tắt dự án (Project Fact Sheet) — phải trình cho user xác nhận trước khi viết.

### Bước 2: Xác nhận thông tin với User

Trước khi bắt đầu sinh nội dung, HỎI user các thông tin thiếu:
- Họ tên sinh viên, MSSV
- Tên GVHD (học hàm, học vị)
- Tên khoa, tên ngành
- Tên đề tài chính xác
- Đề tài bằng Tiếng Việt hay Tiếng Anh?
- Có số liệu thực nghiệm thật hay cần tạo giả lập?
- Có hình ảnh mô hình thực tế không?

### Bước 3: Sinh Dàn ý Chi tiết (Detailed Outline)

Dựa vào Bước 1+2, tạo dàn ý đến H3 theo cấu trúc ở Mục III. Các tiểu mục Chương 3, 4 phải được **ĐIỀU CHỈNH LINH HOẠT** theo từng đề tài — dàn ý ở Mục III chỉ là khung xương.

**Mẫu Prompt để xác nhận dàn ý:**
> "Dựa trên phân tích mã nguồn, tôi đề xuất dàn ý sau cho báo cáo. Anh/chị xem và điều chỉnh nếu cần trước khi tôi bắt đầu viết."

### Bước 4: Viết Nội dung Từng Chương (Chapter-by-Chapter Generation)

**QUAN TRỌNG:** Do giới hạn token, phải viết **TỪNG CHƯƠNG MỘT** theo thứ tự. Mỗi lần gọi AI, kèm prompt:

> "Bạn là một kỹ sư tốt nghiệp xuất sắc chuyên ngành [tên ngành] tại Trường Đại học Tôn Đức Thắng. Hãy viết mục [X.Y] thuộc Chương [X] của báo cáo đồ án tốt nghiệp. Yêu cầu: nội dung chuyên sâu cấp đại học, văn phong hàn lâm, tối thiểu [N] từ. Đề tài: [tên đề tài]. Context từ mã nguồn: [trích đoạn code liên quan]."

**Thứ tự viết:**

| Lần gọi | Nội dung | Ước lượng |
|---|---|---|
| 1 | Front Matter: Lời cảm ơn, Lời cam đoan, Tóm tắt, Abstract | 3-4 trang |
| 2 | Chương 1: Giới thiệu đề tài | 3-4 trang |
| 3 | Chương 2: Tổng quan tài liệu | 5-6 trang |
| 4 | Chương 3: Cơ sở lý thuyết (phần 3.1–3.2) | 4-5 trang |
| 5 | Chương 3 (tiếp): phần 3.3–3.4 | 3-4 trang |
| 6 | Chương 4: Thiết kế phần cứng (4.1–4.2) | 5-6 trang |
| 7 | Chương 4 (tiếp): Thiết kế phần mềm (4.3–4.5) | 5-6 trang |
| 8 | Chương 5: Kết quả thực nghiệm | 6-8 trang |
| 9 | Chương 6 + TÀI LIỆU THAM KHẢO | 3-4 trang |
| 10 | Sinh Mục lục, Danh mục hình/bảng/viết tắt, Phụ lục | 3-5 trang |

### Bước 5: Tạo Sơ đồ & Bảng biểu

Xen kẽ trong nội dung, dùng **Mermaid** + API `mermaid.ink` sinh ít nhất **8 sơ đồ**:

| Loại sơ đồ | Mermaid syntax | Áp dụng đề tài | Chương |
|---|---|---|---|
| Sơ đồ khối hệ thống | `graph TD` | Mọi đề tài | 4 |
| Lưu đồ thuật toán | `flowchart TD` | Mọi đề tài | 4 |
| State Machine | `stateDiagram-v2` | Điều khiển, Robot | 4 |
| Sequence Diagram | `sequenceDiagram` | Client-Server, IoT | 4 |
| Class Diagram | `classDiagram` | Phần mềm OOP | 4 |
| ER Diagram | `erDiagram` | Database, Web app | 4 |
| Kiến trúc phần mềm | `graph LR` | App, Hệ nhúng | 4 |
| Gantt Chart | `gantt` | Timeline dự án | 1 |
| Pie Chart | `pie` | Phân bổ tài nguyên | 5 |
| Bảng so sánh | Markdown table | Mọi đề tài | 2, 3 |

### Bước 6: Kiểm tra Chất lượng (Quality Gate)

Sau khi viết xong, chạy checklist sau đây. **Mọi mục đều phải PASS:**

```
□ Front Matter đầy đủ (bìa, lời cảm ơn, cam đoan, tóm tắt, abstract, mục lục, danh mục)?
□ Ít nhất 6 chương nội dung?
□ Mỗi chương đạt quota số trang tối thiểu?
□ Tổng nội dung ≥ 30 trang (không tính bìa, mục lục, TLTK, phụ lục)?
□ Mọi hình vẽ có đánh số theo chương (Hình X.Y) + chú thích phía dưới?
□ Mọi bảng biểu có đánh số theo chương (Bảng X.Y) + tiêu đề phía trên?
□ Mọi công thức có đánh số (X.Y) + đoạn "Trong đó:" giải thích biến?
□ Mọi hình/bảng/công thức đều được tham chiếu trong nội dung ("xem Hình 3.4")?
□ Có ≥ 3 kịch bản thực nghiệm với bảng số liệu + biểu đồ + phân tích?
□ Tài liệu tham khảo ≥ 10 nguồn, đúng APA6, chia TV/TA?
□ Không có tiểu mục lẻ (có X.1.1 thì phải có X.1.2)?
□ Tiểu mục không quá 3 cấp?
□ Không dùng ngôi thứ nhất (tôi/em)?
□ Font Times New Roman 13pt, dãn dòng 1.5, căn Justify?
□ Lề: Trên 3.5 / Dưới 3 / Trái 3.5 / Phải 2 cm?
□ Viết tắt: lần đầu viết đầy đủ + (viết tắt)?
□ Code: dùng font Consolas 10pt, có giải thích?
□ Đầu mỗi đoạn thụt 1 tab?
□ Phụ lục không dày hơn phần chính?
□ Mỗi tiểu mục ≥ 3 đoạn văn?
```

### Bước 7: Xuất File Word (.docx)

Sử dụng script `scripts/smart_report_engine.py` với class `ReportGenerator`:

```python
from smart_report_engine import ReportGenerator

gen = ReportGenerator(
    title="TÊN ĐỀ TÀI",
    author="HỌ TÊN SV",
    advisor="TS. NGUYỄN VĂN A",
    department="KHOA ĐIỆN – ĐIỆN TỬ",
    major="KỸ THUẬT ĐIỆN TỬ - VIỄN THÔNG",
    year="2026"
)

# Front matter
gen.add_cover_page()

# Kích hoạt chế độ Front Matter (Đánh số La Mã ở Footer)
gen.start_front_matter()
gen.add_acknowledgment()
gen.add_declaration()

# Kích hoạt chế độ Main Content (Đánh số Arabic ở Header + Tiêu đề Footer)
gen.start_main_content()
gen.add_heading("CHƯƠNG 1. GIỚI THIỆU ĐỀ TÀI")
gen.add_heading("1.1 Đặt vấn đề", level=2)
gen.add_paragraph("Nội dung...")

# Công thức - CHÚ Ý: Bắt buộc dùng raw string r"..."
gen.add_formula(r"v = \frac{R}{2} \cdot (v_R + v_L)", formula_number="3.1",
    explanation="Trong đó: v là vận tốc tịnh tiến (m/s), R là bán kính (m)...")

# Bảng
gen.add_table([["Pin", "Chức năng"], ["GPIO16", "PWM Motor"]], caption="Bảng 4.1: Pinout")

# Sơ đồ tự động (Sinh qua Mermaid)
gen.add_mermaid_diagram("graph TD; A-->B", caption="Hình 4.1. Sơ đồ khối")

# KHUNG CHỜ ẢNH THỰC TẾ (Dành cho các mục chèn ảnh Linh kiện / Sản phẩm mẫu / Giao diện người dùng)
gen.add_placeholder_image(caption="Hình 5.1: Ảnh thực tế khung gầm Robot", width_inches=4.5)

# Tự động sinh Mục lục (Cần Ctrl+A và F9 trên Word)
gen.generate_table_of_contents()

gen.save("DoAn_TotNghiep_TDTU.docx")
```

---

## V. MẪU CÂU HÀN LÂM THƯỜNG DÙNG (Academic Phrase Bank)

Để AI viết đúng văn phong, sử dụng các mẫu câu sau:

**Mở đầu chương/mục:**
- "Chương này trình bày [nội dung chính] nhằm [mục đích]."
- "Trên cơ sở phân tích ở Chương [X], phần tiếp theo sẽ đi sâu vào [nội dung]."
- "Mục này tập trung khảo sát [đối tượng] dưới góc độ [khía cạnh]."

**Giới thiệu công nghệ/linh kiện:**
- "[Tên] là một [loại] được phát triển bởi [hãng], cho phép [chức năng chính]."
- "So với [giải pháp cũ], [tên] mang lại ưu điểm vượt trội về [khía cạnh]."
- "Trong phạm vi đề tài này, [tên] được lựa chọn nhờ [lý do 1], [lý do 2] và [lý do 3]."

**Trình bày kết quả:**
- "Bảng [X.Y] tổng hợp kết quả đo lường trong [điều kiện thí nghiệm]."
- "Từ Hình [X.Y] có thể nhận thấy rằng [nhận xét chính]."
- "Kết quả thực nghiệm cho thấy hệ thống đạt [chỉ tiêu] với sai số [giá trị]."
- "So sánh với nghiên cứu của [Tác giả] (Năm), kết quả của đề tài cho thấy [điểm tương đồng/khác biệt]."

**Kết luận:**
- "Đề tài đã hoàn thành [mục tiêu đặt ra] thông qua việc [phương pháp]."
- "Tuy nhiên, hệ thống vẫn còn một số hạn chế bao gồm [liệt kê]."
- "Để khắc phục những hạn chế trên, tác giả đề xuất [giải pháp] cho các nghiên cứu tiếp theo."

---

## VI. MẪU THÍCH ỨNG THEO LOẠI ĐỀ TÀI (Adaptive Templates)

Tuỳ thuộc loại đề tài, Chương 3 và 4 phải được điều chỉnh:

### Đề tài Hệ nhúng / Vi điều khiển (ESP32, STM32, Arduino...)
- Ch3: Động học/Điều khiển + Nguyên lý cảm biến + Giao thức (UART/SPI/I2C)
- Ch4: Sơ đồ mạch + Tính dòng/áp + Pinout + Firmware flowchart

### Đề tài IoT / Smart Home
- Ch3: Kiến trúc IoT (Edge-Fog-Cloud) + Giao thức MQTT/HTTP + Bảo mật
- Ch4: Thiết kế node cảm biến + Server/Broker + Dashboard + Mobile App

### Đề tài AI / Machine Learning
- Ch3: Mô hình toán học (CNN/RNN/Transformer) + Hàm loss + Tối ưu hoá
- Ch4: Pipeline dữ liệu + Kiến trúc mạng + Hyperparameter + Training strategy

### Đề tài Robot / Cơ điện tử
- Ch3: Động học (Kinematics) + Điều khiển PID + SLAM/Navigation
- Ch4: Thiết kế cơ khí 3D + Mạch công suất + Firmware + App giám sát

### Đề tài PLC / Tự động hoá
- Ch3: Lý thuyết điều khiển tự động + PLC architecture + HMI/SCADA
- Ch4: Sơ đồ P&ID + Ladder diagram + Wiring + Cấu hình HMI

### Đề tài Viễn thông / FPGA
- Ch3: Lý thuyết tín hiệu + Điều chế + Mã hoá kênh truyền
- Ch4: Thiết kế HDL + Simulation + Testbench + Resource utilization

### Đề tài Web/Mobile App
- Ch3: Software architecture + Design patterns + Database design
- Ch4: ERD + API design + UI/UX mockup + Deployment architecture

---

## VII. CÔNG CỤ HỖ TRỢ

| Công cụ | Chức năng |
|---|---|
| `scripts/smart_report_engine.py` | Class `ReportGenerator` — xuất .docx chuẩn TDTU |
| `codebase_investigator` | Đọc hiểu mã nguồn trước khi viết nội dung |
| API `mermaid.ink` | Render Mermaid → PNG để chèn vào Word |
| `pandoc` | Chuyển đổi Markdown ↔ DOCX (backup option) |

---

## VIII. NHỮNG LỖI KINH ĐIỂN CẦN TRÁNH (LESSONS LEARNED)

Đây là danh sách các lỗi nghiêm trọng mà các version AI trước thường xuyên vi phạm. AI phiên bản hiện tại **BẮT BUỘC ĐỌC THẬT KỸ VÀ KHÔNG ĐƯỢC LẶP LẠI**:

1. **Lỗi Tự biên tự diễn:** Không chịu hỏi user đề tài là gì mà tự động lấy form AMR cũ ráp vào. (Đã sửa bằng Bước 1 ở trên).
2. **Lỗi Số trang / Header:** Tự móc code chỉnh alignment hoặc không dùng `gen.start_front_matter()` và `gen.start_main_content()`, khiến số trang đầu tiên của một section bị tàng hình. (Đã sửa triệt để trong lõi của Engine). BẠN CHỈ VIỆC GỌI ĐÚNG HÀM.
3. **Lỗi Lười biếng (Short Content):** Viết báo cáo nhưng lười tư duy, chỉ viết Chương 4, 5, 6 vài gạch đầu dòng hời hợt. Đây là **NHIỆM VỤ QUAN TRỌNG NHẤT**: Bạn được phân công tạo ra MỘT FILE PYTHON (`gen_report.py`) HOÀN CHỈNH, CHUYÊN SÂU với độ dài **PHẢI VƯỢT QUÁ 1000 DÒNG CODE** (có thể phân tách thành `gen_ch1_ch3.py` và `gen_ch4_ch6.py` nếu cần). Cần sinh ra hàng trăm lời gọi `add_paragraph()` chứa **hàng ngàn chữ phân tích kỹ thuật sâu về nguyên lý hoạt động, sơ đồ chân linh kiện, lý do chọn phương án, lỗi debug mắc phải và cách fix, cùng bảng dữ liệu thực nghiệm**. CẤM TẠO RA BẢN MVP, PHẢI TẠO RA BẢN BÁO CÁO FULL DETAIL 50-80 TRANG.
4. **Lỗi Trắng trang:** Lạm dụng hàm `gen.add_page_break()` sát trước khi gọi `gen.add_heading(level=1)`. Bản thân `add_heading` Chương Đầu đã có tự ngắt trang, gọi thêm `page_break` sẽ đẻ ra một trang giấy trắng trống trơn vô duyên.
5. **Lỗi Bảng Bị Chặt Đôi (Table Break):** Đã được fix cứng trong lõi bằng `cantSplit` và `keep_with_next`. AI chỉ cần gọi `add_table` bình thường. Không tự can thiệp.
6. **Lỗi Số Công Thức Nằm Ngoài Bảng:** Đã loại bỏ hoàn toàn tính năng đánh số tự động sau công thức. AI khi gọi `add_formula` TUYỆT ĐỐI BỎ THAM SỐ `formula_number`.
7. **Lỗi Quên chèn vị trí Ảnh (Placeholder):** Quên không chừa chỗ cho sinh viên đổ hình thật vào. Trong Chương 4,5,6 bạn BẮT BUỘC phải rải đều các lời gọi `gen.add_placeholder_image(caption="Hinh...", width_inches=5.0)`.
8. **Lỗi Canh Đều Giãn Chữ (Justify Space):** Đã được khắc phục trong lõi Word bằng custom XML fix. AI cứ truyền tiếng Việt bình thường không sợ lỗi dàn chữ.
