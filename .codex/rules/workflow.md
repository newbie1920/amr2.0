# Workflow Rules

## 1. Antigravity Global Rules â€” Smart Coding Agent

### 1.1 Láº­p Káº¿ Hoáº¡ch & TÆ° Váº¥n TrÆ°á»›c Khi LÃ m (Plan & Consult First)
- Vá»›i Má»ŒI yÃªu cáº§u phá»©c táº¡p (thÃªm tÃ­nh nÄƒng, thay Ä‘á»•i kiáº¿n trÃºc, chá»n cÃ´ng nghá»‡), PHáº¢I:
  a) PhÃ¢n tÃ­ch yÃªu cáº§u vÃ  liá»‡t kÃª cÃ¡c PHÆ¯Æ NG ÃN kháº£ thi (Ã­t nháº¥t 2-3 lá»±a chá»n).
  b) Vá»›i Má»–I phÆ°Æ¡ng Ã¡n, giáº£i thÃ­ch rÃµ rÃ ng Æ°u/nhÆ°á»£c Ä‘iá»ƒm khi Ã¡p dá»¥ng vÃ o dá»± Ã¡n cá»§a ngÆ°á»i dÃ¹ng.
  c) Äá» xuáº¥t phÆ°Æ¡ng Ã¡n tá»‘t nháº¥t kÃ¨m lÃ½ do, nhÆ°ng Ä‘á»ƒ NGÆ¯á»œI DÃ™NG QUYáº¾T Äá»ŠNH.
  d) Chá»‰ báº¯t tay vÃ o code SAU KHI ngÆ°á»i dÃ¹ng chá»n phÆ°Æ¡ng Ã¡n.
- Vá»›i yÃªu cáº§u Ä‘Æ¡n giáº£n (fix lá»—i nhá», chá»‰nh CSS, thÃªm comment): lÃ m luÃ´n, khÃ´ng cáº§n há»i.

### 1.2 LÆ°u Tiáº¿n Äá»™, Káº¿ Hoáº¡ch & Nháº­t KÃ½
- Khi báº¯t Ä‘áº§u task lá»›n, tá»± táº¡o file káº¿ hoáº¡ch (`implementation_plan.md`) chá»©a checklist.
- Cáº­p nháº­t `task.md` trong quÃ¡ trÃ¬nh lÃ m.
- **Ghi nháº­t kÃ½ hÃ ng ngÃ y (Journaling):** Sau má»—i khi nháº­n Ä‘Æ°á»£c bÃ¡o cÃ¡o lá»—i hoáº·c hoÃ n thÃ nh fix bug, tá»± Ä‘á»™ng ghi chÃº vÃ o `ai_journals/YYYY-MM-DD.md` theo Ä‘á»‹nh dáº¡ng: `[HH:MM] User: <lá»—i/yÃªu cáº§u> -> AI: <file sá»­a> -> Status: <káº¿t quáº£>.`

### 1.3 TÃ­ch Há»£p Obsidian (Second Brain)
- LuÃ´n coi há»‡ thá»‘ng Obsidian cá»§a ngÆ°á»i dÃ¹ng lÃ  "Bá»™ nÃ£o thá»© hai". Há»i ngÆ°á»i dÃ¹ng Ä‘Æ°á»ng dáº«n Obsidian Vault Ä‘á»ƒ Ä‘á»c tÃ i liá»‡u khi cáº§n bá»‘i cáº£nh.
- Code xong logic khÃ³ â†’ Äá» xuáº¥t viáº¿t tÃ i liá»‡u Markdown + sÆ¡ Ä‘á»“ lÆ°u tháº³ng vÃ o Obsidian.

### 1.4 Hiá»ƒu TrÆ°á»›c Khi LÃ m (Understand Before Act)
- Äá»c hiá»ƒu cáº¥u trÃºc (`README.md`, `package.json`, `platformio.ini`...) trÆ°á»›c khi code.
- Debug: TÃ¬m ROOT CAUSE (nguyÃªn nhÃ¢n gá»‘c), khÃ´ng chá»¯a triá»‡u chá»©ng.
- Báº¯t buá»™c search web hoáº·c GitHub Ä‘á»ƒ tham kháº£o khi gáº·p lá»—i láº¡/tÃ­nh nÄƒng má»›i.
- Thao tÃ¡c chÃ­nh xÃ¡c: DÃ¹ng `grep_search` tÃ¬m chÃ­nh xÃ¡c dÃ²ng code, khÃ´ng Ä‘á»c cáº£ file. Giá»¯ nguyÃªn comment cÅ©.

### 1.5 Test, XÃ¡c Minh & TrÃ¡nh Láº·p Lá»—i
- LuÃ´n cháº¡y build/test/lint sau khi sá»­a. Lá»—i tá»± sá»­a tá»‘i Ä‘a 2 láº§n.
- KhÃ´ng láº·p lá»—i: Sá»­a lá»—i xong â†’ QuÃ©t toÃ n codebase xem cÃ³ lá»—i tÆ°Æ¡ng tá»± khÃ´ng, sá»­a háº¿t.
- Giao tiáº¿p: Ngáº¯n gá»n, sÃºc tÃ­ch (Bullet point/Table). BÃ¡o cÃ¡o: ÄÃ£ lÃ m gÃ¬ â†’ Káº¿t quáº£ â†’ CÃ²n gÃ¬.
- CÃ´ng cá»¥ thÃ´ng minh: Táº­n dá»¥ng MCP (GitHub, Supabase, DevTools), cháº¡y song song lá»‡nh cho nhanh.

---

## 2. AMR2.0 Project Brain & Workflow

### 2.1 Before Editing
1. Check the worktree with `git status --short`.
2. Search narrowly with `rg` or `grep_search` before opening large files.
3. For symbols, run GitNexus impact analysis and note risk.
4. If risk is HIGH or CRITICAL, warn the user before editing.

### 2.2 During Editing
- Keep changes surgical and local to the task.
- Do not rewrite unrelated comments or formatting.
- Preserve baseline robot behavior unless the user explicitly asks to replace it.
- Update `task.md` or `walkthrough.md` only when the task scope calls for ongoing progress tracking.

### 2.3 After Editing & Verification
1. Run the narrowest meaningful verification:
   - Frontend/app: `npm run test`, `npm run build`, or `npm run check`.
   - Trajectory benchmark: `npm run benchmark:trajectory`.
   - Firmware: `python -m platformio run` from `esp32s3xe_v2`.
   - Docs/config-only: validate JSON/Markdown shape and run `git diff --check`.
2. If verification fails, attempt up to two focused fixes.
3. Scan for the same issue pattern elsewhere when a bug class is identified.
4. Report: changed files, result, verification, remaining risk.

---

## 3. GitNexus & Code Intelligence
- **MUST run impact analysis before editing any symbol.**
- **MUST run `gitnexus_detect_changes()` before committing**.
- Warn the user if impact analysis returns HIGH or CRITICAL risk.
- NEVER rename symbols with find-and-replace â€” use `gitnexus_rename`.
- When exploring unfamiliar code, use `gitnexus_query`. When needing full context, use `gitnexus_context`.
