// ============================================================
//   AMR 2.0 — WiFi Connection & WiFiManager (Captive Portal)
// ============================================================

use anyhow::Result;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs};
use esp_idf_svc::sys::{esp_restart, esp_wifi_set_ps, wifi_ps_type_t_WIFI_PS_NONE};
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi,
};
use std::time::Duration;

/// Hàm giải mã URL (để xử lý dấu cách, ký tự đặc biệt khi submit Form)
fn url_decode(encoded: &str) -> String {
    let mut decoded = String::new();
    let mut chars = encoded.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '+' {
            decoded.push(' ');
        } else if c == '%' {
            let hex: String = chars.by_ref().take(2).collect();
            if let Ok(byte) = u8::from_str_radix(&hex, 16) {
                decoded.push(byte as char);
            }
        } else {
            decoded.push(c);
        }
    }
    decoded
}

fn start_captive_dns(ip_addr: std::net::Ipv4Addr) {
    std::thread::spawn(move || {
        let socket = match std::net::UdpSocket::bind("0.0.0.0:53") {
            Ok(s) => s,
            Err(e) => {
                log::error!("Failed to bind DNS port: {:?}", e);
                return;
            }
        };
        log::info!("DNS Server started on port 53 for Captive Portal");

        let mut buf = [0u8; 512];
        loop {
            if let Ok((amt, src)) = socket.recv_from(&mut buf) {
                if amt >= 12 {
                    // Extract Query Type
                    let mut idx = 12;
                    while idx < amt && buf[idx] != 0 {
                        idx += buf[idx] as usize + 1;
                    }
                    let mut qtype = 0;
                    if idx + 4 <= amt {
                        qtype = u16::from_be_bytes([buf[idx + 1], buf[idx + 2]]);
                    }

                    log::info!("DNS Query from {}: Type {}", src, qtype);

                    let mut response = Vec::with_capacity(amt + 16);
                    response.extend_from_slice(&buf[..amt]);

                    // Standard DNS response flags
                    response[2] = 0x81;
                    response[3] = 0x80;

                    if qtype == 1 { // Type A (IPv4)
                        response[6] = 0x00;
                        response[7] = 0x01; // 1 Answer
                        
                        // Answer: Name pointer (C0 0C), Type A, Class IN, TTL 60, Data Length 4, IP
                        response.extend_from_slice(&[0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x04]);
                        response.extend_from_slice(&ip_addr.octets());
                    } else {
                        // Type AAAA (IPv6) or others -> 0 Answers, NOERROR
                        response[6] = 0x00;
                        response[7] = 0x00;
                    }

                    let _ = socket.send_to(&response, src);
                }
            }
        }
    });
}

use esp_idf_hal::i2c::I2cDriver;
use std::sync::{Arc, Mutex};
use crate::drivers::oled::Oled;

/// Connect to WiFi as Station (STA) mode or start AP Captive Portal
pub fn connect_wifi<'a>(
    modem: Modem,
    sysloop: &EspSystemEventLoop,
    nvs_partition: &EspDefaultNvsPartition,
    i2c: Arc<Mutex<I2cDriver<'a>>>,
) -> Result<()> {
    // 1. Đọc SSID và PASS đã lưu trong bộ nhớ NVS
    let nvs = EspNvs::new(nvs_partition.clone(), "wifi_config", true)?;
    let mut ssid_buf = [0u8; 32];
    let mut pass_buf = [0u8; 64];

    let saved_ssid = nvs.get_str("ssid", &mut ssid_buf)?.unwrap_or("").to_string();
    let saved_pass = nvs.get_str("pass", &mut pass_buf)?.unwrap_or("").to_string();

    let esp_wifi = EspWifi::new(modem, sysloop.clone(), Some(nvs_partition.clone()))?;
    let mut wifi = BlockingWifi::wrap(esp_wifi, sysloop.clone())?;

    let mut connected = false;

    // 2. Thử kết nối nếu đã có SSID
    if !saved_ssid.is_empty() {
        log::info!("[WIFI] Found saved SSID: '{}'. Trying to connect...", saved_ssid);

        wifi.set_configuration(&Configuration::Client(ClientConfiguration {
            ssid: saved_ssid.as_str().try_into().unwrap(),
            password: saved_pass.as_str().try_into().unwrap(),
            auth_method: if saved_pass.is_empty() {
                AuthMethod::None
            } else {
                AuthMethod::WPA2Personal
            },
            ..Default::default()
        }))?;

        if wifi.start().is_ok() && wifi.connect().is_ok() {
            log::info!("[WIFI] Waiting for IP...");
            if wifi.wait_netif_up().is_ok() {
                connected = true;
            }
        }
    }

    // 3. Kết nối thành công -> Tắt tiết kiệm pin và return
    if connected {
        let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
        log::info!("[WIFI] Connected! IP: {}", ip_info.ip);
        
        unsafe { esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE); }
        std::mem::forget(wifi); // Leak to keep alive
        return Ok(());
    }

    // 4. Kết nối thất bại -> Bật AP+STA mode (để vừa phát WiFi vừa scan)
    log::warn!("[WIFI] Connect failed. Starting AP+STA Mode (WiFiManager)...");

    wifi.set_configuration(&Configuration::Mixed(
        ClientConfiguration::default(),
        AccessPointConfiguration {
            ssid: "AMR_SETUP".try_into().unwrap(),
            password: "".try_into().unwrap(),
            auth_method: AuthMethod::None,
            channel: 1,
            ..Default::default()
        },
    ))?;

    wifi.start()?;
    // Không gọi wait_netif_up() vì Mixed mode sẽ timeout chờ STA
    // AP interface lên ngay sau start()
    std::thread::sleep(Duration::from_millis(500));

    let ap_ip = wifi.wifi().ap_netif().get_ip_info()?.ip;
    log::info!("==========================================");
    log::info!("[WIFI] AP Started! SSID: 'AMR_SETUP'");
    log::info!("[WIFI] Gateway: http://{}", ap_ip);
    log::info!("==========================================");

    if let Ok(mut i2c_lock) = i2c.lock() {
        let _ = Oled::draw_qr_url(&mut *i2c_lock, "http://192.168.71.1");
    }

    // Scan WiFi networks xung quanh
    log::info!("[WIFI] Scanning for nearby networks...");
    let mut network_names: Vec<String> = Vec::new();
    match wifi.scan() {
        Ok(results) => {
            for ap in &results {
                let name = ap.ssid.to_string();
                if !name.is_empty() && !network_names.contains(&name) {
                    network_names.push(name);
                }
            }
            log::info!("[WIFI] Found {} unique networks", network_names.len());
            for n in &network_names {
                log::info!("  📶 {}", n);
            }
        }
        Err(e) => log::error!("[WIFI] Scan failed: {:?}", e),
    }

    // Giữ wifi alive
    std::mem::forget(wifi);

    // Bật DNS Captive Portal
    start_captive_dns(ap_ip);

    // Build HTML động với danh sách WiFi
    let mut options_html = String::new();
    for name in &network_names {
        options_html.push_str(&format!("<option value='{}'>", name));
    }

    let setup_html = format!(
        concat!(
            "<!DOCTYPE html><html><head>",
            "<meta charset='utf-8'>",
            "<meta name='viewport' content='width=device-width,initial-scale=1'>",
            "<title>AMR WiFi Setup</title>",
            "<style>",
            "*{{box-sizing:border-box;margin:0;padding:0}}",
            "body{{font-family:-apple-system,Arial,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;display:flex;align-items:center;justify-content:center;padding:16px}}",
            ".card{{background:#fff;border-radius:20px;padding:32px 24px;width:100%;max-width:380px;box-shadow:0 20px 60px rgba(0,0,0,.3)}}",
            "h2{{color:#333;margin-bottom:8px;font-size:24px;text-align:center}}",
            ".sub{{color:#888;font-size:14px;text-align:center;margin-bottom:24px}}",
            "label{{display:block;font-size:13px;color:#555;margin-bottom:4px;margin-top:16px;font-weight:600}}",
            "input{{width:100%;padding:14px;font-size:16px;border:2px solid #e0e0e0;border-radius:12px;outline:none;transition:border .2s}}",
            "input:focus{{border-color:#667eea}}",
            "button{{width:100%;padding:16px;font-size:18px;font-weight:700;background:linear-gradient(135deg,#667eea,#764ba2);color:#fff;border:none;border-radius:12px;margin-top:24px;cursor:pointer;transition:transform .1s}}",
            "button:active{{transform:scale(.97)}}",
            ".count{{text-align:center;color:#aaa;font-size:12px;margin-top:16px}}",
            "</style></head><body>",
            "<div class='card'>",
            "<h2>AMR 2.0</h2>",
            "<p class='sub'>WiFi Configuration</p>",
            "<form action='/save' method='post'>",
            "<label>WiFi Network</label>",
            "<input type='text' name='ssid' list='nets' placeholder='Select or type WiFi name...' required autocomplete='off'>",
            "<datalist id='nets'>{}</datalist>",
            "<label>Password</label>",
            "<input type='password' name='pass' placeholder='Enter password'>",
            "<button type='submit'>Connect &amp; Reboot</button>",
            "</form>",
            "<p class='count'>{} networks found</p>",
            "</div></body></html>",
        ),
        options_html,
        network_names.len()
    );

    let saved_html = concat!(
        "<!DOCTYPE html><html><head>",
        "<meta charset='utf-8'>",
        "<meta name='viewport' content='width=device-width,initial-scale=1'>",
        "<style>",
        "*{box-sizing:border-box;margin:0;padding:0}",
        "body{font-family:-apple-system,Arial,sans-serif;background:linear-gradient(135deg,#56ab2f,#a8e063);min-height:100vh;display:flex;align-items:center;justify-content:center;padding:16px}",
        ".card{background:#fff;border-radius:20px;padding:40px 24px;width:100%;max-width:380px;box-shadow:0 20px 60px rgba(0,0,0,.3);text-align:center}",
        "h2{color:#56ab2f;font-size:28px;margin-bottom:12px}",
        "p{color:#666;font-size:16px}",
        "</style></head><body>",
        "<div class='card'>",
        "<h2>Saved!</h2>",
        "<p>Robot is rebooting...<br>Please reconnect to your home WiFi.</p>",
        "</div></body></html>",
    ).to_string();

    // Raw TCP Web Server
    let nvs_clone = nvs_partition.clone();

    std::thread::Builder::new()
        .name("tcp_web".into())
        .stack_size(8192)
        .spawn(move || {
            use std::io::{Read, Write as IoWrite};
            use std::net::TcpListener;

            let listener = match TcpListener::bind("0.0.0.0:80") {
                Ok(l) => l,
                Err(e) => {
                    log::error!("[WEB] Failed to bind port 80: {:?}", e);
                    return;
                }
            };
            log::info!("[WEB] TCP server ready on port 80");

            for stream in listener.incoming() {
                let mut stream = match stream {
                    Ok(s) => s,
                    Err(_) => continue,
                };

                let mut buf = [0u8; 2048];
                let n = match stream.read(&mut buf) {
                    Ok(n) if n > 0 => n,
                    _ => continue,
                };
                let req = String::from_utf8_lossy(&buf[..n]);
                let first_line = req.lines().next().unwrap_or("");
                log::info!("[WEB] {}", first_line);

                if first_line.starts_with("POST /save") {
                    let body = req.splitn(2, "\r\n\r\n").nth(1).unwrap_or("");
                    let mut ssid = String::new();
                    let mut pass = String::new();
                    for param in body.split('&') {
                        let mut kv = param.splitn(2, '=');
                        let key = kv.next().unwrap_or("");
                        let val = kv.next().unwrap_or("");
                        if key == "ssid" { ssid = url_decode(val); }
                        if key == "pass" { pass = url_decode(val); }
                    }

                    if !ssid.is_empty() {
                        log::info!("[WEB] Saving: SSID='{}' PASS={}", ssid, if pass.is_empty() { "(none)" } else { "***" });
                        if let Ok(nvs) = EspNvs::new(nvs_clone.clone(), "wifi_config", true) {
                            let _ = nvs.set_str("ssid", &ssid);
                            let _ = nvs.set_str("pass", &pass);
                        }
                    }

                    let resp = format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                        saved_html.len(), saved_html
                    );
                    let _ = stream.write_all(resp.as_bytes());
                    let _ = stream.flush();
                    drop(stream);

                    std::thread::sleep(Duration::from_secs(2));
                    unsafe { esp_restart(); }
                } else {
                    // Phân tích URI từ request line
                    let uri = first_line.split_whitespace().nth(1).unwrap_or("/");

                    // Kiểm tra Host header — nếu là IP trực tiếp thì serve trang setup
                    let is_direct_ip = req.lines().any(|l| {
                        let l = l.to_lowercase();
                        l.starts_with("host:") && l.contains("192.168.71.1")
                    });

                    // Nếu không có Host header (hiếm) nhưng uri là "/"
                    let is_root = uri == "/";

                    if is_direct_ip || (is_root && !req.to_lowercase().contains("host:")) {
                        // Truy cập trực tiếp IP → serve trang setup
                        let resp = format!(
                            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                            setup_html.len(), setup_html
                        );
                        let _ = stream.write_all(resp.as_bytes());
                        let _ = stream.flush();
                        log::info!("[WEB] ✅ Setup page sent ({} bytes)", setup_html.len());
                    } else {
                        // Captive Portal Probe → 302 redirect để trigger popup
                        log::info!("[WEB] 🔀 Captive portal redirect for: {}", uri);
                        let redirect = "HTTP/1.1 302 Found\r\nLocation: http://192.168.71.1/\r\nContent-Length: 0\r\nConnection: close\r\nCache-Control: no-cache, no-store\r\n\r\n";
                        let _ = stream.write_all(redirect.as_bytes());
                        let _ = stream.flush();
                    }
                }
            }
        })?;

    loop {
        std::thread::sleep(Duration::from_secs(10));
    }
}

