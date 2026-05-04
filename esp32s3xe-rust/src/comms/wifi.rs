// ============================================================
//   AMR 2.0 — WiFi Connection (esp-idf-svc)
//   Replaces WiFiManager captive portal with direct STA connect
// ============================================================

use esp_idf_svc::wifi::{
    AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi,
};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::sys::esp_wifi_set_ps;
use anyhow::Result;

/// WiFi credentials — thay đổi theo mạng nhà bạn
const WIFI_SSID: &str = "YOUR_WIFI_SSID";
const WIFI_PASS: &str = "YOUR_WIFI_PASS";

/// Connect to WiFi as Station (STA) mode.
/// Keeps wifi alive for the lifetime of the program (leaked intentionally).
pub fn connect_wifi(
    modem: Modem,
    sysloop: &EspSystemEventLoop,
    nvs: &EspDefaultNvsPartition,
) -> Result<()> {
    let esp_wifi = EspWifi::new(modem, sysloop.clone(), Some(nvs.clone()))?;
    let mut wifi = BlockingWifi::wrap(esp_wifi, sysloop.clone())?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: WIFI_SSID.try_into().unwrap(),
        password: WIFI_PASS.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    }))?;

    wifi.start()?;
    log::info!("[WIFI] Started, connecting to '{}'...", WIFI_SSID);

    wifi.connect()?;
    log::info!("[WIFI] Connected!");

    wifi.wait_netif_up()?;
    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
    log::info!("[WIFI] IP: {}, GW: {}", ip_info.ip, ip_info.subnet.gateway);

    // Disable power saving for low-latency WebSocket
    unsafe {
        esp_wifi_set_ps(esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_NONE);
    }

    // Leak the wifi object to keep connection alive for entire program lifetime.
    // This is the standard pattern for esp-idf-svc on embedded (no Drop).
    std::mem::forget(wifi);

    Ok(())
}
