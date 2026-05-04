// ============================================================
//   AMR 2.0 — WebSocket Server (Phase 1 stub)
//   Full implementation will use esp-idf-svc HTTP server
//   with WebSocket upgrade support
// ============================================================

use anyhow::Result;

/// WebSocket server placeholder
/// Phase 1: Stub that logs readiness
/// Phase 2+: Will handle GOTO, telemetry, MAP_DATA binary frames
pub struct WebSocketServer {
    port: u16,
}

impl WebSocketServer {
    pub fn new(port: u16) -> Self {
        Self { port }
    }

    pub fn start(&self) -> Result<()> {
        log::info!("[WS] WebSocket server ready on port {} (stub)", self.port);
        // TODO Phase 2: Implement full esp_idf_svc::http::server with WS upgrade
        Ok(())
    }
}
