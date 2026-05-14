/**
 * AMR 2.0 — MQTT Auto-Discovery Service
 * 
 * Subscribes to MQTT broker (HiveMQ WebSocket) to auto-detect
 * robots broadcasting their IP/port. When a robot is discovered,
 * it's automatically added to robotStore and connected via WebSocket.
 * 
 * Topic: amr2/discovery/{robotId}
 * Payload: { id, name, ip, port, status, battery, firmware }
 */

import mqtt from 'mqtt';

// ============================================================
//   CONFIGURATION
// ============================================================

// Defaults are demo-friendly; production should set these to a private broker/topic.
const MQTT_BROKER_URL = import.meta.env.VITE_MQTT_BROKER_URL || 'wss://broker.hivemq.com:8884/mqtt';
const MQTT_TOPIC = import.meta.env.VITE_MQTT_DISCOVERY_TOPIC || 'amr2/discovery/#';
const HEARTBEAT_TIMEOUT_MS = 90000; // Remove robot if no heartbeat for 90s

// ============================================================
//   MQTT DISCOVERY SERVICE (Singleton)
// ============================================================

class MqttDiscoveryService {
  constructor() {
    this.client = null;
    this.connected = false;
    this.discoveredRobots = new Map(); // robotId → { ...info, lastSeen }
    this.onRobotDiscovered = null;     // callback(robotInfo)
    this.onRobotOffline = null;        // callback(robotId)
    this.cleanupTimer = null;
    this._reconnectCount = 0;
  }

  /**
   * Start MQTT client and subscribe to discovery topic
   */
  start() {
    if (this.client) return;

    console.log('[MQTT] Connecting to', MQTT_BROKER_URL);

    this.client = mqtt.connect(MQTT_BROKER_URL, {
      clientId: `amr2_web_${Math.random().toString(16).slice(2, 8)}`,
      clean: true,
      connectTimeout: 10000,
      reconnectPeriod: 5000,
      keepalive: 60,
    });

    this.client.on('connect', () => {
      console.log('[MQTT] ✅ Connected to broker');
      this.connected = true;
      this._reconnectCount = 0;

      this.client.subscribe(MQTT_TOPIC, { qos: 0 }, (err) => {
        if (err) {
          console.error('[MQTT] Subscribe error:', err);
        } else {
          console.log('[MQTT] Subscribed to', MQTT_TOPIC);
        }
      });
    });

    this.client.on('message', (topic, payload) => {
      this._handleMessage(topic, payload);
    });

    this.client.on('error', (err) => {
      console.error('[MQTT] Error:', err.message);
    });

    this.client.on('reconnect', () => {
      this._reconnectCount++;
      if (this._reconnectCount <= 3) {
        console.log(`[MQTT] Reconnecting... (attempt ${this._reconnectCount})`);
      }
    });

    this.client.on('close', () => {
      this.connected = false;
    });

    // Cleanup timer: remove robots that haven't sent heartbeat
    this.cleanupTimer = setInterval(() => this._cleanupStale(), 15000);
  }

  /**
   * Stop MQTT client
   */
  stop() {
    if (this.cleanupTimer) {
      clearInterval(this.cleanupTimer);
      this.cleanupTimer = null;
    }
    if (this.client) {
      this.client.end(true);
      this.client = null;
    }
    this.connected = false;
    this.discoveredRobots.clear();
  }

  /**
   * Handle incoming MQTT message
   */
  _handleMessage(topic, payload) {
    try {
      const data = JSON.parse(payload.toString());

      if (!data.id) return;

      // Robot going offline
      if (data.status === 'offline') {
        console.log(`[MQTT] 🔴 Robot offline: ${data.id}`);
        this.discoveredRobots.delete(data.id);
        if (this.onRobotOffline) this.onRobotOffline(data.id);
        return;
      }

      // Robot online — update or add
      const robotInfo = {
        id: data.id,
        name: data.name || data.id,
        ip: data.ip,
        port: data.port || 81,
        status: data.status || 'online',
        battery: data.battery || 0,
        firmware: data.firmware || 'unknown',
        lastSeen: Date.now(),
      };

      if (!robotInfo.ip || typeof robotInfo.ip !== 'string') {
        console.warn('[MQTT] Ignoring discovery payload without a valid IP:', data.id);
        return;
      }

      const isNew = !this.discoveredRobots.has(data.id);
      this.discoveredRobots.set(data.id, robotInfo);

      if (isNew) {
        console.log(`[MQTT] 🟢 New robot discovered: ${data.id} @ ${data.ip}:${data.port}`);
      }

      if (this.onRobotDiscovered) this.onRobotDiscovered(robotInfo, isNew);
    } catch (e) {
      // Ignore malformed messages
    }
  }

  /**
   * Remove robots that haven't sent heartbeat within timeout
   */
  _cleanupStale() {
    const now = Date.now();
    for (const [id, info] of this.discoveredRobots) {
      if (now - info.lastSeen > HEARTBEAT_TIMEOUT_MS) {
        console.log(`[MQTT] ⏱️ Robot timeout (no heartbeat): ${id}`);
        this.discoveredRobots.delete(id);
        if (this.onRobotOffline) this.onRobotOffline(id);
      }
    }
  }

  /**
   * Get all currently discovered robots
   */
  getDiscoveredRobots() {
    return Array.from(this.discoveredRobots.values());
  }

  /**
   * Check if service is connected
   */
  isConnected() {
    return this.connected;
  }
}

// Singleton instance
const mqttDiscovery = new MqttDiscoveryService();
export default mqttDiscovery;
