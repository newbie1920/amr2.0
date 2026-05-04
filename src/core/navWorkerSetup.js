/**
 * AMR 2.0 — Navigation Worker Setup
 * 
 * navWorker.js chỉ dùng cho:
 *   1. Simulation navigation (simStore, exploration)
 *   2. Scan matching (SLAM — mapStore)
 *   3. App-side Pure Pursuit recovery replan (navStore)
 * 
 * Đối với robot thật (ESP32 onboard A*): GOTO command gửi thẳng
 * qua connection.goto() — KHÔNG qua worker.
 * 
 * Export:
 *   - simNavWorkerApi: Tên mới, rõ ràng hơn (khuyến nghị dùng)
 *   - navWorkerApi: Backward-compatible alias (sẽ deprecated dần)
 */

import * as Comlink from 'comlink';
import NavWorker from './navWorker.js?worker';

let _workerApi = null;
if (typeof Worker !== 'undefined') {
  _workerApi = Comlink.wrap(new NavWorker());
}

// Primary export — simulation/SLAM worker
export const simNavWorkerApi = _workerApi;

// Backward-compatible alias — will be deprecated
export const navWorkerApi = _workerApi;
