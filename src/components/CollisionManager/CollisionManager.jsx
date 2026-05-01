import { useEffect, useRef } from 'react';
import useRobotStore from '../../stores/robotStore';
import useNavStore from '../../stores/navStore';

const SAFE_DISTANCE = 1.0; // Khoảng cách an toàn (mét)

/**
 * CollisionManager (Headless Component)
 * Giám sát khoảng cách không gian đa xe ở tần số 10Hz.
 * Yêu cầu xe nhường đường tự động dừng khẩn cấp nếu có nguy cơ va chạm.
 */
export default function CollisionManager() {
  const lastState = useRef({}); // { [robotId]: isPaused }

  useEffect(() => {
    const checkCollisions = () => {
      const { robots } = useRobotStore.getState();
      const { pauseNav, resumeNav } = useNavStore.getState();
      const connected = Object.values(robots).filter(r => r.status === 'connected' && r.telemetry);
      
      const toPause = new Set();
      
      // Quét từng cặp xe
      for(let i = 0; i < connected.length; i++) {
        for(let j = i + 1; j < connected.length; j++) {
            const r1 = connected[i];
            const r2 = connected[j];
            
            const dx = r1.telemetry.x - r2.telemetry.x;
            const dy = r1.telemetry.y - r2.telemetry.y;
            const dist = Math.hypot(dx, dy);
            
            if (dist < SAFE_DISTANCE) {
                // Xung đột. Quy tắc nhường đường: Xe ID lớn hơn phải nhường (Dừng PAUSE)
                if (r1.id < r2.id) {
                    toPause.add(r2.id); 
                } else {
                    toPause.add(r1.id);
                }
            }
        }
      }

      // Xử lý Gửi lệnh
      for(const r of connected) {
          const id = r.id;
          const isCurrentlyPaused = lastState.current[id];
          
          if (toPause.has(id)) {
              if (!isCurrentlyPaused) {
                  pauseNav(id);
                  lastState.current[id] = true;
                  console.warn(`[CollisionManager] ⚠️ Xe ${id} đã bị YIELD (Nhường đường) do xâm phạm Safe Radius.`);
              }
          } else {
              if (isCurrentlyPaused) {
                  resumeNav(id);
                  lastState.current[id] = false;
                  console.info(`[CollisionManager] 🟢 Xe ${id} được cấp RESUME (Hết cản trở).`);
              }
          }
      }
    };

    const interval = setInterval(checkCollisions, 100); // 10Hz quét liên tục
    return () => clearInterval(interval);
  }, []);

  return null;
}
