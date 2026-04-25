import { useEffect, useState } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';

export default function TelemetryChart() {
  const robots = useRobotStore(state => state.robots);
  const selectedRobotId = useRobotStore(state => state.selectedRobotId);
  const [data, setData] = useState([]);
  
  const activeRobot = selectedRobotId ? robots[selectedRobotId] : Object.values(robots).find(r => r.status === 'connected');

  useEffect(() => {
    if (!activeRobot || !activeRobot.telemetry) return;
    
    // Only capture when navigating to not clutter with zeros when IDLE
    // But for demonstration, we can capture all the time.
    setData(prev => {
      // Create a short time label like "14:05:02"
      const d = new Date();
      const timeStr = `${d.getMinutes().toString().padStart(2, '0')}:${d.getSeconds().toString().padStart(2, '0')}`;

      const newLine = {
        name: timeStr,
        eX: parseFloat(activeRobot.telemetry.eX || 0).toFixed(3),
        eY: parseFloat(activeRobot.telemetry.eY || 0).toFixed(3),
        eYaw: parseFloat(activeRobot.telemetry.eYaw || 0).toFixed(3)
      };
      
      const newData = [...prev, newLine];
      if (newData.length > 50) {
        newData.shift(); // Chỉ giữ 50 mẫu gần nhất
      }
      return newData;
    });
  }, [activeRobot?.telemetry]);

  if (!activeRobot) {
    return null;
  }

  return (
    <div className="panel" style={{ marginTop: '16px' }}>
      <div className="panel__header">
        <div style={{ display: 'flex', flexDirection: 'column' }}>
          <span className="panel__title">📈 Cross-track Error (Lyapunov)</span>
          <span style={{ fontSize: '10px', color: 'var(--text-muted)' }}>Real-time telemetry tracking - Robot: {activeRobot.id}</span>
        </div>
      </div>
      <div style={{ height: '220px', width: '100%', fontSize: '11px' }}>
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data} margin={{ top: 10, right: 10, left: -25, bottom: 0 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
            <XAxis dataKey="name" stroke="var(--text-muted)" tick={{fontSize: 9}} tickMargin={5} />
            <YAxis stroke="var(--text-muted)" tick={{fontSize: 9}} width={50} />
            <Tooltip 
              contentStyle={{ backgroundColor: 'var(--bg-dark)', borderColor: 'var(--border)', borderRadius: '6px' }}
              itemStyle={{ fontSize: '12px' }}
              labelStyle={{ fontSize: '11px', color: 'var(--text-muted)', marginBottom: '4px' }}
            />
            <Legend wrapperStyle={{ fontSize: '11px', paddingTop: '10px' }} iconType="circle" />
            <Line type="monotone" dataKey="eX" stroke="#8884d8" name="Error X (m)" dot={false} strokeWidth={2} isAnimationActive={false} />
            <Line type="monotone" dataKey="eY" stroke="#82ca9d" name="Error Y (m)" dot={false} strokeWidth={2} isAnimationActive={false} />
            <Line type="monotone" dataKey="eYaw" stroke="#ffc658" name="Error Yaw (rad)" dot={false} strokeWidth={2} isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
