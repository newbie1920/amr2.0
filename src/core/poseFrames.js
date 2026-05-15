import { normalizeAngle } from './mathUtils.js';

function finiteNumber(value) {
  return Number.isFinite(value) ? value : null;
}

function telemetryPose(telem = {}) {
  const x = finiteNumber(telem.x) ?? 0;
  const y = finiteNumber(telem.y) ?? 0;
  const theta = finiteNumber(telem.headingRad)
    ?? (finiteNumber(telem.heading) != null ? telem.heading * Math.PI / 180 : 0);
  return { x, y, theta: normalizeAngle(theta), source: 'odom', frame: 'odom' };
}

export function isPcSlamRobot(robot) {
  const telem = robot?.telemetry || {};
  return !!(
    robot &&
    !robot._sim &&
    !(telem.hitl || robot.connection?.hitlEnabled) &&
    (
      robot.connection?.architectureProfile === 'pc_slam' ||
      robot.adapter?.architectureProfile === 'pc_slam' ||
      telem.architecture === 'pc_slam' ||
      telem.onboardNavEnabled === false
    )
  );
}

export function resolveMapFramePose(robot, mapState, robotId) {
  const telem = robot?.telemetry || {};
  const odom = telemetryPose(telem);

  if (!isPcSlamRobot(robot)) {
    return odom;
  }

  const slamX = finiteNumber(telem.slamMapX);
  const slamY = finiteNumber(telem.slamMapY);
  const slamTheta = finiteNumber(telem.slamMapTheta);
  if (slamX != null && slamY != null) {
    return {
      x: slamX,
      y: slamY,
      theta: normalizeAngle(slamTheta ?? odom.theta),
      source: 'slam',
      frame: 'map',
    };
  }

  const tf = mapState?.mapToOdom?.[robotId];
  if (tf && Number.isFinite(tf.dx) && Number.isFinite(tf.dy)) {
    return {
      x: odom.x + tf.dx,
      y: odom.y + tf.dy,
      theta: normalizeAngle(odom.theta + (Number.isFinite(tf.dTheta) ? tf.dTheta : 0)),
      source: 'mapToOdom',
      frame: 'map',
    };
  }

  return odom;
}
