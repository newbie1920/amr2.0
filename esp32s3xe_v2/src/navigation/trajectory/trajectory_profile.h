/**
 * AMR 2.0 v2 - lightweight trajectory reference generator.
 *
 * The default legacy ramp preserves the proven behavior. The opt-in adaptive
 * S-curve mode precomputes one fixed-size segment plan at a time and samples a
 * jerk-limited Type IV/III/I profile without heap allocation.
 */

#ifndef TRAJECTORY_PROFILE_H
#define TRAJECTORY_PROFILE_H

#include <Arduino.h>
#include "config.h"

enum TrajectoryProfileMode : uint8_t {
    TRAJ_PROFILE_LEGACY_RAMP = 0,
    TRAJ_PROFILE_ADAPTIVE_SCURVE = 1
};

static inline const char* trajectoryProfileModeName(TrajectoryProfileMode mode) {
    return mode == TRAJ_PROFILE_ADAPTIVE_SCURVE ? "adaptive_scurve" : "legacy_ramp";
}

struct TrajectoryWaypoint {
    float x = 0.0f;
    float y = 0.0f;
    float heading = NAN;
    bool useReverse = false;
};

struct TrajectoryConfig {
    bool enabled = true;
    TrajectoryProfileMode mode = TRAJ_PROFILE_LEGACY_RAMP;
    float vMax = 0.30f;
    float aMax = 0.30f;
    float jMax = 1.00f;
};

struct TrajectoryReference {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float v = 0.0f;
    float w = 0.0f;
    float segmentProgress = 0.0f;
    float segmentTime = 0.0f;
    float segmentDuration = 0.0f;
    const char* profileType = "idle";
    const char* profileMode = "legacy_ramp";
    int targetIndex = 0;
    bool active = false;
    bool done = false;
};

class TrajectoryProfile {
public:
    void loadPath(const TrajectoryWaypoint* input, int count,
                  float maxVel, float maxAccel, float maxJerk) {
        TrajectoryConfig cfg;
        cfg.vMax = maxVel;
        cfg.aMax = maxAccel;
        cfg.jMax = maxJerk;
        loadPath(input, count, cfg);
    }

    void loadPath(const TrajectoryWaypoint* input, int count,
                  const TrajectoryConfig& cfg) {
        _count = constrain(count, 0, MAX_WAYPOINTS);
        for (int i = 0; i < _count; i++) {
            _points[i] = input[i];
        }

        _config = sanitizeConfig(cfg);
        _speed = 0.0f;
        _accel = 0.0f;
        _distanceOnSegment = 0.0f;
        _segmentTime = 0.0f;
        _segmentPlan = SegmentPlan{};
        _targetIndex = (_count > 1) ? 1 : 0;
        _active = _count > 0;
        _done = _count == 0;

        if (_active) {
            _ref.x = _points[0].x;
            _ref.y = _points[0].y;
            _ref.theta = (_count > 1)
                ? atan2f(_points[_targetIndex].y - _points[0].y, _points[_targetIndex].x - _points[0].x)
                : 0.0f;
            _ref.v = 0.0f;
            _ref.w = 0.0f;
            _ref.segmentProgress = 0.0f;
            _ref.segmentTime = 0.0f;
            _ref.segmentDuration = 0.0f;
            _ref.profileType = initialProfileType();
            _ref.profileMode = trajectoryProfileModeName(_config.mode);
            _ref.targetIndex = _targetIndex;
            _ref.active = true;
            _ref.done = false;
        }
    }

    void setConfig(const TrajectoryConfig& cfg) {
        _config = sanitizeConfig(cfg);
        _segmentPlan = SegmentPlan{};
    }

    const TrajectoryConfig& config() const { return _config; }

    void reset() {
        _count = 0;
        _targetIndex = 0;
        _speed = 0.0f;
        _accel = 0.0f;
        _distanceOnSegment = 0.0f;
        _segmentTime = 0.0f;
        _segmentPlan = SegmentPlan{};
        _active = false;
        _done = true;
        _ref = TrajectoryReference{};
        _ref.done = true;
    }

    TrajectoryReference sample(float dt) {
        if (!_active || _done || _count <= 0) {
            _ref.active = false;
            _ref.done = true;
            _ref.v = 0.0f;
            _ref.w = 0.0f;
            _ref.segmentProgress = 1.0f;
            _ref.segmentTime = 0.0f;
            _ref.segmentDuration = 0.0f;
            _ref.profileType = "idle";
            _ref.profileMode = trajectoryProfileModeName(_config.mode);
            return _ref;
        }

        if (_count == 1 || _targetIndex >= _count) {
            _done = true;
            _speed = 0.0f;
            _accel = 0.0f;
            _segmentTime = 0.0f;
            _ref.v = 0.0f;
            _ref.w = 0.0f;
            _ref.segmentProgress = 1.0f;
            _ref.segmentTime = 0.0f;
            _ref.segmentDuration = 0.0f;
            _ref.profileType = "done";
            _ref.profileMode = trajectoryProfileModeName(_config.mode);
            _ref.done = true;
            return _ref;
        }

        const TrajectoryWaypoint& from = _points[_targetIndex - 1];
        const TrajectoryWaypoint& to = _points[_targetIndex];
        float dx = to.x - from.x;
        float dy = to.y - from.y;
        float segLen = sqrtf(dx * dx + dy * dy);

        if (segLen < 0.001f) {
            advanceSegment(to);
            return sample(dt);
        }

        if (_config.enabled && _config.mode == TRAJ_PROFILE_ADAPTIVE_SCURVE) {
            return sampleAdaptiveScurve(from, to, dx, dy, segLen, dt);
        }

        return sampleLegacyRamp(from, to, dx, dy, segLen, dt);
    }

    const TrajectoryReference& current() const { return _ref; }
    int currentTargetIndex() const { return _targetIndex; }
    bool isFinished() const { return _done; }

private:
    struct SegmentPlan {
        float distance = 0.0f;
        float tJ = 0.0f;
        float tA = 0.0f;
        float tV = 0.0f;
        float tTotal = 0.0f;
        float vPeak = 0.0f;
        const char* profileType = "idle";
        int targetIndex = -1;
        bool ready = false;
    };

    struct MotionSample {
        float s = 0.0f;
        float v = 0.0f;
        float a = 0.0f;
    };

    TrajectoryWaypoint _points[MAX_WAYPOINTS];
    int _count = 0;
    int _targetIndex = 0;
    TrajectoryConfig _config;
    SegmentPlan _segmentPlan;
    float _speed = 0.0f;
    float _accel = 0.0f;
    float _distanceOnSegment = 0.0f;
    float _segmentTime = 0.0f;
    bool _active = false;
    bool _done = true;
    TrajectoryReference _ref;

    static float normalizeAngle(float a) {
        while (a > PI) a -= 2.0f * PI;
        while (a < -PI) a += 2.0f * PI;
        return a;
    }

    static TrajectoryConfig sanitizeConfig(const TrajectoryConfig& input) {
        TrajectoryConfig cfg = input;
        cfg.vMax = fmaxf(0.02f, cfg.vMax);
        cfg.aMax = fmaxf(0.05f, cfg.aMax);
        cfg.jMax = fmaxf(0.10f, cfg.jMax);
        if (cfg.mode != TRAJ_PROFILE_ADAPTIVE_SCURVE) {
            cfg.mode = TRAJ_PROFILE_LEGACY_RAMP;
        }
        return cfg;
    }

    const char* initialProfileType() const {
        if (!_config.enabled) return "direct";
        return _config.mode == TRAJ_PROFILE_ADAPTIVE_SCURVE ? "type_i" : "legacy_ramp";
    }

    const char* classifyLegacyProfile(float segLen) const {
        if (!_config.enabled) return "direct";

        float accelDistance = (_config.vMax * _config.vMax) / fmaxf(_config.aMax, 0.001f);
        float jerkDistance = (_config.aMax * _config.aMax * _config.aMax) /
                             fmaxf(_config.jMax * _config.jMax, 0.001f);

        if (segLen >= accelDistance) return "type_iv";
        if (segLen >= jerkDistance) return "type_iii";
        return "type_i";
    }

    SegmentPlan buildAdaptivePlan(float segLen) const {
        SegmentPlan plan;
        plan.distance = segLen;
        plan.targetIndex = _targetIndex;
        plan.ready = true;

        const float j = _config.jMax;
        const float a = _config.aMax;
        const float v = _config.vMax;
        const float tJMax = a / j;
        const float dTypeIBoundary = 2.0f * j * tJMax * tJMax * tJMax;

        if (segLen <= dTypeIBoundary) {
            plan.tJ = powf(segLen / (2.0f * j), 1.0f / 3.0f);
            plan.tA = 0.0f;
            plan.tV = 0.0f;
            plan.vPeak = j * plan.tJ * plan.tJ;
            plan.profileType = "type_i";
        } else {
            plan.tJ = tJMax;
            float root = sqrtf(fmaxf(0.0f, tJMax * tJMax + 4.0f * segLen / a));
            float tAForDistance = fmaxf(0.0f, (-3.0f * tJMax + root) * 0.5f);
            float peakVNoCruise = a * (tAForDistance + tJMax);

            if (peakVNoCruise <= v) {
                plan.tA = tAForDistance;
                plan.tV = 0.0f;
                plan.vPeak = peakVNoCruise;
                plan.profileType = "type_iii";
            } else {
                plan.tA = fmaxf(0.0f, v / a - tJMax);
                float accelHalfDistance = a * tJMax * tJMax +
                                          1.5f * a * tJMax * plan.tA +
                                          0.5f * a * plan.tA * plan.tA;
                float noCruiseDistance = 2.0f * accelHalfDistance;

                if (noCruiseDistance > segLen) {
                    plan.tJ = powf(segLen / (2.0f * j), 1.0f / 3.0f);
                    plan.tA = 0.0f;
                    plan.tV = 0.0f;
                    plan.vPeak = j * plan.tJ * plan.tJ;
                    plan.profileType = "type_i";
                } else {
                    plan.tV = (segLen - noCruiseDistance) / v;
                    plan.vPeak = v;
                    plan.profileType = "type_iv";
                }
            }
        }

        plan.tTotal = 4.0f * plan.tJ + 2.0f * plan.tA + plan.tV;
        if (plan.tTotal < 0.001f) {
            plan.tTotal = 0.001f;
        }
        return plan;
    }

    MotionSample sampleMotion(const SegmentPlan& plan, float time) const {
        MotionSample sample;
        float remaining = constrain(time, 0.0f, plan.tTotal);
        const float durations[7] = {
            plan.tJ, plan.tA, plan.tJ, plan.tV, plan.tJ, plan.tA, plan.tJ
        };
        const float jerks[7] = {
            _config.jMax, 0.0f, -_config.jMax, 0.0f, -_config.jMax, 0.0f, _config.jMax
        };

        for (int i = 0; i < 7 && remaining > 0.0f; i++) {
            float tau = fminf(remaining, durations[i]);
            float jerk = jerks[i];
            sample.s += sample.v * tau + 0.5f * sample.a * tau * tau +
                        (jerk * tau * tau * tau) / 6.0f;
            sample.v += sample.a * tau + 0.5f * jerk * tau * tau;
            sample.a += jerk * tau;
            remaining -= tau;
        }

        sample.s = constrain(sample.s, 0.0f, plan.distance);
        sample.v = constrain(sample.v, 0.0f, _config.vMax);
        return sample;
    }

    void publishReference(const TrajectoryWaypoint& from, const TrajectoryWaypoint& to,
                          float dx, float dy, float segLen, float distanceOnSegment,
                          float speed, const char* profileType, float segmentTime,
                          float segmentDuration, float dt) {
        float t = constrain(distanceOnSegment / fmaxf(segLen, 0.001f), 0.0f, 1.0f);
        float prevTheta = _ref.theta;
        _ref.x = from.x + dx * t;
        _ref.y = from.y + dy * t;
        _ref.theta = atan2f(dy, dx);
        _ref.v = speed * (to.useReverse ? -1.0f : 1.0f);
        _ref.w = normalizeAngle(_ref.theta - prevTheta) / fmaxf(dt, 0.001f);
        _ref.segmentProgress = t;
        _ref.segmentTime = segmentTime;
        _ref.segmentDuration = segmentDuration;
        _ref.profileType = profileType;
        _ref.profileMode = trajectoryProfileModeName(_config.mode);
        _ref.targetIndex = _targetIndex;
        _ref.active = true;
        _ref.done = false;
    }

    TrajectoryReference sampleLegacyRamp(const TrajectoryWaypoint& from, const TrajectoryWaypoint& to,
                                         float dx, float dy, float segLen, float dt) {
        float remaining = fmaxf(0.0f, segLen - _distanceOnSegment);
        const char* profileType = classifyLegacyProfile(segLen);

        if (_config.enabled) {
            float brakingSpeed = sqrtf(2.0f * _config.aMax * remaining);
            float desiredSpeed = fminf(_config.vMax, brakingSpeed);
            float desiredAccel = (desiredSpeed > _speed) ? _config.aMax : -_config.aMax;
            float maxAccelStep = _config.jMax * dt;

            if (_accel < desiredAccel) _accel = fminf(_accel + maxAccelStep, desiredAccel);
            else if (_accel > desiredAccel) _accel = fmaxf(_accel - maxAccelStep, desiredAccel);

            _speed += _accel * dt;
            _speed = constrain(_speed, 0.0f, _config.vMax);
        } else {
            _accel = 0.0f;
            _speed = fminf(_config.vMax, remaining / fmaxf(dt, 0.001f));
            profileType = "direct";
        }

        float step = _speed * dt;
        if (step >= remaining) {
            _distanceOnSegment = segLen;
            _ref.x = to.x;
            _ref.y = to.y;
            _ref.theta = atan2f(dy, dx);
            _ref.v = 0.0f;
            _ref.w = 0.0f;
            _ref.segmentProgress = 1.0f;
            _ref.segmentTime = 0.0f;
            _ref.segmentDuration = 0.0f;
            _ref.profileType = profileType;
            _ref.profileMode = trajectoryProfileModeName(_config.mode);
            advanceSegment(to);
            return _ref;
        }

        _distanceOnSegment += step;
        publishReference(from, to, dx, dy, segLen, _distanceOnSegment, _speed,
                         profileType, 0.0f, 0.0f, dt);
        return _ref;
    }

    TrajectoryReference sampleAdaptiveScurve(const TrajectoryWaypoint& from, const TrajectoryWaypoint& to,
                                             float dx, float dy, float segLen, float dt) {
        if (!_segmentPlan.ready || _segmentPlan.targetIndex != _targetIndex) {
            _segmentPlan = buildAdaptivePlan(segLen);
            _segmentTime = 0.0f;
            _distanceOnSegment = 0.0f;
        }

        _segmentTime = fminf(_segmentTime + dt, _segmentPlan.tTotal);
        MotionSample motion = sampleMotion(_segmentPlan, _segmentTime);
        _distanceOnSegment = motion.s;

        if (_segmentTime >= _segmentPlan.tTotal || _distanceOnSegment >= segLen - 0.0005f) {
            _distanceOnSegment = segLen;
            _ref.x = to.x;
            _ref.y = to.y;
            _ref.theta = atan2f(dy, dx);
            _ref.v = 0.0f;
            _ref.w = 0.0f;
            _ref.segmentProgress = 1.0f;
            _ref.segmentTime = _segmentPlan.tTotal;
            _ref.segmentDuration = _segmentPlan.tTotal;
            _ref.profileType = _segmentPlan.profileType;
            _ref.profileMode = trajectoryProfileModeName(_config.mode);
            advanceSegment(to);
            return _ref;
        }

        publishReference(from, to, dx, dy, segLen, _distanceOnSegment, motion.v,
                         _segmentPlan.profileType, _segmentTime, _segmentPlan.tTotal, dt);
        return _ref;
    }

    void advanceSegment(const TrajectoryWaypoint& arrived) {
        _speed = 0.0f;
        _accel = 0.0f;
        _distanceOnSegment = 0.0f;
        _segmentTime = 0.0f;
        _segmentPlan = SegmentPlan{};
        _targetIndex++;
        _ref.targetIndex = _targetIndex;

        if (_targetIndex >= _count) {
            _done = true;
            _ref.x = arrived.x;
            _ref.y = arrived.y;
            _ref.v = 0.0f;
            _ref.w = 0.0f;
            _ref.segmentProgress = 1.0f;
            _ref.profileType = "done";
            _ref.profileMode = trajectoryProfileModeName(_config.mode);
            _ref.done = true;
            return;
        }

        _ref.done = false;
    }
};

#endif // TRAJECTORY_PROFILE_H
