#include "sim_world.h"
#include <cmath>
#include <algorithm>
#include "esp_log.h"

static const char* TAG = "SimWorld";

SimWorld::SimWorld() {
    defaultSpawn = {3.5f, 2.0f, (float)(M_PI / 2.0)};
    buildStaticWorld();
}

void SimWorld::buildStaticWorld() {
    staticSegments.clear();

    float W = WAREHOUSE_WIDTH;
    float H = WAREHOUSE_HEIGHT;

    // Borders
    staticSegments.push_back({0, 0, W, 0, SegmentTag::WALL});
    staticSegments.push_back({0, H, W, H, SegmentTag::WALL});
    staticSegments.push_back({0, 0, 0, H, SegmentTag::WALL});
    staticSegments.push_back({W, 0, W, H, SegmentTag::WALL});

    // Shelves (Shelf 1, 2, 3, 4)
    struct Bounds { float x1, y1, x2, y2; };
    Bounds shelves[] = {
        {1.5f, 3.0f, 3.5f, 4.5f},
        {6.5f, 3.0f, 8.5f, 4.5f},
        {1.5f, 6.0f, 3.5f, 7.5f},
        {6.5f, 6.0f, 8.5f, 7.5f}
    };
    
    for (const auto& s : shelves) {
        staticSegments.push_back({s.x1, s.y1, s.x2, s.y1, SegmentTag::SHELF});
        staticSegments.push_back({s.x2, s.y1, s.x2, s.y2, SegmentTag::SHELF});
        staticSegments.push_back({s.x2, s.y2, s.x1, s.y2, SegmentTag::SHELF});
        staticSegments.push_back({s.x1, s.y2, s.x1, s.y1, SegmentTag::SHELF});
    }

    // Chargers
    struct Point { float x, y; };
    Point chargers[] = {
        {2.0f, 9.0f}, {4.0f, 9.0f}, {6.0f, 9.0f}, {8.0f, 9.0f}
    };
    for (const auto& c : chargers) {
        float hw = 0.5f, hd = 0.6f;
        staticSegments.push_back({c.x - hw, c.y + hd, c.x + hw, c.y + hd, SegmentTag::CHARGER}); // Back
        staticSegments.push_back({c.x - hw, c.y, c.x - hw, c.y + hd, SegmentTag::CHARGER});      // Left
        staticSegments.push_back({c.x + hw, c.y, c.x + hw, c.y + hd, SegmentTag::CHARGER});      // Right
    }

    rebuildSegments();
}

void SimWorld::loadSlamTestMap() {
    staticSegments.clear();
    dynamicObstacles.clear();

    float W = WAREHOUSE_WIDTH;
    float H = WAREHOUSE_HEIGHT;

    // Tường biên kín
    staticSegments.push_back({0, 0, W, 0, SegmentTag::WALL});
    staticSegments.push_back({W, 0, W, H, SegmentTag::WALL});
    staticSegments.push_back({W, H, 0, H, SegmentTag::WALL});
    staticSegments.push_back({0, H, 0, 0, SegmentTag::WALL});

    // Tường chia phòng
    staticSegments.push_back({0, 4, 2, 4, SegmentTag::WALL});
    staticSegments.push_back({3, 4, 6.5f, 4, SegmentTag::WALL});
    staticSegments.push_back({7.5f, 4, 10, 4, SegmentTag::WALL});

    staticSegments.push_back({0, 7, 4, 7, SegmentTag::WALL});
    staticSegments.push_back({5.5f, 7, 10, 7, SegmentTag::WALL});

    staticSegments.push_back({5, 0, 5, 1.5f, SegmentTag::WALL});
    staticSegments.push_back({5, 2.5f, 5, 4, SegmentTag::WALL});

    staticSegments.push_back({3, 7, 3, 9, SegmentTag::WALL});

    // Boxes
    struct Box { float cx, cy, w, h; };
    Box boxes[] = {
        {2.0f, 2.0f, 0.8f, 0.8f}, {7.5f, 2.0f, 1.0f, 0.6f}, {8.5f, 1.2f, 0.5f, 0.5f},
        {1.5f, 5.5f, 0.6f, 1.2f}, {8.0f, 5.5f, 1.2f, 0.6f}, {5.0f, 5.8f, 0.5f, 0.5f},
        {1.5f, 8.5f, 0.7f, 0.7f}, {7.0f, 8.5f, 1.0f, 0.8f}
    };
    
    for (const auto& b : boxes) {
        float x1 = b.cx - b.w / 2, y1 = b.cy - b.h / 2;
        float x2 = b.cx + b.w / 2, y2 = b.cy + b.h / 2;
        staticSegments.push_back({x1, y1, x2, y1, SegmentTag::OBSTACLE});
        staticSegments.push_back({x2, y1, x2, y2, SegmentTag::OBSTACLE});
        staticSegments.push_back({x2, y2, x1, y2, SegmentTag::OBSTACLE});
        staticSegments.push_back({x1, y2, x1, y1, SegmentTag::OBSTACLE});
    }

    // Pillars
    struct Pillar { float cx, cy, r; };
    Pillar pillars[] = { {4.0f, 5.5f, 0.25f}, {6.5f, 5.5f, 0.25f}, {5.0f, 8.5f, 0.3f} };
    int sides = 8;
    for (const auto& p : pillars) {
        for (int i = 0; i < sides; i++) {
            float a1 = (2 * M_PI * i) / sides;
            float a2 = (2 * M_PI * (i + 1)) / sides;
            staticSegments.push_back({
                p.cx + p.r * std::cos(a1), p.cy + p.r * std::sin(a1),
                p.cx + p.r * std::cos(a2), p.cy + p.r * std::sin(a2),
                SegmentTag::OBSTACLE
            });
        }
    }

    // L-shape
    staticSegments.push_back({8.5f, 4.5f, 8.5f, 6.0f, SegmentTag::OBSTACLE});
    staticSegments.push_back({8.5f, 6.0f, 9.5f, 6.0f, SegmentTag::OBSTACLE});

    defaultSpawn = {5.0f, 5.0f, (float)(-M_PI / 2.0)};
    rebuildSegments();
    ESP_LOGI(TAG, "SLAM Test Map loaded: %d segments", allSegments.size());
}

void SimWorld::rebuildSegments() {
    allSegments = staticSegments;
    for (const auto& pair : dynamicObstacles) {
        for (const auto& seg : pair.second.segments) {
            allSegments.push_back(seg);
        }
    }
}

void SimWorld::addBoxObstacle(const std::string& id, float cx, float cy, float w, float h) {
    float x1 = cx - w / 2, y1 = cy - h / 2;
    float x2 = cx + w / 2, y2 = cy + h / 2;
    
    DynamicObstacle obs;
    obs.bounds = {x1, y1, x2, y2};
    obs.segments.push_back({x1, y1, x2, y1, SegmentTag::OBSTACLE});
    obs.segments.push_back({x2, y1, x2, y2, SegmentTag::OBSTACLE});
    obs.segments.push_back({x2, y2, x1, y2, SegmentTag::OBSTACLE});
    obs.segments.push_back({x1, y2, x1, y1, SegmentTag::OBSTACLE});
    
    dynamicObstacles[id] = obs;
    rebuildSegments();
}

void SimWorld::addCircleObstacle(const std::string& id, float cx, float cy, float radius, int sides) {
    DynamicObstacle obs;
    obs.bounds = {cx - radius, cy - radius, cx + radius, cy + radius};
    
    for (int i = 0; i < sides; i++) {
        float a1 = (2 * M_PI * i) / sides;
        float a2 = (2 * M_PI * (i + 1)) / sides;
        obs.segments.push_back({
            cx + radius * std::cos(a1), cy + radius * std::sin(a1),
            cx + radius * std::cos(a2), cy + radius * std::sin(a2),
            SegmentTag::OBSTACLE
        });
    }
    
    dynamicObstacles[id] = obs;
    rebuildSegments();
}

void SimWorld::removeObstacle(const std::string& id) {
    dynamicObstacles.erase(id);
    rebuildSegments();
}

void SimWorld::moveObstacle(const std::string& id, float newCX, float newCY) {
    if (dynamicObstacles.find(id) == dynamicObstacles.end()) return;
    
    auto& obs = dynamicObstacles[id];
    float oldCX = (obs.bounds.x1 + obs.bounds.x2) / 2.0f;
    float oldCY = (obs.bounds.y1 + obs.bounds.y2) / 2.0f;
    float dx = newCX - oldCX;
    float dy = newCY - oldCY;
    
    obs.bounds.x1 += dx; obs.bounds.y1 += dy;
    obs.bounds.x2 += dx; obs.bounds.y2 += dy;
    
    for (auto& seg : obs.segments) {
        seg.x1 += dx; seg.y1 += dy;
        seg.x2 += dx; seg.y2 += dy;
    }
    rebuildSegments();
}

float SimWorld::pointToSegmentDist(float px, float py, float x1, float y1, float x2, float y2) const {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float lenSq = dx * dx + dy * dy;
    if (lenSq == 0) return std::hypot(px - x1, py - y1);

    float t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
    t = std::max(0.0f, std::min(1.0f, t));

    float nearX = x1 + t * dx;
    float nearY = y1 + t * dy;
    return std::hypot(px - nearX, py - nearY);
}

bool SimWorld::checkCollision(float x, float y, float radius, float theta) const {
    float hw = ROBOT_HALF_WIDTH;
    float hl = ROBOT_HALF_LENGTH;
    float cosT = std::cos(theta);
    float sinT = std::sin(theta);

    struct Point { float lx, ly; };
    Point checkPoints[] = {
        {-hw, hl}, {hw, hl}, {hw, -hl}, {-hw, -hl},
        {0, hl}, {0, -hl}, {-hw, 0}, {hw, 0}
    };

    for (const auto& pt : checkPoints) {
        float wx = x + pt.lx * cosT - pt.ly * sinT;
        float wy = y + pt.lx * sinT + pt.ly * cosT;

        for (const auto& seg : allSegments) {
            float dist = pointToSegmentDist(wx, wy, seg.x1, seg.y1, seg.x2, seg.y2);
            if (dist < 0.02f) return true; // 2cm tolerance
        }

        if (wx < 0 || wx > WAREHOUSE_WIDTH || wy < 0 || wy > WAREHOUSE_HEIGHT) {
            return true;
        }
    }
    return false;
}
