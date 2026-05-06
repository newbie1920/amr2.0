#pragma once

#include <vector>
#include <string>
#include <map>

// Kích thước kho xưởng
#define WAREHOUSE_WIDTH 10.0f
#define WAREHOUSE_HEIGHT 10.0f
#define ROBOT_RADIUS 0.22f
#define ROBOT_HALF_WIDTH 0.15f
#define ROBOT_HALF_LENGTH 0.15f

enum class SegmentTag {
    WALL,
    SHELF,
    OBSTACLE,
    CHARGER
};

struct Segment {
    float x1, y1, x2, y2;
    SegmentTag tag;
};

struct BoundingBox {
    float x1, y1, x2, y2;
};

struct DynamicObstacle {
    BoundingBox bounds;
    std::vector<Segment> segments;
};

struct SpawnPoint {
    float x;
    float y;
    float theta;
};

class SimWorld {
private:
    std::vector<Segment> staticSegments;
    std::vector<Segment> allSegments;
    std::map<std::string, DynamicObstacle> dynamicObstacles;
    SpawnPoint defaultSpawn;

    void buildStaticWorld();
    void rebuildSegments();
    float pointToSegmentDist(float px, float py, float x1, float y1, float x2, float y2) const;

public:
    SimWorld();
    
    void loadSlamTestMap();
    
    void addBoxObstacle(const std::string& id, float cx, float cy, float w, float h);
    void addCircleObstacle(const std::string& id, float cx, float cy, float radius, int sides = 8);
    void removeObstacle(const std::string& id);
    void moveObstacle(const std::string& id, float newCX, float newCY);

    bool checkCollision(float x, float y, float radius = ROBOT_RADIUS, float theta = 0.0f) const;

    const std::vector<Segment>& getSegments() const { return allSegments; }
    SpawnPoint getDefaultSpawn() const { return defaultSpawn; }
};
