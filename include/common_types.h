#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>

// Configuration structure to hold runtime parameters
struct RunConfig {
    double start_x = 70;
    double start_y = 60;
    double start_theta = 1.2;
    double goal_x = 180;
    double goal_y = 127;
    double goal_theta = 1.8;
    double ITER = 280;
    std::string solver_type = "cilqr";
    std::string selected_map = "B301";
    double obstacle_speed = 1.0;
    int obstacle_count = 4;
    double obstacle_distance = 3.5;
};

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Typedefs from ilqr.h
typedef Eigen::Vector4d State;
typedef Eigen::Vector2d Control;

// Point struct from ilqr.h
struct Point{
    Point(double X,double Y,double Heading){
        x = X;
        y = Y;
        heading = Heading;
    }
    bool operator==(const Point& other) const {
        return (x == other.x && y == other.y && heading == other.heading);
    }
    double x;
    double y;
    double heading;
};

// MapData struct from utils.h
struct MapData {
    int width;
    int height;
    double resolution;
    std::vector<double> origin;
    double max_elevation;
    std::vector<std::vector<double>> data;
};

// SemanticMap structures from utils.h
struct SemanticMapPoint {
    double x;
    double y;
    int type;
};

struct SemanticMapData {
    std::vector<SemanticMapPoint> points;
};

// OccupancyGrid struct from utils.h
struct OccupancyGrid {
    int width;
    int height;
    double resolution;
    double origin_x; // 地图原点x（米）
    double origin_y; // 地图原点y（米）
    std::vector<uint8_t> cells; // 0=free, 1=occupied
};
