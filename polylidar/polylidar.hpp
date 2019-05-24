
#ifndef POLYLIDAR
#define POLYLIDAR
#define _USE_MATH_DEFINES
#define VERSION_INFO "0.0.3"

#include <array>
#include <ostream>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>

#include "helper.hpp"
#include "delaunator.hpp"


#define DEFAULT_DIM 2
#define DEFAULT_ALPHA 1.0
#define DEFAULT_XYTHRESH 0.0
#define DEFAULT_LMAX 0.0
#define DEFAULT_MINTRIANGLES 20
#define DEFAULT_MINBBOX 100.0
#define DEFAULT_ZTHRESH 0.20
#define DEFAULT_NORMTHRESH 0.90
#define DEFAULT_ALLOWEDCLASS 4.0

#define DEBUG 1



namespace py = pybind11;

namespace polylidar {

    using vvi = std::vector<std::vector<size_t>>;
    struct Config
    {
        // 2D base configuration
        int dim = DEFAULT_DIM;
        double alpha = DEFAULT_ALPHA;
        double xyThresh = DEFAULT_XYTHRESH;
        double lmax = DEFAULT_LMAX;
        size_t minTriangles = DEFAULT_MINTRIANGLES;
        double minBboxArea = DEFAULT_MINBBOX;
        // 3D configuration
        double zThresh = DEFAULT_ZTHRESH;
        double normThresh = DEFAULT_NORMTHRESH;
        // 4D configuration
        double allowedClass = DEFAULT_ALLOWEDCLASS;
        // Not configurable, yet...
        std::array<double, 3> desiredVector = std::array<double, 3>{0.0, 0.0, 1.0};
    };

    struct Polygon {
        std::vector<size_t> shell;
        std::vector<std::vector<size_t>> holes;
        // I know this looks crazy
        // but for some reason I need this to allow the polygon to have holes
        // without it I could access the holes, but only ONCE. Then they would be gone
        // i.e in python "polygon.holes" <- Now its dead the next time you acces it
        // So I think this now makes a copy on every access. but whatever
        vvi getHoles() const {return holes;}
        void setHoles(vvi x) {holes = x;}
    };


    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> _extractPlanesAndPolygons(mdarray nparray, Config config);
    std::vector<Polygon> _extractPolygons(polylidar::mdarray nparray, Config config);
    std::vector<Polygon> _extractPolygonsAndTimings(mdarray nparray, Config config, std::vector<float> &timings);


}


#endif
