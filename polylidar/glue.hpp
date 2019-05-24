#include "polylidar.hpp"
#include "delaunator.hpp"

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include <pybind11/stl_bind.h> // Pybind11 stl bindings
#include "pybind11/numpy.h"


namespace polylidar {

    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> extractPlanesAndPolygons(py::array_t<double> nparray,
                                                                                                                        double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                                                                                        double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                                                                                        double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        Config config{0, alpha, xyThresh, lmax, minTriangles, minBboxArea, zThresh, normThresh, allowedClass};
        auto info = nparray.request();
        auto size = static_cast<size_t>(info.shape[0] * info.shape[1]);
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        mdarray points = xt::adapt((double*)info.ptr, size, xt::no_ownership(), shape);
        return _extractPlanesAndPolygons(points, config);
    }


    std::vector<Polygon> extractPolygons(py::array_t<double> nparray,
                                        double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                        double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                        double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        Config config{0, alpha, xyThresh, lmax, minTriangles, minBboxArea, zThresh, normThresh, allowedClass};

        auto info = nparray.request();
        // std::cout << info.shape << info.strides << std::endl;
        auto size = static_cast<size_t>(info.shape[0] * info.shape[1]);
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        mdarray points = xt::adapt((double*)info.ptr, size, xt::no_ownership(), shape);

        return _extractPolygons(points, config);
    }

    std::tuple<std::vector<Polygon>, std::vector<float>> extractPolygonsAndTimings(py::array_t<double> nparray,
                                        double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                        double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                        double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        Config config{0, alpha, xyThresh, lmax, minTriangles, minBboxArea, zThresh, normThresh, allowedClass};
        std::vector<float> timings;

        auto info = nparray.request();
        auto size = static_cast<size_t>(info.shape[0] * info.shape[1]);
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        mdarray points = xt::adapt((double*)info.ptr, size, xt::no_ownership(), shape);
        auto polygons = _extractPolygonsAndTimings(points, config, timings);
        
        return std::make_tuple(polygons, timings);
    }



}