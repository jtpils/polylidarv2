from polylidar import extractPolygons
import numpy as np


points = np.array([[0.0,0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 2.0]])
print(points.dtype)

s = extractPolygons(points, lmax=3.0, minTriangles=0)
print(points)
print(s)
