cmake_minimum_required(VERSION 3.6)
project(pba)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        base/DynamicalState.C
        base/LinearAlgebra.C
        base/Matrix.C
        base/PbaThing.C
        base/PbaViewer.C
        base/Solver.C
        base/Force.C
        include/Color.h
        include/DynamicalState.h
        include/LinearAlgebra.h
        include/Matrix.h
        include/PbaThing.h
        include/PbaViewer.h
        include/Vector.h
        include/Solver.h
        include/Force.h
        things/TriangleCollision.h
        include/Tools.h base/Tools.C include/Collision.h base/Collision.C include/Geometry.h base/Geometry.C include/BBox.h base/BBox.C include/KdTree.h base/KdTree.C include/Triangle.h base/Triangle.C things/BoidFlocking.h include/Boid.h base/Boid.C)

include_directories(include/)

add_executable(pba ${SOURCE_FILES})