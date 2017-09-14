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
        swig/PbaThings.i
        swig/PbaThings_wrap.cxx
        swig/PbaViewer.i
        swig/PbaViewer_wrap.cxx
        things/RotatingCube.h
        things/TriangleCollision.h)

include_directories(include/)

add_executable(pba ${SOURCE_FILES})