# Physically Based Animation System
>Jingcong Zhang (zhjingcong@gmail.com)


>2017-11-21


This is a basic physically based animation system C++ library with Python API (binding with Swig). This program uses OpenGL as display program. It implements several dynamic simulation systems
including particles, boids, rigid body and cloth(soft body). This system is still under development, in the future, it may have features: adding constraints, 
providing user-friendly Python API which can be embeded into 3D commercial softwares like Maya or Houdini.

### Build
To build this code:
```sh
    cd pba
    make clean
    make
    make stuff
    make viewer
```

### Demo
For now, I have demo codes for particles collision, boids flocking, rigid body and cloth. Each one has own command line usage and keyboard controls.
All the simulation application codes are under pba/things. All the controlling python scripts are under pba/python.
