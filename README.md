# Spheres Collisions
>Jingcong Zhang (jingcoz@g.clemson.edu)


>2017-11-26

This program assembles a sphere dynamic system, implements sphere-sphere collision and sphere-triangle mesh collision.

### Build
To build this code:
```sh
    cd pba
    make clean
    make
    make stuff
    make viewer
```

### Usage
To run this code: 
```sh
    cd pba/python
    pba_cloth_viewer.py <sphere_num>
```

**Command line usage:** 
  - Basic viewer usage
  - PbaThing usage: 
  ```
    c/C          reduce/increase Cr
    s/S          reduce/increase Cs
    g/G          reduce/increase gravity constant
    e            add spheres
    a            turn on/off kdtree
    q            switch solver between Leap Frog Solver and Sixth Order Solver
    t/T          reduce/increase timestep
    space        pause simulation
    Esc          quit
  ```
