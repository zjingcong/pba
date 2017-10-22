# Rigid Body Dynamics
>Jingcong Zhang (jingcoz@g.clemson.edu)
>2017-10-22

This program assembles a rigid body dynamics system in which the rigid body is initialized to points on the surface of specified geometry.
- This program is able to read an obj file and initialize the particle positions at the vertices of the obj file.
- This program contains the RBD object within a cube that the RBD object collides with.Rigid body model will automatically be scaled to fit in the cube.
- The RBD object starts near (0,0,0) with an initial center of mass velocity and angular velocity.
- Keyboard controls.
- This system uses Leap Frog Solver with sub steps.

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
    ./pba_rbd_viewer.py <obj_path>
```

  <obj_path> is the file path to the .obj rigid body model, 
  the default scene is box.obj in models folder.
  
  

**Command line usage:** 
  - Basic viewer usage
  - PbaThing usage: 
  ```
    v/V     reduce/increase center of mass velocity magnitude
    w/W     reduce/increase angular velocity magnitude
    t/T     reduce/increase time step
    g/G     reduce/increase gravity constant
    l       switch between wireframe and normal display mode
    1-9     specify sub step
    Esc     quit
  ```
