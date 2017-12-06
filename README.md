# Pba-Houdini Particle Dynamics
>Jingcong Zhang (jingcoz@g.clemson.edu)


>2017-12-05

This program implements particle simulation. The particle simulation includes basic particle dynamics features related to velocity fields.
The particles can be advected by volume from vdb files or 3D noise fields built into the program. The entire program can be embedded into Houdini.

### Build
To build this code:
```sh
    cd pba
    make clean
    make
    make base
    make hou
```

### Usage
To test Python APIs: 
```sh
    cd pba/python
    pba_houdini_test.py
```
Houdini nodes are under pba/houdini.
