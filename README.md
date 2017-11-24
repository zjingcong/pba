# Cloth Dynamics
>Jingcong Zhang (jingcoz@g.clemson.edu)


>2017-11-07

This program assembles a soft body dynamics system, which typically has the setting for cloth.

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
    pba_cloth_viewer.py [-h] [-p PRESET] [-s SETTING]
```

  optional arguments:
  ```sh
  -h, --help            show this help message and exit
  -p PRESET, --preset PRESET
                        Input preset id.
                        To reach the goal 
                        (to make the cloth falls through the smallest hole in a plane in the shortest time), 
                        I have two cloth presets attributes: 
                            0 - only soft edges; 
                            1 - soft edges and soft triangles.
  -s SETTING, --setting SETTING
                        Input cloth setting: 
                            a-add soft triangles; 
                            p-add soft edges.
  ```

**Command line usage:** 
  - Basic viewer usage
  - PbaThing usage: 
  ```
    s/S          reduce/increase Ks
    k/K          reduce/increase Kf
    z/Z          reduce/increase As
    v/V          reduce/increase Af
    c/C          reduce/increase Cr
    x/X          reduce/increase Cs
    g/G          reduce/increase gravity constant
    a            turn on/off kdtree
    1-9          specify sub step
    Esc     quit
  ```
