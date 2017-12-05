#!/usr/bin/python


import os
import sys
import argparse

sys.path.append('../../')

import pba.swig.PbaHou as pbah

args = pbah.StringArray()

for s in sys.argv:
    args.push_back(s)

houdini = pbah.CreatePbaHouParticles()
houdini.init(args)
gamma = 0.333
freq = 10.3572
fjump = 2.0
octaves = 0.5
offset = 0.005
houdini.create_noise(gamma, freq, fjump, octaves, offset)

houdini.add_DS(1, 1000, 10)
houdini.set_pos(0, 10.0, 10, 100)
vel_tmp =  houdini.get_vel(0)
vel = pbah.doubleArray_frompointer(vel_tmp)
print vel[0], vel[1], vel[2]
houdini.advect_by_noise(1.0)
vel_tmp =  houdini.get_vel(0)
vel = pbah.doubleArray_frompointer(vel_tmp)
print vel[0], vel[1], vel[2]
