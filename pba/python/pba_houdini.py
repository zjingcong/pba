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
print houdini.get_nb()
houdini.add_DS(1)
houdini.set_pos(0, 0, 0, 0)
houdini.set_vel(0, 0, 1, 0)
p = houdini.get_pos(0)
a = pbah.doubleArray_frompointer(p)
print a[0], a[1], a[2]

houdini.solve()

p = houdini.get_pos(0)
a = pbah.doubleArray_frompointer(p)
print a[0], a[1], a[2]
