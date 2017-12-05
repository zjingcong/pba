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

houdini.add_DS(1, 1000, 10)
print houdini.get_life(0)
houdini.add_DS(1, 1000, 10)
print houdini.get_life(1)
houdini.add_DS(1, 1000, 10)
print houdini.get_life(2)
