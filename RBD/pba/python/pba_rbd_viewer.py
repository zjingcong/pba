#!/usr/bin/python


import os
import sys

sys.path.append('../../')

import pba.swig.PbaViewer as pbav
import pba.swig.PbaThings as things

if len(sys.argv) < 2:
    sys.argv += ['../../models/box.obj']  # default rigid body model

viewer = pbav.CreateViewer()

args = pbav.StringArray()


for s in sys.argv:
	args.push_back(s)

rbd = things.RBD()
viewer.AddThing(rbd)

viewer.Init(args)
viewer.MainLoop()
