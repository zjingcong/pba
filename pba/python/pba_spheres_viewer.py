#!/usr/bin/python


import os
import sys
import argparse

sys.path.append('../../')

import pba.swig.PbaViewer as pbav
import pba.swig.PbaThings as things

########## main ##########

viewer = pbav.CreateViewer()
args = pbav.StringArray()

for s in sys.argv:
	args.push_back(s)

spheres = things.Spheres()
viewer.AddThing(spheres)

viewer.Init(args)
viewer.MainLoop()
