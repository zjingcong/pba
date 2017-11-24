#!/usr/bin/python


import os
import sys

sys.path.append('../../')

import pba.swig.PbaViewer as pbav
import pba.swig.PbaThings as things

viewer = pbav.CreateViewer()

args = pbav.StringArray()


for s in sys.argv:
	args.push_back(s)

triangle_collision = things.TriangleCollision()
viewer.AddThing(triangle_collision)

viewer.Init(args)
viewer.MainLoop()
