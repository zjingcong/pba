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

unit_test = things.unitTest()
viewer.AddThing(unit_test)

viewer.Init(args)
viewer.MainLoop()
