#!/usr/bin/python


import os
import sys
import argparse

sys.path.append('../../')

import pba.swig.PbaViewer as pbav
import pba.swig.PbaThings as things

def get_argvs():
	parser = argparse.ArgumentParser(description="Cloth simulation presets.")
	parser.add_argument('-p', '--preset', type=int, dest='preset', default=3,
						help='Input preset id. \nTo reach the goal, preset two cloth attributes: \n0 - only soft edges; \n1 - soft edges and soft triangles.')
	parser.add_argument('-s', '--setting', type=str, dest='setting', default='p',
						help='Input cloth setting: \na-add soft triangles; p-add soft edges.')
	args = parser.parse_args()
	return args

########## main ##########

cloth_args = []
cloth_args += [sys.argv[0]]
argvs = get_argvs()
cloth_args += [str(argvs.preset)]
if argvs.preset == 0:
	print "Preset 0"
	cloth_args += ['p']
elif argvs.preset == 1:
	print "Preset 1"
	cloth_args += ['a', 'p']
else:
	cloth_args += list(argvs.setting)

print cloth_args

viewer = pbav.CreateViewer()
args = pbav.StringArray()

for s in cloth_args:
	args.push_back(s)

cloth = things.ClothInHole()
viewer.AddThing(cloth)

viewer.Init(args)
viewer.MainLoop()
