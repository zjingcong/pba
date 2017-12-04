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

vdb_file_path = '/DPA/wookie/dpa/projects/jingcoz/pba/hou_particles/houdini/hou/vdb/pba.test.9.vdb'
vdb_file = '/home/jingcoz/workspace/pba/final/vdb/pba.vdbfile15.merge.26.vdb'
# houdini.advect(vdb_file_path)
houdini.test(0, 0, 0, vdb_file_path)

# print houdini.get_nb()
# houdini.add_DS(1)
# houdini.set_pos(0, 0, 0, 0)
# houdini.set_vel(0, 0, 1, 0)
# p = houdini.get_pos(0)
# a = pbah.doubleArray_frompointer(p)
# print a[0], a[1], a[2]
#
# houdini.solve()
#
# p = houdini.get_pos(0)
# a = pbah.doubleArray_frompointer(p)
# print a[0], a[1], a[2]
