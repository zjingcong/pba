
import sys
sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()
geo = node.geometry()
fps = hou.fps()

################# init parms #################


################# init pba ###################
args = pbah.StringArray()

houdini = pbah.CreatePbaHouParticles()
houdini.init(args)
dt = 1.0 / fps
houdini.set_dt(dt)
houdini.set_gravity(9.8)

node.setCachedUserData('hou', houdini)

houdini.reset()

print "Init complete!"