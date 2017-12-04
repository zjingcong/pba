
import sys
sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()
geo = node.geometry()
fps = hou.fps()

################# init parms #################

g = 8.0

################# init pba ###################
args = pbah.StringArray()

houdini = pbah.CreatePbaHouParticles()
houdini.init(args)
dt = 1.0 / fps
houdini.set_dt(dt)
houdini.set_gravity(g)

node.setCachedUserData('hou', houdini)
node.setCachedUserData('geo', geo)

houdini.reset()

print "Init complete!"