
import sys
sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()
geo = node.geometry()

################# init parms #################

g = 0.0
start_advect_frame = 50
advect_scale = 1.0

fps = hou.fps()
start_sim_frame = int(hou.chsop("../../../startframe"))

config_parms = dict(start_sim=start_sim_frame, start_advect=start_advect_frame, advect_scale=advect_scale)

################# init pba ###################
print "Init Pba Houdini..."
args = pbah.StringArray()

houdini = pbah.CreatePbaHouParticles()
houdini.init(args)
dt = 1.0 / fps
houdini.set_dt(dt)
houdini.set_gravity(g)
houdini.reset()

################# set usr datas ###################
print "Set all user datas..."
node.setCachedUserData('hou', houdini)
node.setCachedUserData('geo', geo)
node.setCachedUserData('config_parms', config_parms)

print "Init complete!"