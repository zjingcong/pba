
import sys
sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()
geo = node.geometry()

################# init parms #################

g = 0.0
start_advect_frame = 30
advect_scale = 1.0
life = 30.0
life_var = 5.0

fps = hou.fps()
start_sim_frame = int(hou.chsop("../../../startframe"))

config_parms = dict(start_sim=start_sim_frame,
                    start_advect=start_advect_frame,
                    advect_scale=advect_scale,
                    life=life,
                    life_var=life_var)

################# init pba ###################
print "Init Pba Houdini..."
args = pbah.StringArray()

pba = pbah.CreatePbaHouParticles()
pba.init(args)
dt = 1.0 / fps
pba.set_dt(dt)
pba.set_gravity(g)
pba.reset()

################# set usr datas ###################
print "Set all user datas..."
node.setCachedUserData('pba', pba)
node.setCachedUserData('geo', geo)
node.setCachedUserData('config_parms', config_parms)

print "Init complete!"
