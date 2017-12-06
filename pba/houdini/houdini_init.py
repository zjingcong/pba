
import sys
sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()
geo = node.geometry()

################# init parms #################

# force cfg
g = 0.0
# sim cfg
start_advect_frame = 30
advect_scale = 1.0
start_noise_frame = 10
life = 200.0
life_var = 20.0
pscale = 1.0
# noise cfg
gamma = 0.333
freq = 2.3572
fjump = 2.0
octaves = 0.5
offset = 0.005
# houdini cfg
fps = hou.fps()
start_sim_frame = int(hou.chsop("../../../startframe"))

config_parms = dict(start_sim=start_sim_frame,
                    start_advect=start_advect_frame,
                    start_noise=start_noise_frame,
                    advect_scale=advect_scale,
                    life=life,
                    life_var=life_var,
                    pscale=pscale)

################# init pba ###################
print "Init Pba Houdini..."
args = pbah.StringArray()

pba = pbah.CreatePbaHouParticles()
pba.init(args)
dt = 1.0 / fps
pba.set_dt(dt)
pba.set_gravity(g)
pba.set_noise_force_scale(0.0)
pba.create_noise(gamma, freq, fjump, octaves, offset)
pba.reset()

################# set usr datas ###################
print "Set all user datas..."
node.setCachedUserData('pba', pba)
node.setCachedUserData('geo', geo)
node.setCachedUserData('config_parms', config_parms)

print "Init complete!"
