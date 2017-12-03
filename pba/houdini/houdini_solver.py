
import sys
sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()    # this node
inputs = node.inputs()  # inputs data
geo = node.geometry()   # incoming geometry

houdini = inputs[1].cachedUserData('hou')
node.setCachedUserData('hou', houdini)

def create_init_ds():
    init_num = len(geo.points())
    houdini.add_DS(init_num)

# generate particles from third input
def create_particles():
    particles = inputs[2].geometry()
    nb = houdini.get_nb()
    num = len(particles.points())
    houdini.add_DS(num)
    for point in particles.points():
        pos = point.position()
        # create dynamic state
        houdini.set_pos(nb, pos[0], pos[1], pos[2])
        houdini.set_vel(nb, 0, 0, 0)
        # create point
        p = geo.createPoint()
        p.setPosition(pos)
        nb = nb + 1

# update houdini points after solving pba dynamical state
def update_points():
    points = geo.points()
    particles_num = houdini.get_nb()
    for i in xrange(particles_num):
        new_pos_tmp = houdini.get_pos(i)
        new_pos = pbah.doubleArray_frompointer(new_pos_tmp)
        points[i].setPosition((new_pos[0], new_pos[1], new_pos[2]))

start_frame = 1

if hou.frame() <= start_frame:
    houdini.reset()
    print "reset pba dynamical state: ", houdini.get_nb()

if hou.frame() == start_frame:
    print "create init ds"
    create_init_ds()

print "-" * 10, "SIM FRAME", hou.frame(), "-" * 10
# add particles
create_particles()
print "particles len: ", len(geo.points())
print "ds len: ", houdini.get_nb()
houdini.solve()
update_points()
