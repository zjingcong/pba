
import sys
sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()    # this node
inputs = node.inputs()  # inputs data
geo = node.geometry()   # incoming geometry

'''============================ SETUP ============================'''

# get input data
houdini = inputs[1].cachedUserData('hou')
vel_field = inputs[3].cachedUserData('vel_field')
# set node user data
node.setCachedUserData('hou', houdini)

frame_id = int(hou.frame())
start_frame = 1

'''==============================================================='''


# init dynamical state
def create_init_ds():
    init_num = len(geo.points())
    houdini.add_DS(init_num)
    i = 0
    for point in geo.points():
        pos = point.position()
        # create dynamic state
        houdini.set_pos(i, pos[0], pos[1], pos[2])
        houdini.set_vel(i, 0, 0, 0)
        i = i + 1


# generate particles from third input
def create_particles():
    # get third input geom
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
        new_vel_tmp = houdini.get_vel(i)
        new_pos = pbah.doubleArray_frompointer(new_pos_tmp)
        new_vel = pbah.doubleArray_frompointer(new_vel_tmp)
        points[i].setPosition((new_pos[0], new_pos[1], new_pos[2]))
        points[i].setAttribValue("vel", (new_vel[0], new_vel[1], new_vel[2]))


'''============================ RESET ============================'''

if hou.frame() <= start_frame:
    houdini.reset()
    print "reset pba dynamical state: ", houdini.get_nb()

'''==============================================================='''

'''============================ INIT ============================'''

if hou.frame() == start_frame:
    print "create init ds"
    color_attrib = geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    vel_attrib = geo.addAttrib(hou.attribType.Point, "vel", (0.0, 10.0, 0.0))
    create_init_ds()

'''==============================================================='''

print "-" * 10, "SIM FRAME", frame_id, "-" * 10
# add particles
create_particles()
print "particles len: ", len(geo.points())
print "ds len: ", houdini.get_nb()
# solve
houdini.solve()
# draw
update_points()

