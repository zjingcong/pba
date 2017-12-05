
import sys
import exceptions

sys.path.append('/home/jingcoz/workspace/pba/final')

import pba.swig.PbaHou as pbah

node = hou.pwd()    # this node
inputs = node.inputs()  # inputs data
geo = node.geometry()   # incoming geometry

'''============================ SETUP ============================'''

# get input data
pba = inputs[1].cachedUserData('pba')
config_parms = inputs[1].cachedUserData('config_parms')
reseed_geo = inputs[2].geometry()
dens = inputs[3].cachedUserData('dens')
vel_x = inputs[3].cachedUserData('vel_x')
vel_y = inputs[3].cachedUserData('vel_y')
vel_z = inputs[3].cachedUserData('vel_z')
# set node user data
node.setCachedUserData('hou', pba)

frame_id = int(hou.frame())
start_frame = config_parms['start_sim']
advect_start_frame = config_parms['start_advect']
advect_scale = config_parms['advect_scale']
life = config_parms['life']
life_var = config_parms['life_var']

'''==============================================================='''


# init dynamical state
def create_init_ds():
    init_num = len(geo.points())
    pba.add_DS(init_num, life, life_var)
    i = 0
    for point in geo.points():
        pos = point.position()
        # create dynamic state
        pba.set_pos(i, pos[0], pos[1], pos[2])
        pba.set_vel(i, 0, 0, 0)
        i = i + 1


# generate particles from third input
def create_particles(reseed_geo):
    print " - current points num: ", len(geo.points())
    # get third input geom
    nb = pba.get_nb()
    num = len(reseed_geo.points())
    print " - reseed num: ", num
    pba.add_DS(num, life, life_var)
    for point in reseed_geo.points():
        pos = point.position()
        # create dynamic state
        pba.set_pos(nb, pos[0], pos[1], pos[2])
        pba.set_vel(nb, 0, 0, 0)
        # create point
        p = geo.createPoint()
        p.setPosition(pos)
        nb = nb + 1


# update houdini points after solving pba dynamical state
def update_points():
    points = geo.points()
    particles_num = pba.get_nb()
    num = particles_num
    if particles_num > len(points):
        num = len(points)
    for i in xrange(num):
        new_pos_tmp = pba.get_pos(i)
        new_vel_tmp = pba.get_vel(i)
        new_pos = pbah.doubleArray_frompointer(new_pos_tmp)
        new_vel = pbah.doubleArray_frompointer(new_vel_tmp)
        life_value = pba.get_life(i)
        age_value = pba.get_age(i)
        pscale = pba.get_pscale(i)
        dead = pba.get_dead(i)
        # set attrib to houdini points, match the attrib type between houdini and pba
        points[i].setPosition((new_pos[0], new_pos[1], new_pos[2]))
        points[i].setAttribValue("vel", (new_vel[0], new_vel[1], new_vel[2]))
        points[i].setAttribValue("life", life_value)
        points[i].setAttribValue("age", age_value)
        points[i].setAttribValue("pscale", pscale)
        points[i].setAttribValue("dead", dead)


# delete dead points
def delete_dead():
    points = geo.points()
    dead_points = [p for p in points if p.attribValue("dead") == 1]
    print " - current points num: ", len(points), "delete dead points: ", len(dead_points), "..."
    geo.deletePoints(dead_points)
    print " - delete success, rest points num: ", len(geo.points())


# get vel
def get_vel_from_volume(v_x, v_y, v_z, pos):
    x = v_x.sample(pos)
    y = v_y.sample(pos)
    z = v_z.sample(pos)
    vel = (x, y, z)

    return vel


# get dens
def get_dens_from_volume(dens, pos):
    density = dens.sample(pos)
    return density


# ramp
def clamp(value, min, max):
    if value < min:
        return min
    if value > max:
        return max
    return value


# advect points by vel field
def advect_by_volume(v_x, v_y, v_z, scale):
    particles_num = pba.get_nb()
    for i in xrange(particles_num):
        pos_tmp = pba.get_pos(i)
        pos = pbah.doubleArray_frompointer(pos_tmp)
        position = (pos[0], pos[1], pos[2])
        vel_from_volume = get_vel_from_volume(v_x, v_y, v_z, position)
        dens_from_volume = get_dens_from_volume(dens, position)

        factor = 1.0
        # factor = scale * clamp(dens_from_volume, 0.0, 1.0)
        vel = tuple(map(lambda x: x * factor, vel_from_volume))
        pba.set_vel(i, vel[0], vel[1], vel[2])


'''============================ RESET ============================'''

if hou.frame() <= start_frame:
    pba.reset()
    print "reset pba dynamical state: ", pba.get_nb()

'''==============================================================='''

'''============================ INIT ============================'''

if hou.frame() == start_frame:
    print "create init ds"
    color_attrib = geo.addAttrib(hou.attribType.Point, "Cd", (1.0, 1.0, 1.0))
    vel_attrib = geo.addAttrib(hou.attribType.Point, "vel", (0.0, 10.0, 0.0))
    # match the attrib type!!!
    life_attrib = geo.addAttrib(hou.attribType.Point, "life", 1000.0)   # float
    age_attrib = geo.addAttrib(hou.attribType.Point, "age", -1.0)   # float
    dead_attrib = geo.addAttrib(hou.attribType.Point, "dead", 0)    # int
    pscale_attrib = geo.addAttrib(hou.attribType.Point, "pscale", 1.0)  # float
    create_init_ds()

'''==============================================================='''

'''============================= SIM ============================='''

print "-" * 10, "SIM FRAME", frame_id, "-" * 10
# add particles
print "reseed src..."
create_particles(reseed_geo)
# advect
if frame_id > advect_start_frame:
    print "advect by volume..."
    advect_by_volume(vel_x, vel_y, vel_z, advect_scale)
# solve
print "solve using leap frog..."
pba.solve()
# draw
print "update houdini geom points..."
update_points()
# clean
print "delete houdini dead points..."
delete_dead()
print "clean pba dynamical state..."
pba.clean()

'''==============================================================='''
