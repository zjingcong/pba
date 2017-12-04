node = hou.pwd()
geo = node.geometry()

vel_x = None
vel_y = None
vel_z = None

# get vdb grid
dens = geo.prims()[0]
vel_x = geo.prims()[1]
vel_y = geo.prims()[2]
vel_z = geo.prims()[3]

node.setCachedUserData('dens', dens)
node.setCachedUserData('vel_x', vel_x)
node.setCachedUserData('vel_y', vel_y)
node.setCachedUserData('vel_z', vel_z)
