import constants_module as cm
import pygame as pg

from scipy.linalg import norm
from numpy import matrix, block, eye, zeros, diag, sin, cos, tan, sqrt, arctan2, \
                  pi, inf, rad2deg, deg2rad
from pygame.math import Vector2

#--------------------------------------------------------------------------------#
#   AUSILIAR FUNCTIONS                                                           #
#--------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------#
#   rot_X(): the function output is the rotation matrix associated with a        #
#            counterclockwise rotation by a given angle around the x axis        #
#--------------------------------------------------------------------------------#
def rot_X(angle):
    
    c = cos(angle)
    s = sin(angle)
    R = matrix([[1, 0, 0], [0, c, -s], [0, s, c]])
    return R

#--------------------------------------------------------------------------------#
#   rot_Y(): the function output is the rotation matrix associated with a        #
#            counterclockwise rotation by a given angle around the y axis        #
#--------------------------------------------------------------------------------#
def rot_Y(angle):

    c = cos(angle)
    s = sin(angle)
    R = matrix([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return R

#--------------------------------------------------------------------------------#
#   rot_Z(): the function output is the rotation matrix associated with a        #
#            counterclockwise rotation by a given angle around the z axis        #
#--------------------------------------------------------------------------------#
def rot_Z(angle):

    c = cos(angle)
    s = sin(angle)
    R = matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R

#--------------------------------------------------------------------------------#
#   skew(): the function output is the skew matrix associated                    #
#           with the column vector "vect"                                        #
#--------------------------------------------------------------------------------#
def skew(vect):

    a = vect[0, 0]
    b = vect[1, 0]
    c = vect[2, 0]
    S = matrix([[0, -c, b], [c, 0, -a], [-b, a, 0]])
    return S

#--------------------------------------------------------------------------------#
#   wrap_angle(): input angle is mapped in the range [-pi, pi]                   #
#--------------------------------------------------------------------------------#
def wrap_angle(angle):

    while angle > pi:
        angle -= 2*pi
    while angle <= -pi:
        angle += 2*pi
    return angle

#--------------------------------------------------------------------------------#
#   jacobian_matrix(): the function output is the AUV jacobian matrix            #
#--------------------------------------------------------------------------------#
def jacobian_matrix(phi, theta, psi):

    # null matrix 3*3
    O = zeros([3, 3])

    # jacobian matrix diagonal blocks
    J_1 = rot_Z(psi) * rot_Y(theta) * rot_X(phi)
    J_2 = matrix([
        [1, sin(phi) * tan(theta), cos(phi) * tan(theta)],
        [0, cos(phi), -sin(phi)],
        [0, sin(phi) / cos(phi), cos(phi) / cos(theta)]])

    # assemble of jacobian matrix diagonal blocks
    J = block([[J_1, O], [O, J_2]])
    return J

#--------------------------------------------------------------------------------#
#   geo2ecef(): coordinate transformation from geodetic system to ECEF system    #
#--------------------------------------------------------------------------------#
def geo2ecef(pos_geo):

    lat = pos_geo[0]
    lon = pos_geo[1]
    alt = pos_geo[2]

    # conversion into radians
    lat = deg2rad(lat)
    lon = deg2rad(lon)

    # normal radius
    R_n = cm.R_E / sqrt(1 - (cm.ECC * sin(lat))**2)

    # coordinate transformation
    x_ecef = (R_n + alt) * cos(lat) * cos(lon)
    y_ecef = (R_n + alt) * cos(lat) * sin(lon)
    z_ecef = (R_n * (1 - cm.ECC**2) + alt) * sin(lat)

    pos_ecef = [x_ecef, y_ecef, z_ecef]
    return pos_ecef

#--------------------------------------------------------------------------------#
#   ecef2geo(): coordinate transformation from ECEF system to geodetic system    #
#--------------------------------------------------------------------------------#
def ecef2geo(pos_ecef):

    x_ecef = pos_ecef[0]
    y_ecef = pos_ecef[1]
    z_ecef = pos_ecef[2]

    lon = arctan2(y_ecef, x_ecef)
    rho = norm([x_ecef, y_ecef])

    lat_old = arctan2(z_ecef, (1 - cm.ECC**2) * rho)
    lat_err = inf

    # compute normal radius (R_n), altitude e latitude with an iterative process
    while abs(lat_err) > 1e-12:
        R_n = cm.R_E / sqrt(1 - (cm.ECC * sin(lat_old))**2)
        alt = rho / cos(lat_old) - R_n
        coeff = R_n / (R_n + alt)
        lat = arctan2(z_ecef, (1 - cm.ECC**2 * coeff) * rho)

        lat_err = lat - lat_old
        lat_old = lat

    # conversion into degrees
    lat = rad2deg(lat)
    lon = rad2deg(lon)

    pos_geo = [lat, lon, alt]
    return pos_geo

#--------------------------------------------------------------------------------#
#   ecef2ned(): coordinate transformation from ECEF system to NED system         #
#--------------------------------------------------------------------------------#
def ecef2ned(pos_ecef):

    # NED origin position referred to the ECEF system
    pos_0_geo  = [cm.LAT_0, cm.LON_0, cm.ALT_0]
    pos_0_ecef = geo2ecef(pos_0_geo)

    # rotations bringing NED system onto ECEF system
    R1 = rot_Z( cm.LON_0)
    R2 = rot_Y(-cm.LAT_0 + pi/2)
    R  = R2 * R1

    # conversion from lists to matrices, due to matrices computational reasons
    pos_0_ecef = matrix(pos_0_ecef).T
    pos_ecef   = matrix(pos_ecef).T

    # coordinate transformation
    pos_ned = R * (pos_ecef - pos_0_ecef)

    # convert pos_ned into a flatten list
    pos_ned = pos_ned.tolist()
    pos_ned = [value for sublist in pos_ned for value in sublist]
    return pos_ned

#--------------------------------------------------------------------------------#
#   ned2ecef(): coordinate transformation from NED system to ECEF system         #
#--------------------------------------------------------------------------------#
def ned2ecef(pos_ned):

    # NED origin position referred to ECEF system
    pos_0_geo = [cm.LAT_0, cm.LON_0, cm.ALT_0]
    pos_0_ecef = geo2ecef(pos_0_geo)

    # rotations bringing ECEF system onto NED system
    R1 = rot_Z(-cm.LON_0)
    R2 = rot_Y( cm.LAT_0 - pi/2)
    R  = R1 * R2

    # conversion from lists to matrices, due to matrices computational reasons
    pos_0_ecef = matrix(pos_0_ecef).T
    pos_ned    = matrix(pos_ned).T

    # coordinate transformation
    pos_ecef = R * pos_ned + pos_0_ecef

    # convert pos_ecef into a flatten list
    pos_ecef = pos_ecef.tolist()
    pos_ecef = [value for sublist in pos_ecef for value in sublist]
    return pos_ecef

#--------------------------------------------------------------------------------#
#   geo2ned(): coordinate transformation from geodetic system to NED system      #
#--------------------------------------------------------------------------------#
def geo2ned(pos_geo):

    pos_ecef = geo2ecef(pos_geo)
    pos_ned  = ecef2ned(pos_ecef)
    return pos_ned

#--------------------------------------------------------------------------------#
#   ned2geo(): coordinate transformation from NED system to geodetic system      #
#--------------------------------------------------------------------------------#
def ned2geo(pos_ned):

    pos_ecef = ned2ecef(pos_ned)
    pos_geo  = ecef2geo(pos_ecef)
    return pos_geo

#--------------------------------------------------------------------------------#
#   topleftcorner2center(): coordinate transformation from a reference system    #
#                           placed on the top left corner of the image(with axes #
#                           aligned with the image sides) to a reference system  #
#                           placed on the center of the image (keeping the same  #
#                           orientation of the previous reference system)        #
#--------------------------------------------------------------------------------#
def topleftcorner2center(coord_topleft, dim_image):
    # coord_topleft = 2D-coordinates of the point on the image plane with respect
    #                 to the reference system placed on the top left corner 
    #                 of the image
    # dim_image     = list containing image width and height
 
    w = dim_image[0]    # image width
    h = dim_image[1]    # image height

    # point coordinates on the centred reference system
    u = coord_topleft[0] - w//2
    v = coord_topleft[1] - h//2

    coord_center = [u, v]
    return coord_center

#--------------------------------------------------------------------------------#
#   image_coord2angles(): computing spherical coordinates angles of the point on #
#                         the image plane, referred to the camera system.        #
#                         2D-coordinates of the point, vertical angle of view    #
#                         and image height are the function inputs.              #
#                         The outputs are the angles                             #
#--------------------------------------------------------------------------------#
def image_coord2angles(coord_center, AOV_vert, h_image):
    # coord_center = 2D-coordinates of the point on the image plane referred
    #                to the reference system placed on the center of the image
    # AOV_vert     = vertical angle of view
    # h_image      = image height

    # vertical angle of view  
    AOV_vert = deg2rad(AOV_vert) 

    # compute distance between the camera centre and the image plane
    f = 1/tan(AOV_vert/2) * h_image/2

    # angles of the spherical reference system centred on camera 
    alpha = arctan2(coord_center[0], f)
    beta  = arctan2(coord_center[1], f)

    angles = [alpha, beta]
    return angles

#--------------------------------------------------------------------------------#
#   spherical2cartesian(): tranformation from spherical coordinate system to     #
#                          cartesian coordinate system                           #
#--------------------------------------------------------------------------------#  
def spherical2cartesian(coord_spher):

    # spherical coordinates
    R     = coord_spher[0]    # point distance from origin
    alpha = coord_spher[1]    # longitude angle
    beta  = coord_spher[2]    # latitude angle

    # cartesian coordinates
    x = R * cos(beta) * cos(alpha)
    y = R * cos(beta) * sin(alpha)  
    z = R * sin(beta)

    coord_cart = [x, y, z]
    return coord_cart

#--------------------------------------------------------------------------------#
#   cam2body(): coordinate transformation from camera system to body system      #
#--------------------------------------------------------------------------------#
def cam2body(coord_cam):

    # convert coord_cam into a column vector
    coord_cam = matrix(coord_cam).T

    # rotation and traslation bringing body system onto camera system
    R = eye(3)
    d = matrix([0.5, 0, 0]).T

    # coordinate transformation
    coord_body = R * coord_cam + d

    # convert coord_body into a flatten list
    coord_body = coord_body.tolist()
    coord_body = [value for sublist in coord_body for value in sublist]
    return coord_body

#--------------------------------------------------------------------------------#
#   body2ned(): coordinate transformation from body system to NED system         #
#--------------------------------------------------------------------------------#
def body2ned(coord_body, pos_AUV_ned, euler_angles_AUV):

    phi   = euler_angles_AUV[0]
    theta = euler_angles_AUV[1]
    psi   = euler_angles_AUV[2]

    # convert coord_body into a column vector
    coord_body = matrix(coord_body).T

    # rotation and traslation bringing NED system on body system
    R = rot_Z(psi) * rot_Y(theta) * rot_X(phi)
    d = matrix(pos_AUV_ned).T

    # coordinate transformation
    coord_ned = R * coord_body + d

    # convert coord_ned into a flatten list
    coord_ned = coord_ned.tolist()
    coord_ned = [value for sublist in coord_ned for value in sublist]
    return coord_ned

#--------------------------------------------------------------------------------#
#   compute_virtual_wp(): the function output is a virtual waypoint placed       #
#                         on the conjunction line between the camera and the     # 
#                         object of interest with a distance r from camera       #
#--------------------------------------------------------------------------------#
def compute_virtual_wp(pos_AUV, euler_angles_AUV, dim_image, AOV_vert, box, r):
    # dim_image = list containing image width and height
    # AOV_vert  = vertical angle of view
    # box       = list of two points (top-left corner and bottom-right corner)
    #             defining the bounding box

    # compute the bounding box center on a frame placed in the top left 
    # corner of the image and axes aligned with the image sides
    xc = (box[0] + box[2])/2
    yc = (box[1] + box[3])/2

    # bounding box center coordinates 
    coord_topleft = [xc, yc]

    # coordinate transformation from a frame placed in the top-left corner of 
    # the image to a frame placed in the center of the image, 
    # with the same orientation of the previous one
    coord_center = topleftcorner2center(coord_topleft, dim_image)

    # virtual waypoint coordinates in the camera reference system
    # (spherical coordinate system)
    angles = image_coord2angles(coord_center, AOV_vert, dim_image[1])
    coord_spherical = [r, angles[0], angles[1]]

    # virtual waypoint coordinates in the camera reference system 
    # (cartesian coordinate system)
    coord_cam = spherical2cartesian(coord_spherical)

    # virtual waypoint coordinates in the body reference system
    coord_body = cam2body(coord_cam)

    # virtual waypoint coordinates in the NED reference system
    coord_ned = body2ned(coord_body, pos_AUV, euler_angles_AUV)
    return coord_ned

#--------------------------------------------------------------------------------#
#   compute_aiming_angles(): computing pitch and yaw angles to turn              #
#                            the AUV fore towards the waypoint                   #
#--------------------------------------------------------------------------------#
def compute_aiming_angles(waypoint, pos_cur):

    # components of the vector joining the current position with the waypoint
    delta_x = waypoint[0] - pos_cur[0]
    delta_y = waypoint[1] - pos_cur[1]
    delta_z = waypoint[2] - pos_cur[2]
    
    # norm of the projection on the xy plane of the conjunction vector
    delta_norm = norm([delta_x, delta_y])   

    # desired euler angles
    psi_des   =  arctan2(delta_y, delta_x)
    theta_des = -arctan2(delta_z, delta_norm)

    angles_des = [0, theta_des, psi_des]   # default: phi_des = 0
    return angles_des 

#--------------------------------------------------------------------------------#
#   build_msg_UDP(): building the msg_UDP message                                #
#--------------------------------------------------------------------------------#
def build_msg_UDP(eta):

    msg_UDP = [
        eta[0, 0], 
        eta[1, 0], 
        eta[2, 0] * -1,
        rad2deg(eta[5, 0]),
        rad2deg(eta[4, 0]) * -1,
        rad2deg(eta[3, 0]) * -1,
        1, -1, 0, 0, 0, -1]

    return msg_UDP

#--------------------------------------------------------------------------------#
#   build_nav_status(): building the nav_status message                          #
#--------------------------------------------------------------------------------#
def build_nav_status(eta, ni, mission_mode):
    
    # Zeno geodetic coordinates
    pos_geo = ned2geo([eta[0, 0], eta[1, 0], eta[2, 0]])
    lat   = pos_geo[0] 
    lon   = pos_geo[1]
    depth = pos_geo[2]

    # euler angles
    phi   = eta[3, 0]   # roll
    theta = eta[4, 0]   # pitch
    psi   = eta[5, 0]   # yaw

    # euler parameters (unitary quaternion components)
    q0 = 0.
    q1 = 0.
    q2 = 0.
    q3 = 0.

    # twist
    tw0 = ni[0, 0]
    tw1 = ni[1, 0]
    tw2 = ni[2, 0]
    tw3 = ni[3, 0]
    tw4 = ni[4, 0]
    tw5 = ni[5, 0]

    # other parameters of interest
    drift       = mission_mode      # temporary!
    course      = 0.
    speed       = 0.
    altitude    = 0.
    time_stamp  = 0.
 
    # nav_status message
    nav_status = [
        lat, lon, depth, 
        phi, theta, psi, 
        q0, q1, q2, q3, 
        drift, 
        course, 
        speed, 
        altitude, 
        tw0, tw1, tw2, tw3, tw4, tw5, 
        time_stamp]
        
    return nav_status

#--------------------------------------------------------------------------------#
#  FUNCTIONS FOR THE DYNAMIC SIMULATION                                          #
#--------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------#
#   compute_dynamics(): implementing dynamic equations for the AUV               #
#--------------------------------------------------------------------------------#
def compute_dynamics(eta, ni, tau):
    # Note:
    # We are assuming a body frame centered at the robot's center of gravity.
    # Body frame axes are directed as the principal axes of inertia.
    # z axis of body frame is directed from the top surface to the bottom surface. 
    # the inertial system is NED (positive z for increasing dephts).

    # some state components useful to build matrices
    depth = eta[2, 0]       # depth from sea level (NED frame)
    phi   = eta[3, 0]       # roll angle
    theta = eta[4, 0]       # pitch angle
    psi   = eta[5, 0]       # yaw angle
    omega = ni[3:6, 0]      # angolar velocity vector (body frame)
    
    # null matrix and identity matrix 3*3
    O = zeros([3, 3])
    I = eye(3)
    
    # jacobian matrix
    J = jacobian_matrix(phi, theta, psi)

    # tensor of inertia about the center of gravity G and with respect 
    # to the principal axis frame
    In_G = matrix([[cm.IN_X, 0, 0], [0, cm.IN_Y, 0], [0, 0, cm.IN_Z]]) 

    # inertia matrix (block matrix)
    M = block([[cm.MASS * I, O], [O, In_G]])

    # centrifugal/Coriolis matrix
    C = block([[skew(cm.MASS * omega), O], [O, -skew(In_G * omega)]])

    # hydrodynamic resistance matrix (linear contribution)
    diag_elements = [cm.D1, cm.D2, cm.D3, cm.D4, cm.D5, cm.D6]
    D = diag(diag_elements)

    # force of gravity (f_g) and center of gravity position (r_g)
    f_g = (matrix([0, 0, cm.MASS * cm.GRV]) * J[0:3, 0:3]).T  
    r_g =  matrix([0, 0, 0]).T  # null due to the reference system centred on G

    # force of buoyancy (f_b) and center of buoyancy position (r_b)
    if depth > cm.MIN_DEPTH:
        f_b = (matrix([0, 0, -cm.RHO * cm.GRV * cm.VOL]) * J[0:3, 0:3]).T
    elif depth == cm.MIN_DEPTH:
        # if the robot is at zero depth, the force of buoyancy balances the force of gravity
        f_b = -f_g
    else:
        f_b = matrix([0, 0, 0]).T

    r_b =  matrix([0, 0, -cm.DIST_B]).T
    
    # gravitational contribution in terms of net force (R_tot) and resultant 
    # moment (M_tot) of the force of gravity and the force of buoyancy
    R_tot = f_g + f_b
    M_tot = skew(r_g) * f_g + skew(r_b) * f_b
    grv = -block([[R_tot], [M_tot]])     

    # the engine propellers at zero depth can no longer provide upward thrust
    if (depth == 0 and tau[2, 0] < 0) or depth < 0:
        tau[2, 0] = 0

    # lower and upper limit of depth
    # if  depth < cm.MIN_DEPTH:
    #     eta[2, 0] = cm.MIN_DEPTH
    #     ni[2, 0] = 0.0
    if depth > cm.MAX_DEPTH:
        eta[2, 0] = cm.MAX_DEPTH
        ni[2, 0] = 0

    # force and torque saturation
    force_norm = norm(tau[0:3, 0])
    force_max = cm.MAX_SPEED * cm.D1
    if force_norm > force_max:
        tau[0:3, 0] = force_max * tau[0:3, 0] / force_norm

    torque_norm = norm(tau[3:6, 0])
    torque_max = cm.MAX_SPIN * cm.D5
    if torque_norm > torque_max:
        tau[3:6, 0] = torque_max * tau[3:6, 0] / torque_norm

    # dynamic equations
    ni_dot = M.I * (tau - grv - (C + D) * ni)
    eta_dot = J * ni

    # Euler forward
    ni  +=  ni_dot * cm.TS
    eta += eta_dot * cm.TS

    # euler angles mapped on the interval [-pi, pi]
    for i in range(3, 6):
        eta[i, 0] = wrap_angle(eta[i, 0])

    return eta, ni

#--------------------------------------------------------------------------------#
#   compute_control(): implement control law                                     #
#--------------------------------------------------------------------------------#
def compute_control(eta, ni, eta_des, ni_des, ni_dot_des):
    
    # some state components useful to build matrices
    depth = eta[2, 0]       # depth from sea level (NED frame)
    phi   = eta[3, 0]       # roll angle
    theta = eta[4, 0]       # pitch angle
    psi   = eta[5, 0]       # yaw angle
    omega = ni[3:6, 0]      # angolar velocity vector (body frame)

    # null matrix and identity matrix 3*3
    O = zeros([3, 3])
    I = eye(3)

    # jacobian matrix
    J = jacobian_matrix(phi, theta, psi)

    # tensor of inertia about the center of gravity G and with respect 
    # to the principal axis frame
    In_G = matrix([[cm.IN_X, 0, 0], [0, cm.IN_Y, 0], [0, 0, cm.IN_Z]]) 

    # inertia matrix (block matrix)
    M = block([[cm.MASS * I, O], [O, In_G]])

    # centrifugal/Coriolis matrix
    C = block([[skew(cm.MASS * omega), O], [O, -skew(In_G * omega)]])

    # hydrodynamic resistance matrix (linear contribution)
    diag_elements = [cm.D1, cm.D2, cm.D3, cm.D4, cm.D5, cm.D6]
    D = diag(diag_elements)

    # force of gravity (f_g) and center of gravity position (r_g)
    f_g = (matrix([0, 0, cm.MASS * cm.GRV]) * J[0:3, 0:3]).T  
    r_g =  matrix([0, 0, 0]).T  # null because our reference system is centred on G

    # force of buoyancy (f_b) and center of buoyancy position (r_b)
    if depth > cm.MIN_DEPTH:
        f_b = (matrix([0, 0, -cm.RHO * cm.GRV * cm.VOL]) * J[0:3, 0:3]).T
    elif depth == cm.MIN_DEPTH:
        # if the robot is at zero depth, the force of buoyancy balances the force of gravity
        f_b = -f_g
    else:
        f_b = matrix([0, 0, 0]).T

    r_b =  matrix([0, 0, -cm.DIST_B]).T

    # gravitational contribution in terms of net force (R_tot) and net 
    # torque (M_tot) of the force of gravity and the force of buoyancy
    R_tot = f_g + f_b
    M_tot = skew(r_g) * f_g + skew(r_b) * f_b
    grv = -block([[R_tot], [M_tot]])

    # error calculation
    ni_err = ni_des - ni
    eta_err = eta_des - eta

    # eta_err with angles mapped on the interval [-pi, pi]
    for i in range(3, 6):
        eta_err[i, 0] = wrap_angle(eta_err[i, 0]) 

    # compute tau for control (computed torque)
    tau = M * (ni_dot_des + cm.KD * ni_err + cm.KP * J.I * eta_err) + (C + D) * ni + grv

    return tau

#--------------------------------------------------------------------------------#
#   GRAPHIC INTERFACE FUNCTIONS                                                  #
#--------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------#
#   blit_rotate(): rotating the image of a desired angle and print it on "surf"  #
#--------------------------------------------------------------------------------#
def blit_rotate(surf, image, pos, origin_pos, angle):

    # calculate the axis aligned bounding box of the rotated image
    w, h       = image.get_size()
    box        = [Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]
    box_rotate = [p.rotate(angle) for p in box]

    box_min    = (min(box_rotate, key = lambda p: p[0])[0],
                  min(box_rotate, key = lambda p: p[1])[1])

    box_max    = (max(box_rotate, key = lambda p: p[0])[0],
                  max(box_rotate, key = lambda p: p[1])[1])

    # calculate the translation of the pivot 
    pivot        = Vector2(origin_pos[0], -origin_pos[1])
    pivot_rotate = pivot.rotate(angle)
    pivot_move   = pivot_rotate - pivot

    # calculate the upper left origin of the rotated image
    origin = (pos[0] - origin_pos[0] + box_min[0] - pivot_move[0],
              pos[1] - origin_pos[1] - box_max[1] + pivot_move[1])

    # get a rotated image
    rotated_image = pg.transform.rotate(image, angle)

    # rotate and blit the image
    surf.blit(rotated_image, origin)

#--------------------------------------------------------------------------------#
#   arrow(): drawing an oriented segment                                         #
#--------------------------------------------------------------------------------#
def arrow(screen, lcolor, tricolor, start, end, triang_r):

    angle = arctan2(end[1] - start[1], end[0] - start[0])

    # pg.draw.aaline(screen, lcolor, start, 
    #     (end[0] + triang_r * cos(angle), end[1] + triang_r * sin(angle)))

    pg.draw.polygon(screen, tricolor, (
                    (end[0],
                     end[1]),
                    (end[0] + triang_r * (cos(angle + 3/4 * pi) - cos(angle)), 
                     end[1] + triang_r * (sin(angle + 3/4 * pi) - sin(angle))),
                    (end[0] + triang_r * (cos(angle - 3/4 * pi) - cos(angle)),
                     end[1] + triang_r * (sin(angle - 3/4 * pi) - sin(angle)))))

#--------------------------------------------------------------------------------#
#   create_view(): creating the image of an orthogonal view of the AUV           #
#--------------------------------------------------------------------------------#
def create_view(file_path, width):

    transparent_color = cm.PINK

    image = pg.Surface((width, width))
    image.fill(transparent_color)
    image.set_colorkey(transparent_color)

    # load image 
    zeno = pg.image.load(file_path)
    zeno.set_colorkey(transparent_color)
    zeno = pg.transform.scale(zeno, (width, width))

    image.blit(zeno, (0, 0))

    start = Vector2(width/2, width/2)
    if file_path == 'images/zeno_top.bmp':
        end   = Vector2(width/2, 0)
    else:
        end   = Vector2(width, width/2)
 
    # draw arrow
    line_color  = cm.WHITE
    arrow_color = cm.WHITE
    arrow_dim   = 6  
    arrow(image, line_color, arrow_color, start, end, arrow_dim)

    return image

#--------------------------------------------------------------------------------#
#   handle_keyboard_input(): managing keyboard commands                          #
#--------------------------------------------------------------------------------#
def handle_keyboard_input(events, tau):

    value = 1000.       # value > force_max and value > torque_max

    keys = [
        pg.K_w,     pg.K_s,             # keys W and S        act upon Fx
        pg.K_d,     pg.K_a,             # keys D and A        act upon Fy
        pg.K_e,     pg.K_q,             # keys E and Q        act upon Fz
        pg.K_RIGHT, pg.K_LEFT,          # keys RIGHT and LEFT act upon Mx
        pg.K_DOWN,  pg.K_UP,            # keys DOWN  and UP   act upon My
        pg.K_x,     pg.K_z]             # keys X and Z        act upon Mz

    for i in range(6):
        if tau[i, 0] > 0:
            tau[i, 0] =  value
        if tau[i, 0] < 0:
            tau[i, 0] = -value
    
    # update of tau components depending on the keys pressed
    even_numbers = [n for n in range(len(keys)) if n % 2 == 0]
    
    for event in events:          
        if event.type == pg.KEYDOWN:        # key pressed
            for i in even_numbers:
                j = int(i / 2)
                if event.key == keys[i]:
                    tau[j, 0] =  value
                if event.key == keys[i + 1]:
                    tau[j, 0] = -value

        if event.type == pg.KEYUP:          # key released
            for i in even_numbers:
                j = int(i / 2)
                if event.key == keys[i] and tau[j, 0] == value:
                    tau[j, 0] = 0
                if event.key == keys[i + 1] and tau[j, 0] == -value:
                    tau[j, 0] = 0
            
    return tau 

#--------------------------------------------------------------------------------#
#   draw_elements(): drawing all graphic elements                                #
#--------------------------------------------------------------------------------#
def draw_elements(screen, image_1, image_2, image_3, eta, ni, tau):

    # eta (angles are in degrees)
    angles_degree = [
        rad2deg(eta[3, 0]), 
        rad2deg(eta[4, 0]), 
        rad2deg(eta[5, 0])]

    eta_degree = matrix([eta[0, 0], eta[1, 0], eta[2, 0]] + angles_degree).T

    # recolor the screen
    screen.fill(cm.GREY)

    # compute dial centre position (for each dial)
    centre_2 = Vector2(cm.SCREEN_DIM[0]/2, cm.DIAL_SD)
    centre_1 = centre_2 - Vector2(2 * cm.DIAL_SD, 0)
    centre_3 = centre_2 + Vector2(2 * cm.DIAL_SD, 0)

    set_of_centres = [centre_1, centre_2, centre_3]

    # draw dial border (for each dial)
    dial_border_color = cm.LIGHT_GREY
    radius = cm.DIAL_R + cm.DIAL_BORDER_T

    for centre in set_of_centres:
        pg.draw.circle(screen, dial_border_color, centre, radius)

    # draw dial background (for each dial)
    dial_color = cm.DARK_BLUE
    radius = cm.DIAL_R

    for centre in set_of_centres:
        pg.draw.circle(screen, dial_color, centre, radius)

    # draw stylized vehicles according to trim angles (for each dial)
    width = cm.DIAL_R
    blit_rotate(screen, image_1, centre_1, (width, width), -angles_degree[2]) 
    blit_rotate(screen, image_2, centre_2, (width, width),  angles_degree[1])
    blit_rotate(screen, image_3, centre_3, (width, width), -angles_degree[0]) 
        # minus signs are due to the view: for the first one we look at the vehicle from 
        # above and the axis points downwards, for the third one we look at it from behind

    # draw the cross reference of the dial (for each dial)
    dial_line_color = cm.WHITE
    line_t = 1

    for centre in set_of_centres:
        start = centre - Vector2(cm.DIAL_R, 0)
        end   = centre + Vector2(cm.DIAL_R, 0)
        pg.draw.line(screen, dial_line_color, start, end, line_t)
        start = centre - Vector2(0, cm.DIAL_R)
        end   = centre + Vector2(0, cm.DIAL_R)
        pg.draw.line(screen, dial_line_color, start, end, line_t)

    # print eta, ni, tau components
    font = pg.font.SysFont('None', cm.DIM_FONT)

    strings = [
        [ 'x',  'y',  'z',  'φ',  'θ',  'ψ'],       # eta components
        [ 'u',  'v',  'w',  'p',  'q',  'r'],       # ni  components
        ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']]       # tau components

    vectors = [eta_degree, ni, tau]

    shift = 15
    for i in range(3):
        x_value = cm.X_VALUE + i * cm.DIST_X_VALUE
        if i == 2:
            shift = 25

        for j in range(6):
            y_value = cm.Y_VALUE + j * cm.DIST_Y_VALUE

            # print the name of the components of eta, ni, tau (elements of the list "strings")
            line = font.render(strings[i][j], True, cm.YELLOW)
            screen.blit(line, (x_value, y_value))

            # print numerical values of the components of eta, ni, tau
            string = '= {0: .3f}'.format(vectors[i][j, 0])
            line = font.render(string, True, cm.YELLOW)
            screen.blit(line, (x_value + shift, y_value))