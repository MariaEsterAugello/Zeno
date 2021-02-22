import threading

from time import sleep, time
from numpy import matrix, deg2rad, rad2deg, arctan2, sign, inf, pi
from scipy.linalg import norm
from serial_connection import SerialConnection
from object_detection import ObjectDetection
from functions_module import ned2geo, geo2ned, body2ned, compute_aiming_angles,  \
                             compute_virtual_wp
from constants_module import TS, MAX_PITCH, DIM_IMAGE, AOW_VERT

#--------------------------------------------------------------------------------#
#   FUNCTIONS                                                                    #
#--------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------#
#   get_mission_mode(): reading the value of the variable mission_mode           #
#--------------------------------------------------------------------------------#
def get_mission_mode():

    # read shared variable "nav_status" secured by lock
    nav_status_lock.acquire()
    mission_mode = nav_status[10]
    nav_status_lock.release()

    return mission_mode

#--------------------------------------------------------------------------------#
#   get_pos(): reading the first three components of the variable nav_status     #
#--------------------------------------------------------------------------------#
def get_pos():

    # wait for the mission activation
    # (if manual control mode is active)
    while not mission_mode:
        sleep(TS)

    # read shared variable "nav_status" secured by lock
    nav_status_lock.acquire()
    pos = geo2ned(nav_status[0:3])
    nav_status_lock.release()

    return pos

#--------------------------------------------------------------------------------#
#   get_euler_angles(): reading the third, fourth and fifth component            #
#                       of  the variable nav_status                              #
#--------------------------------------------------------------------------------#
def get_euler_angles():

    # wait for the mission activation
    # (if manual control mode is active)
    while not mission_mode:
        sleep(TS)

    # read shared variable "nav_status" secured by lock
    nav_status_lock.acquire()
    euler_angles = nav_status[3:6]
    nav_status_lock.release()

    return euler_angles

#--------------------------------------------------------------------------------#
#   get_pos_des(): reading the first three components of the variable command    #
#--------------------------------------------------------------------------------#
def get_pos_des():
    
    # wait for the mission activation
    # (if manual control mode is active)
    while not mission_mode:
        sleep(TS)
        
    # read shared variable "command" secured by lock
    command_lock.acquire()
    pos_des = geo2ned(command[0:3])
    command_lock.release()

    return pos_des

#--------------------------------------------------------------------------------#
#   get_euler_angles_des(): reading the last three components of the             #
#                           variable command                                     #
#--------------------------------------------------------------------------------#
def get_euler_angles_des():
    
    # wait for the mission activation
    # (if manual control mode is active)
    while not mission_mode:
        sleep(TS)

    # read shared variable "command" secured by lock
    command_lock.acquire()
    euler_angles_des = command[3:6]
    command_lock.release()

    return euler_angles_des

#--------------------------------------------------------------------------------#
#   set_pos_des(): writing on the first three components of the variable command #
#--------------------------------------------------------------------------------#
def set_pos_des(pos_des):
    global command
    
    # wait for the mission activation
    # (if manual control mode is active)
    while not mission_mode:
        sleep(TS)

    # write on shared variable "command" secured by lock
    command_lock.acquire()
    command[0:3] = ned2geo(pos_des)
    command_lock.release()

#--------------------------------------------------------------------------------#
#   set_euler_angles_des(): writing on the last three components of              #
#                           the variable command                                 #
#--------------------------------------------------------------------------------#
def set_euler_angles_des(euler_angles_des):
    global command

    # wait for the mission activation
    # (if manual control mode is active)
    while not mission_mode:
        sleep(TS)

    # write on shared variable "command" secured by lock
    command_lock.acquire()
    command[3:6] = euler_angles_des
    command_lock.release()

#--------------------------------------------------------------------------------#
#   get_dist_from_point(): getting dist from the AUV to a desired point          #
#--------------------------------------------------------------------------------#
def get_dist_from_point(point):

    # wait for the mission activation
    # (if manual control mode is active)
    while not mission_mode:
        sleep(TS)

    # conversion into column vectors
    # (this is necessary to do the vector difference and to use norm())
    pos_cur = matrix(get_pos()).T
    point = matrix(point).T

    dist = norm(point - pos_cur)
    return dist

#--------------------------------------------------------------------------------#
#   steer_to_wp(): pointing the fore of Zeno to the direction of the desired wp  #                                           
#--------------------------------------------------------------------------------#
def steer_to_wp(waypoint, blocking_mode, angle_err_max):

    # point the fore of the AUV only if the waypoint is far enough
    # (the distance is chosen > 0.5)
    # we want to avoid rotations if the robot is close to the point 
    dist = get_dist_from_point(waypoint)
    if dist > 0.5:
        # keep the current position and change attitude
        # (desired position = current position)
        set_pos_des(get_pos())

        # desired attitude is equal to the waypoint pointing angles
        euler_angles_des = compute_aiming_angles(waypoint, get_pos())

        # restriction on pitch angle
        if abs(euler_angles_des[1]) > deg2rad(MAX_PITCH):
            euler_angles_des[1] = deg2rad(MAX_PITCH) * sign(euler_angles_des[1])

        # setting desired attitude
        set_euler_angles_des(euler_angles_des)

        # while loop for blocking mode
        while blocking_mode:
            euler_angles_cur = get_euler_angles()

            theta_err = euler_angles_des[1] - euler_angles_cur[1]
            psi_err   = euler_angles_des[2] - euler_angles_cur[2]
        
            if mission_mode \
            and abs(theta_err) < angle_err_max \
            and abs(psi_err)   < angle_err_max:
                break
            sleep(TS)

#--------------------------------------------------------------------------------#
#   go_to_wp(): steering the AUV fore to the desired waypoint, then moving       #
#               it to the waypoint. Pointing angles are constantly checked.      #                                  
#--------------------------------------------------------------------------------#  
# Watch out: only blocking function 
def go_to_wp(waypoint, angle_err_max, pos_err_max):

    # prior orientation towards the waypoint
    steer_to_wp(waypoint, blocking, angle_err_max)
    # setting desired position as the waypoint
    set_pos_des(waypoint)

    # continuous updating of pointing angles to the waypoint
    while True:     
        dist = get_dist_from_point(waypoint)
        # updating pointing angles if far enough from the waypoint,
        # we want to avoid rotations of Zeno if it is closer 
        # than 0.5m to the desired position
        if dist > 0.5:
            euler_angles_cur = get_euler_angles()
            euler_angles_des = compute_aiming_angles(waypoint, get_pos())

            # restriction on pitch angle
            if abs(euler_angles_des[1]) > deg2rad(MAX_PITCH):
                euler_angles_des[1] = deg2rad(MAX_PITCH) * sign(euler_angles_des[1])

            # setting desired attitude
            set_euler_angles_des(euler_angles_des)

            theta_err = euler_angles_des[1] - euler_angles_cur[1]
            psi_err   = euler_angles_des[2] - euler_angles_cur[2]

            if mission_mode \
            and dist < pos_err_max \
            and abs(theta_err) < angle_err_max \
            and abs(psi_err)   < angle_err_max:
                break
        else:
            if mission_mode and dist < pos_err_max:
                break
        sleep(TS)

#--------------------------------------------------------------------------------#
#   translate(): moving Zeno to the desired position while keeping               #
#                its attitude                                                    #                                         
#--------------------------------------------------------------------------------#
def translate(pos_des, blocking_mode, pos_err_max):

    # set desired position and attitude
    # WATCH OUT: we want to keep the current attitude
    set_pos_des(pos_des)
    set_euler_angles_des(get_euler_angles())

    # while loop for blocking mode
    while blocking_mode:
        dist = get_dist_from_point(pos_des)
        if mission_mode and dist < pos_err_max: 
            break
        sleep(TS)

#--------------------------------------------------------------------------------#
#   rotate(): rotating Zeno by desired angles while keeping the current position #                            
#--------------------------------------------------------------------------------#
def rotate(euler_angles_des, blocking_mode, angle_err_max):
    
    # degrees to radians conversion
    euler_angles_des = deg2rad(euler_angles_des)

    # set desired position and attitude
    # WATCH OUT: we want to keep the current position
    set_pos_des(get_pos())
    set_euler_angles_des(euler_angles_des)

    # while loop for blocking mode
    while blocking_mode:
        euler_angles_cur = get_euler_angles()

        phi_err   = euler_angles_des[0] - euler_angles_cur[0]
        theta_err = euler_angles_des[1] - euler_angles_cur[1]
        psi_err   = euler_angles_des[2] - euler_angles_cur[2]
    
        if mission_mode \
        and abs(phi_err)   < angle_err_max \
        and abs(theta_err) < angle_err_max \
        and abs(psi_err)   < angle_err_max:
            break
        sleep(TS)

#--------------------------------------------------------------------------------#
#   approach_obj_and_come_back(): starting to approach the object if the label   #
#                                 wanted is detected with the desired            #
#                                 confidence level. Come back to the mission     #
#                                 path moving to the point the robot left before #                
#--------------------------------------------------------------------------------#
def approach_obj_and_come_back(angle_err_max, pos_err_max):

    # flag indicating whether start position of the approach manoevre
    # has been saved or not
    pos_start_saved = False 
    pos_start = []

    # flag indicating whether end position of the approach manoevre
    # has been saved or not
    pos_stop_saved = False
    pos_stop = []

    # save the last desired position and the last desired attitude
    last_pos_des = get_pos_des()
    last_euler_angles_des = get_euler_angles_des()

    while True:
        dist = get_dist_from_point(last_pos_des)
        euler_angles_cur = get_euler_angles()

        # stop when the AUV reaches last desired position and 
        # last desired angles 
        if mission_mode \
        and dist < pos_err_max \
        and last_euler_angles_des[0] - euler_angles_cur[0] < angle_err_max \
        and last_euler_angles_des[1] - euler_angles_cur[1] < angle_err_max \
        and last_euler_angles_des[2] - euler_angles_cur[2] < angle_err_max:
            break
        
        # request output to the obj_detection thread
        output = obj_detection.get_output()

        # if output contains data and the data aging is less than 5 sec
        # execute following instructions
        if output != None:
            data_aging = output[1]
            if data_aging < 5:
                if not pos_start_saved:
                    # save start position of the approach manoevre
                    pos_start_saved = True
                    pos_start = get_pos()
                
                # box is a list. It has the coordinates of the
                # two points needed to define the bounding box
                # (top-left corner and bottom-right corner)
                box = output[0]

                # distance of the wp_virt from the camera current position 
                step = 10  
                # computing the virtual wp to direct Zeno (NED coordinates)
                wp_virt = compute_virtual_wp(get_pos(), get_euler_angles(), DIM_IMAGE, AOW_VERT, box, step)

                # evaluate bounding box dimensions
                w_box = box[2] - box[0]     # bounding box width
                h_box = box[3] - box[1]     # bounding box height

                # if Zeno isn't close enough to the desired object (bounding box isn't large enough)
                # execute following instructions
                if w_box < DIM_IMAGE[0]/2 or h_box < DIM_IMAGE[1]/2:    
                    # point the desired object and approach it
                    set_euler_angles_des(compute_aiming_angles(wp_virt, get_pos()))
                    set_pos_des(wp_virt)
                else:
                    # stand still for 10 sec in the position reached
                    if not pos_stop_saved:
                        # save end position of the approach manoevre
                        pos_stop_saved = True
                        pos_stop = get_pos()

                    set_pos_des(pos_stop)
                    set_euler_angles_des(compute_aiming_angles(wp_virt, pos_stop))
                    sleep(10) 
                    # end object detection process
                    obj_detection.stop()
                    # go back to start position of the approach manoevre
                    go_to_wp(pos_start, 0.01, 0.2)
                    break
            else:
                obj_detection.stop()
                go_to_wp(pos_start, 0.01, 0.2) 
                break 
        sleep(TS)

#--------------------------------------------------------------------------------#
#   receive_nav_status(): saving the message received from AUV_simulator         #
#                         into the global variable "nav_status"                  #
#--------------------------------------------------------------------------------#
def receive_nav_status():
    global nav_status

    # receive message from AUV_simulator
    message = simulator_com.getPacket()

    if message != None:
        # write on shared variable "nav_status" secured by lock
        nav_status_lock.acquire()
        nav_status = message
        nav_status_lock.release()

#--------------------------------------------------------------------------------#
#   send_command(): sending the "command" message to AUV_simulator               #
#--------------------------------------------------------------------------------#
def send_command():

    # read shared variable "command" secured by lock
    command_lock.acquire()
    message = command
    command_lock.release()

    if message != None:
        # send message "command" to AUV_simulator
        simulator_com.send1d(message)

#--------------------------------------------------------------------------------#
#   start_threads(): starting threads execution                                  #
#--------------------------------------------------------------------------------#
def start_threads():
    global thread1, thread2

    # execute threads (target is the function called by each thread)
    thread1 = threading.Thread(target = handle_messages)
    thread1.start()

    thread2 = threading.Thread(target = mission)
    thread2.start()

#--------------------------------------------------------------------------------#
#   wait_for_threads_termination(): keep waiting for threads termination         #
#--------------------------------------------------------------------------------#
def wait_for_threads_termination():

    thread1.join()
    thread2.join()

#--------------------------------------------------------------------------------#
#   handle_messages(): managing the sending and receiving of messages            #
#                      that connect with the AUV_simulator process               #
#--------------------------------------------------------------------------------#
def handle_messages():
    global mission_mode

    while mission_running:
        mission_mode = get_mission_mode()
        # receiving the "nav_status" message from AUV_simulator
        receive_nav_status()       
        # sending the "command" message to AUV_simulator
        send_command()
      
        sleep(TS)

#---------------------------------------------------------------------------------#
#   mission(): sequence of instructions that build up the programmable mission    #
#---------------------------------------------------------------------------------#
def mission():
    global mission_running
    
    # starting point of the exploration path
    wp = [120, 60, 2]
    go_to_wp(wp, 0.01, 0.2)

    # 20 degrees pitch towards the seabed
    angles = [0, -20, 30]
    rotate(angles, blocking, 0.01)

    # Exploration path : moving along the right side for 100 meters. 
    # We want to give a waypoint with respect to the body frame.
    # Body to NED conversion is needed 
    wp = body2ned([0, 100, 0], get_pos(), get_euler_angles())
    set_pos_des(wp)

    # start object detection process
    # Choose an object to be detected and the desired confidence level you want
    # For now the object to find is an aeroplane
    obj_detection.start('aeroplane', 0.85)
    # pause scouting, approach the object if detected and
    # go back to the mission path
    approach_obj_and_come_back(0.01, 0.2)
    # end object detection process
    obj_detection.stop()

    # continue moving towards the last desired waypoint 
    # with the last desired attitude
    rotate(angles, blocking, 0.01)
    translate(wp, blocking, 0.2)

    # back to base
    wp = [0, 0, 0]
    go_to_wp(wp, 0.01, 0.2)

    angles = [0, 0, 0]
    set_euler_angles_des(angles)
    
    print("Mission completed\n")
    sleep(1)
    mission_running = False

#--------------------------------------------------------------------------------#
#   MAIN                                                                         #
#--------------------------------------------------------------------------------#
# shared variables and their respective mutual exclusion mechanisms 
nav_status = None
nav_status_lock = threading.Lock()

command = 6 * [0]
command_lock = threading.Lock()

# global variables
mission_running = True
mission_mode = True
blocking = True

thread1 = None
thread2 = None

obj_detection = ObjectDetection()

# initializing serial connection with AUV_simulator
simulator_com = SerialConnection(port = 'com1', baud = 115200)
simulator_com.start()

# wait for the first not empty message from AUV_simulator
while nav_status == None:
    receive_nav_status()

# threads running
start_threads()

# wait for threads termination
wait_for_threads_termination()

# close serial connection with AUV_simulator
simulator_com.close()