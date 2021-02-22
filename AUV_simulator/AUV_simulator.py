import pygame as pg
import struct

from time import sleep
from numpy import matrix, deg2rad
from pygame_widgets import TextBox, Button

from functions_module import create_view, draw_elements, handle_keyboard_input,  \
                             compute_dynamics, compute_control, wrap_angle,      \
                             build_nav_status, build_msg_UDP, geo2ned              
                             
from constants_module import SCREEN_DIM, DIAL_R, TS, Y_VALUE,   \
                             RED, GREEN, YELLOW, DARK_BLUE, LIGHT_GREY

from UDPUnity import CustomUDP
from serial_connection import SerialConnection

#--------------------------------------------------------------------------------#
#   INTERACTIVE ELEMENTS FUNCTIONS                                               #
#--------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------#
#   output(): textbox function                                                   #
#--------------------------------------------------------------------------------#
def read_textbox():
    global eta_des

    if control_mode and not mission_mode:
        # get string from textbox
        string = textbox.getText()

        # split the string gotten into more strings 
        # (default separator is whitespace)
        values_des = []
        for t in string.split():  
            values_des.append(float(t))
        
        if len(values_des) == 6:
            eta_des = matrix(values_des).T
            for i in range(3, 6):
                eta_des[i] = deg2rad(eta_des[i])
        else:
            print("<<ERROR: Wrong number of inputs. Values must be six.>>\n")
    else:
        print("<<WARNING: Textbox is disabled in this modality.>>\n")

#--------------------------------------------------------------------------------#
#   button1_clicked(): function associated with Button 1 (Control)               #
#--------------------------------------------------------------------------------#
def button1_clicked():
    global control_mode, mission_mode, eta_des, tau 

    control_mode = not control_mode
    if not control_mode:
        mission_mode = False

    tau = matrix(6 * [0.]).T
    eta_des = eta

#--------------------------------------------------------------------------------#
#   button2_clicked(): function associated with Button 2 (Mission)               #
#--------------------------------------------------------------------------------#
def button2_clicked():
    global mission_mode, control_mode, eta_des, ni_des

    mission_mode = not mission_mode
    if mission_mode:
        control_mode = True
    
    eta_des = eta
    ni_des  = matrix(6 * [0.]).T

#--------------------------------------------------------------------------------#
#   MAIN                                                                         #
#--------------------------------------------------------------------------------#
eta = matrix(6 * [0.]).T        #status pos : [x, y, z, phi, theta, psi]
ni  = matrix(6 * [0.]).T        #status vel : [u, v, w, p, q, r]
tau = matrix(6 * [0.]).T        #inputs :     [Fx, Fy, Fz, Mx, My, Mz]

eta_des     = matrix(6 * [0.]).T
ni_des      = matrix(6 * [0.]).T
ni_dot_des  = matrix(6 * [0.]).T

button1_colour = GREEN
button2_colour = GREEN

# flag variables
AUV_simulator_running = True

control_mode = False
mission_mode = False

#--------------------------------------------------------------------------------#
# inizializing UDP connection with Unity
HOST_IP = 'localhost'
HOST_PORT_UDP = 29000
UDP_IP = 'localhost'
UDP_PORT = 25000

unity_com = CustomUDP(HOST_IP, HOST_PORT_UDP, UDP_IP, UDP_PORT)
unity_com.start()

# inizializing connection with mission_manager
mission_com = SerialConnection(port = 'com2', baud = 115200)
mission_com.start()

# inizializing pygame
pg.init()

# creation GUI screen
pg.display.set_caption('Zeno GUI')
screen = pg.display.set_mode(SCREEN_DIM)

# creation of stylised images of the AUV (top, side and back view)   
# note: images are square and image width is equal to dial diameter
image_width = 2 * DIAL_R

image_1 = create_view('images/zeno_top.bmp' , image_width)       # top view
image_2 = create_view('images/zeno_side.bmp', image_width)       # side view
image_3 = create_view('images/zeno_back.bmp', image_width)       # back view

# textbox to write desired position and trim angles
textbox = TextBox(screen, 360, Y_VALUE + 35, 185, 35,
                font            = 'Calibri',
                fontSize        = 20, 
                textColour      = YELLOW,
                radius          = 10, 
                borderThickness = 2,
                borderColour    = LIGHT_GREY, 
                colour          = DARK_BLUE,
                onSubmit        = read_textbox
            )
# button to activate and deactivate control
button1 = Button(screen, 360, Y_VALUE, 90, 30,              
                text            = 'CONTROL',
                font            = pg.font.SysFont('Calibri', 15),
                inactiveColour  = button1_colour, 
                radius          = 10,
                onClick         = button1_clicked,
            )          
# button to activate and deactivate mission
button2 = Button(screen, 455, Y_VALUE, 90, 30,
                text            = 'MISSION',
                font            = pg.font.SysFont('Calibri', 15),
                inactiveColour  = button2_colour, 
                radius          = 10,
                onClick         = button2_clicked,
            )

while AUV_simulator_running:
    # listening for key/mouse events
    events = pg.event.get()
    for event in events:
        if event.type == pg.QUIT:
            AUV_simulator_running = False

    textbox.listen(events)
    button1.listen(events)
    button2.listen(events)

    # computing input vector for dynamic equations
    if control_mode:
        button1.inactiveColour = GREEN
        if not mission_mode:
            textbox.colour = DARK_BLUE
        else:
            textbox.colour = LIGHT_GREY
        tau = compute_control(eta, ni, eta_des, ni_des, ni_dot_des)
    else:
        button1.inactiveColour = RED
        textbox.colour = LIGHT_GREY
        tau = handle_keyboard_input(events, tau)

    # updating status due to dynamic equations
    eta, ni = compute_dynamics(eta, ni, tau)

    # sending message to Unity
    msg_UDP = build_msg_UDP(eta)
    udp_msg = struct.pack('12d', *msg_UDP)
    unity_com.pullPacket(udp_msg)

    # sending massage to mission manager
    nav_status = build_nav_status(eta, ni, mission_mode)
    mission_com.send1c(nav_status)

    # executing this block of code if mission mode is active
    if mission_mode:
        button2.inactiveColour = GREEN
        # receiving message from mission_manager
        command = mission_com.getPacket()
        if command != None:
            command[0:3] = geo2ned(command[0:3])
            eta_des = matrix(command[0:6]).T
    else :
        button2.inactiveColour = RED

    # drawing all graphic elements
    draw_elements(screen, image_1, image_2, image_3, eta, ni, tau)
    textbox.draw()
    button1.draw()
    button2.draw()

    # updating GUI screen
    pg.display.update()
    
    sleep(TS)

# closing UDP connection with Unity
unity_com.close()
# closing serial connection with mission manager
mission_com.close()
# closing pygame
pg.quit()