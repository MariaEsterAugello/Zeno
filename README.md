# Zeno READ ME

# The peculiarity of mission_manager is that it is programmable.
# You can change the order of the functions as you like. There are blocking and not blocking functions available. You can reasonably use the detection function only if the not blocking functions are used.

# HOW TO RUN
# • Create a “Pair” device on Virtual Serial Ports Emulator (COM1-COM2)
# • Import package Zeno as a new project on Unity 3D and click play
# • Open two different terminals and run AUV_simulator.py and mission_manager.py
# • Zeno GUI:
#     ▪ If CONTROL button is green: the control is active, the robot moves to eta_des if you write it in the textbox as six numbers separated by using the space bar
#     ▪ If MISSION button is green: the programmable mission starts, you can choose the order of the functions as you like in the MAIN of mission_manager.py
#     ▪ If both buttons are red:
#         W = move ahead (+surge) 
#         S = move back (-surge)
#         A = move left (+sway)
#         D = move right (-sway)
#         Q = go up (+heave)
#         E = go down (-heave)
#         Z = +yaw
#         X = -yaw
#         arrow down = -pitch
#         arrow left = +roll
#         arrow up = +pitch
#         arrow right = -roll
