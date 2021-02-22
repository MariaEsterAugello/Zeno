#-------------------------------------------------------------------------------------#
#   DEFINING CONSTANTS                                                                #
#-------------------------------------------------------------------------------------#
#---Geodetic WGS84 System Parameters--------------------------------------------------#
R_E = 6378137           # reference ellipsoid: semi-major axis  [m]
ECC = 0.0818191908426   # reference ellipsoid: eccentricity     [-]

#---NED System Parameters-------------------------------------------------------------#
LAT_0 = 0.76307959105   # NED origin latitude                   [rad]
LON_0 = 0.18134648465   # NED origin longitude                  [rad]
ALT_0 = 0               # NED origin altitude                   [m]

#---Environment Parameters------------------------------------------------------------#
GRV = 9.81              # gravity acceleration                 [m/s^2]
RHO = 1025.0            # seawater density (salinity 3.7 %)    [kg/m^3] 

#---AUV Model Parameters--------------------------------------------------------------#
VOL  = 0.042            # vehicle volume                       [m^3]
MASS = 42.56            # vehicle mass                         [kg]
IN_X = 0.94             # moment of inertia x axis             [kg*m^2]
IN_Y = 3.28             # moment of inertia y axis             [kg*m^2]
IN_Z = 3.79             # moment of inertia z axis             [kg*m^2]
DIST_B = 0.006          # distance of the centre of buoyancy   [m] 
                        # relative to the centre of gravity

# hydrodynamic resistence coefficients (linear contribution). 
# Diagonal elements:
D1 = 30.31              # [kg/s]
D2 = 59.68              # [kg/s]
D3 = 72.60              # [kg/s]
D4 = 1.84               # [kg*m^2/s]
D5 = 1.29               # [kg*m^2/s]
D6 = 1.32               # [kg*m^2/s]

MAX_DEPTH = 48.0        # maximum depth                         [m]
MIN_DEPTH = 0.0         # minimum depth: sea level altitude     [m]
MAX_SPEED = 15.0 #1.543 # maximum speed                         [m/s] (1.543 = circa 3 nodi)
MAX_SPIN  = 5.0         # maximum rotation speed                [rad/s]
MAX_PITCH = 80.0        # maximum pitch angle                   [degrees]
TS        = 0.005       # sampling time                         [s]

#---Control System Constants----------------------------------------------------------#
KD = 10                 # derivative component constant         [1/s]
KP = 10                 # proportional component constant       [1/s^2]

#---Graphic User Interface Parameters-------------------------------------------------#
# screen dimensions                                             [pixels]
SCREEN_DIM = (600, 350)

# dial dimensions and relative positions                        [pixels]
DIAL_R = 75                             # dial radius
DIAL_BORDER_T = 10                      # dial border thickness
LINE_T = 1                              # dial cross lines thickness

DIAL_SD = DIAL_R + DIAL_BORDER_T + 15   # semi-distance between dial centres

# 16 variable values are printed on the screen (eta, ni, tau components).
# They are printed in a grid with 3 columns and 6 rows:
X_VALUE = DIAL_SD - DIAL_R              # horizontal position of the first column 
Y_VALUE = 225                           # vertical position of the first row 
DIST_X_VALUE = 100                      # horizontal distance between two columns 
DIST_Y_VALUE = 17                       # vertical distance between two rows    
DIM_FONT = 20                           # font dimension of printed values

# color palette (R, G, B)
WHITE      = (255, 255, 255)
YELLOW     = (255, 255,   0)
RED        = (255,   0,   0)
GREEN      = (  0, 255,   0)
PINK       = (255,   0, 255)
GREY       = ( 60,  60,  60)
LIGHT_GREY = (120, 120, 120)
DARK_BLUE  = (  3,  33, 133)

#---Unity Camera Constants------------------------------------------------------------#
DIM_IMAGE = (640, 480)                  # camera image dimensions [pixels]
AOW_VERT = 60                           # vertical angle of view  [degrees]