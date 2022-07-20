"""
Description:
    -   Creates WeBots readable terrain elevation map of .wbo file format. This approach uses
        randomly generated points [x,y] and a quartic regression function to determine point
        densities, called Kernel Density Estimation (KDE); higher point densities correspond 
        to higher elevations.
    -   Using the output terrain elevation map, a slope map is generated using the method shown
        in the imgs folder (https://www.mdpi.com/2220-9964/10/11/785).
    -   Using the aforementioned slope map, a path planning algorithm is used to plot a route
        between two points, while avoiding regions of non-permissable slope angles (a param
        defined in the setup section).
    -   To reduce the number of waypoints required, an R2 approach is then used to select a
        minimum number of waypoints required to reach the target location. The results are
        saved into a text file thereafter.
    -   Results are plotted as two heat maps using Matplotlib

By Oliver Heilmann
"""
# add 'search' directory to path (make sure you launch file from the working
# directory rather than sub-dirs)
import sys
sys.path.append('./search')    

import matplotlib.pyplot as plt
import numpy as np
import math
import random
import cv2

from scipy import stats
from maps.landtypes import Tile, LandTypes
from greedysearch import greedyRoute3D
from astarsearch import astarRoute3D
from collections import defaultdict

############################## SETUP ###################################
#### WEBOTS VEHICLE PROPERTIES
MAX_SLOPE_ANGLE = 0.65      # Maximum permissible slope angle for vehicle in radians (0.65 for Moose)
VEHICLE_LENGTH = 2.964      # Vehicle length in meters
VEHICLE_HEIGHT = 1.145      # Vehicle height in meters
MAX_VELOCITY = 30.0         # Maximum Vehicle velocity in km/h
RSQ_THRESHOLD = 0.9999      # R-Squared value for determining waypoints (lower val ∝ less waypoints)

#### WEBOTS ELEVATION MAP PARAMS
XDIMENSION = YDIMENSION = 128    # Max number of nodes in x.y dirs (MUST BE A POWER OF 2!)

XSPACING = YSPACING = 10    # The spacing between nodes in x, y dir [meters]
CORNER_SIZE = 1             # Number of corners to ignore for path planning (to not fall off edge of map)

XTRANSLATE = -(XDIMENSION-1)*XSPACING / 2.  # Offset for terrain in x dir
YTRANSLATE = -(YDIMENSION-1)*YSPACING / 2.  # Offset for terrain in y dir
ZTRANSLATE = 0                              # Offset for terrain in z dir

USE_WAYPOINTS = False    # Option to use fewer waypoints on route to minimise route complexity (blue dots on plots)

APPEARANCE = "TerrainSandy"         # e.g. "SandyGround" with SCALE = 10, e.g. "TerrainSandy" or "TerrainMatte" with SCALE = 1 (see proto files)
SCALE = 1                           # Scale of appearance image over texture (in WeBots simulator)
PIXEL_RESOLUTION = 2048             # Pixel resolution of terrain feature image (MUST BE A POWER OF 2!)

#### KERNEL DENSITY ESTIMATOR PARAMS
H = 200                      # Radius (h) defines how much affect each point has to KDE (higher H is more reach)
ELEVATION_SCALING = 15       # multiply elevation points by "n" (larger "n" is higher peaks, 0 < n < 1 is smaller peaks)

x_pts = [724, 958, 910, 780, 678, 674, 808, 950, 837, 785, 860, 626, 603, 591, 668, 713, 648, 768, 674, 701, 561, 613, 629, 39, 46, 230, 211, 119, 162, 150, 143, 329, 57, 85, 53, 159, 266, 53, 155, 222, 83, 970, 1086, 1220, 1253, 1217, 1081, 1023, 1210, 1253, 1265, 1250, 1201, 1178, 1257, 1253, 1222, 1225, 1223, 1120, 994, 864, 813, 914, 971, 1067, 1246, 1216, 1161, 1122, 1048, 982, 939, 886, 911, 516, 551, 730, 645, 151, 41, 78, 34, 11, 54, 26, 78, 96, 106, 118, 130, 146, 155, 188, 233, 279, 315, 355, 386, 420, 430, 463, 507, 540, 578, 585, 521, 481, 458, 430, 400, 343, 243, 382, 55, 85, 135, 256, 365, 448, 706, 674, 705, 786, 798, 753, 721, 752, 738, 922, 1001, 1108, 1093, 951, 852, 868, 963, 1128, 1001, 1151, 1071, 613, 468, 638, 728, 418, 328]
y_pts = [14, 405, 555, 595, 520, 378, 308, 348, 510, 406, 397, 238, 126, 49, 27, 157, 162, 76, 106, 60, 24, 17, 45, 227, 65, 25, 87, 139, 215, 73, 27, 35, 144, 79, 22, 401, 390, 557, 672, 642, 644, 71, 35, 119, 218, 305, 379, 348, 377, 305, 186, 114, 39, 19, 27, 62, 14, 12, 50, 17, 22, 19, 32, 60, 37, 18, 255, 335, 417, 419, 392, 421, 456, 394, 349, 775, 714, 624, 667, 320, 795, 708, 662, 614, 660, 722, 1207, 1172, 1151, 1105, 1065, 1017, 999, 971, 959, 965, 979, 1004, 1038, 1087, 1119, 1165, 1225, 1244, 1261, 1280, 1245, 1232, 1215, 1165, 1127, 1080, 1063, 1194, 1192, 1094, 985, 932, 963, 1062, 1182, 1110, 1084, 1093, 1154, 1179, 1140, 1132, 1110, 915, 898, 914, 1015, 1013, 973, 929, 844, 864, 960, 970, 955, 928, 888, 848, 917, 785, 699]

ADD_NOISE = False       # include additional noise?
SAMPLES = 110           # Number of additional random samples used to generate heat map and terrain profile

#### PATH PLANNING PARAMS
# note that min index value is 0 and max is "XDIMENSION - corner size"...
START = (4,118)       # index value which agent starts at after including corner size (row, col)
END = None      # index value which agent ends at after including corner size, set to None for ending at top, right corner (row, col)

#######################################################################

class Terrain( Tile, LandTypes ):
    """Terrain class holds all functions relating to terrain generation."""
    def __init__( self, *args, **kwargs ):
        # initialise inherited land type class
        LandTypes.__init__( self )

        # make a 2D array of tile objects, one for each node which contains
        # terrain "traits"/ attributes...
        self.tiles = np.zeros( (YDIMENSION-1, XDIMENSION-1), dtype=object )
        for iy, ix in np.ndindex( self.tiles.shape ):
            self.tiles[ iy, ix ] = Tile()

        # prepare to store values, create variable names
        self.x_mesh = list
        self.y_mesh = list
        self.elevationCorners = np.ndarray
        self.slopeCorners = np.ndarray
        self.image = np.ndarray

    def kde_quartic( self, d, H ):
        """Function to calculate intensity with quartic kernel."""
        return (15/16)*(1-(d/H)**2)**2

    def elevation_map( self, xs : list, ys : list ):
        """Take input points and return an intensity map as a 2D numpy array."""
        # Construct grid
        x_grid = np.arange(0,XDIMENSION*XSPACING,XSPACING)
        y_grid = np.arange(0,YDIMENSION*YSPACING,YSPACING)
        self.x_mesh, self.y_mesh = np.meshgrid(x_grid,y_grid)

        # Grid center points
        xc=self.x_mesh+(XSPACING/2)
        yc=self.y_mesh+(YSPACING/2)

        self.elevationCorners = np.full( (YDIMENSION, XDIMENSION), 0.0 )
        for j in range(len(xc)):
            for k in range(len(xc[0])):
                kde_value_list=[]
                for i in range(len(xs)):
                    # Calculate distance
                    d = math.sqrt( math.pow(xc[j][k]-xs[i],2) + math.pow(yc[j][k]-ys[i],2) )
                    if d<=H:
                        p=self.kde_quartic(d,H)
                    else:
                        p=0
                    kde_value_list.append(p)
                # Sum all intensity values
                p_total=sum(kde_value_list)
                self.elevationCorners[j][k] = p_total
        
        # Set border heights to 0 to avoid steep edges in WeBots
        self.elevationCorners[0,:-1] = self.elevationCorners[:-1,-1] = self.elevationCorners[:-1,0] = self.elevationCorners[-1,:-1] = 0
        
        # Scale values by the user specified value
        self.elevationCorners *= ELEVATION_SCALING                                        

        # Now we must resize the elevation data by x-1,y-1 before adding to the Tile Class. This is
        # necessary because the number of grid squares is == number of nodes-1. We must interpolate
        # between the nodes to get the correct elevation data for the CENTER of each grid square/ tile
        # such that the image grid square data corresponds to the correct elevation data.
        elevationNodes = cv2.resize(self.elevationCorners,
                                    (YDIMENSION-1, XDIMENSION-1), 
                                    interpolation = cv2.INTER_LINEAR_EXACT )  # interpolation method!
        for iy, ix in np.ndindex( elevationNodes.shape ):
            self.tiles[ iy, ix ].elevation = elevationNodes[ iy, ix ]
        return (self.x_mesh, self.y_mesh), self.elevationCorners

    def wbo_map( self, elevationCorners : np.ndarray ):
        """Create .wbo WeBots readable terrain map using intensity map 2D numpy array."""

        # convert numpy array into string format usable by .wbo file format
        heights = ",".join( [",".join(item) for item in np.round(elevationCorners,2).astype(str)] )

        # structure of .wbo file
        formatted =  """#VRML_OBJ R2022a utf8
DEF TERRAIN Solid {{
    translation {} {} {}
    children [
        Shape {{
            appearance {} {{
                textureTransform TextureTransform {{
                scale {} {}
            }}
            }}
            geometry DEF TERRAIN_MAP ElevationGrid {{
                height [{}]
                xDimension {}
                xSpacing {}
                yDimension {}
                ySpacing {}
            }}
        }}
    ]
name "ELE_MOD"
boundingObject USE TERRAIN_MAP
}}"""   .format(XTRANSLATE,
                YTRANSLATE,
                ZTRANSLATE,
                APPEARANCE,
                SCALE, SCALE,
                heights,
                XDIMENSION,
                XSPACING,
                YDIMENSION,
                YSPACING
                )
        try:
            with open('maps/elevationmap_heatmap.wbo', 'w') as f:
                f.write( formatted )
                f.close()
        except:
            raise ValueError('"elevationmap_heatmap.wbo" did not save!')

    def slope_map( self, elevCnr : np.ndarray ):
        """Calculate the slope map using previously generated terrain elevation data."""
        rows, cols = elevCnr.shape
        self.slopeCorners = np.zeros([rows, cols])
        elevCnr = np.pad(elevCnr, 1, mode='symmetric')   # add padding for kernel
        for i in range(1,rows+1):
            for j in range(1,cols+1):
                slope_we = ((elevCnr[i+1][j-1] + 2*elevCnr[i][j-1] + elevCnr[i-1][j-1]) -    \
                            (elevCnr[i+1][j+1] + 2*elevCnr[i][j+1] + elevCnr[i-1][j+1]))/    \
                            (8 * XSPACING)

                slope_sn = ((elevCnr[i+1][j+1] + 2*elevCnr[i+1][j] + elevCnr[i+1][j-1]) -    \
                            (elevCnr[i-1][j+1] + 2*elevCnr[i-1][j] + elevCnr[i-1][j-1]))/    \
                            (8 * YSPACING)

                self.slopeCorners[i-1][j-1] = np.arctan( math.sqrt(slope_we**2 + slope_sn**2) )

        # Now we must resize the slope data by x-1,y-1 before adding to the Tile Class. This is
        # necessary because the number of grid squares is == number of nodes-1. We must interpolate
        # between the nodes to get the correct elevation data for the CENTER of each grid square/ tile
        # such that the image grid square data corresponds to the correct slope data.
        slopeNodes = cv2.resize(self.slopeCorners,
                                (YDIMENSION-1, XDIMENSION-1), 
                                interpolation = cv2.INTER_LINEAR_EXACT )  # interpolation method!
        for iy, ix in np.ndindex( slopeNodes.shape ):
            self.tiles[ iy, ix ].slope = slopeNodes[ iy, ix ]
        return self.slopeCorners

    def image_map( self, imageDir = "webots_moose/protos/textures/TerrainFeatures.png", pixelRes = 2048, check = False ):
        """Divide terrain image into same number of grid squares as other terrain features."""
        # Load an color image in BGR, reformat and get key params
        self.image = cv2.imread( imageDir, cv2.IMREAD_UNCHANGED )
        self.image = cv2.resize( self.image, (pixelRes, pixelRes), interpolation = cv2.INTER_AREA )
        row, col, _ = self.image.shape
        
        # add alpha channel to image so that WeBots doesn't perform image interpolation
        self.image = cv2.cvtColor( self.image, cv2.COLOR_RGB2RGBA ) # --> RGBA

        # resave image to correct dimensions for WeBots simulator
        cv2.imwrite("webots_moose/protos/textures/TerrainFeaturesScaled.png", self.image)
        
        # calculate segment size in pixels
        M = row//(YDIMENSION-1)
        N = col//(XDIMENSION-1)

        # get tiles in format/ sequence shown below:
        #     y
        #     ^
        #     |   1,0      1,1
        #     |
        #     |   0,0      0,1
        #   (0,0)–––––––––––––––––> x
        # each tile will receive a 2D array with the corresponding image segment stored
        # as a 2D array
        for rn, r in enumerate(range(row,0,-M)):
            for cn, c in enumerate(range(0,col,N)):
                self.tiles[ rn, cn ].image = self.image[r-M:r, c:c+N] # x4 channel image into tiles

        # check segmentation has worked...
        if check:
            cv2.namedWindow('Main', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Tile', cv2.WINDOW_NORMAL)
            cv2.imshow( 'Main', self.image )
            cv2.resizeWindow('Main', 250, 250 )
            for row in self.tiles:
                for col in row:
                    cv2.imshow( 'Tile', col.image )
                    cv2.resizeWindow('Tile', 250, 250 )
                    cv2.waitKey(0)
            cv2.destroyAllWindows()

    def iop_map( self, *args, **kwargs ):
        """Calculate vehicle velocities based on vegetation roughness factors presented in literature."""
        # get % coverage area of feature within input tile
        getPctArea = lambda mask : (cv2.countNonZero(mask) * tile.image.shape[-1]) / tile.image.size

        # loop through tile objects and calculate their IOP values
        for tile in np.nditer( self.tiles, flags=['refs_ok'] ):
            tile = tile.tolist()    # make accessible

            for feature in self.get_type_keys():
                vrf = tile.get_type_info( feature )[-1]  # vegetation roughness factor (VRF)

                # if slope type then append None placeholder, we handle this differently
                if feature == "Slope": tile.iop += tile.slope * vrf             # updating iop total
                else:
                    image_hsv = cv2.cvtColor( tile.image, cv2.COLOR_BGR2HSV )   # get hsv of tile image
                    lower, upper = self.get_colour_range( feature )             # get colour range
                    image_mask = cv2.inRange( image_hsv, lower, upper )         # create mask
                    tile.iop += getPctArea(image_mask) * vrf                    # updating iop total

            # Set IOP values to be in range -1 < x < +1
            if tile.iop > 1: tile.iop = 1
            elif tile.iop < -1: tile.iop = -1

            # calculate estimated velocity at tile using IOP
            tile.velocity = 0.5 * MAX_VELOCITY * ( 1 + tile.iop )

        # get all tile velocities and put into list
        iops = [obj.iop for row in self.tiles for obj in row]
        velocities = [obj.velocity for row in self.tiles for obj in row]

        # scaled = 2.*(iops - np.min(iops))/np.ptp(iops)-1
        return np.array(iops).reshape(-1, XDIMENSION-1), np.array(velocities).reshape(-1, XDIMENSION-1)   # return 2D array of iops

def isPowerOfTwo(n):
    """Function to check if x is power of 2."""
    if (n == 0): return False
    while (n != 1):
        if (n % 2 != 0): return False
        n = n // 2
    return True

def get_xys( route ):
    xs = [coord[0]+XSPACING for coord in route]
    ys = [coord[1]+YSPACING for coord in route]
    return xs, ys

def get_waypoints( route : np.ndarray ):
    """Create a set of waypoints for vehicle to drive toward in WeBots simulator."""
    wpts = [ (route[0][0], route[0][1]) ]

    x_prev = route[0][0]
    y_prev = route[0][1]

    xs, ys = [], []
    for x_curr, y_curr in route:
        # append newest x,y coords to respective lists
        xs.append( x_curr )
        ys.append( y_curr )

        # get R value for determining waypoints
        if not all(x==xs[0] for x in xs):
            slope, intercept, r_value, p_value, std_err = stats.linregress(xs, ys)
            rSq = r_value**2
        else: rSq = 1.0

        # if the new coord drops r2 value below threshold, add the previous coord to
        # the list of waypoints and reset the array to only include current (x,y)
        if  0 < rSq < RSQ_THRESHOLD:
            wpts.append( (x_prev, y_prev) )
            xs, ys = [x_prev, x_curr], [y_prev, y_curr]

        # update previous values to equal current before new loop
        x_prev, y_prev = x_curr, y_curr
    wpts.append( (x_curr, y_curr) )
    return wpts

def get_wpt_elev( wpt : np.array, terrain : Terrain ):
    """Get waypoint elevation float value from corresponding tile object."""
    # note that waypoints are in the form [X, Y] i.e. [Col, Row]! This is opposite to
    # array lookup notation...
    c, r = ( np.array(wpt) / ((XSPACING+YSPACING)/2.0) ).astype(int) + CORNER_SIZE
    return terrain.tiles[r, c].elevation

def get_xyz_wb( wpt : np.array, trn : Terrain, dp = 4 ):
    webots_coords = lambda n, dn, dw : n + dn + (3*dw)/2
    x = webots_coords(wpt[0], XTRANSLATE, XSPACING)
    y = webots_coords(wpt[1], YTRANSLATE, YSPACING)
    z = round( get_wpt_elev( wpt, trn ) + VEHICLE_HEIGHT/2, dp )
    return x, y, z

def wbo_vehicle_config( wpts : np.ndarray, trn : Terrain ):
    """Save key Webots startup information in text file for C code."""
    # calculate translation values for vehicle (WeBots coordinate system)
    tx, ty, tz = get_xyz_wb( wpts[0], trn )

    # put all waypoints into string and adjust for elevation map offset in WeBots
    wpts_string = ",".join([ '{{{},{},{}}}'.format(x,y,z+0.3) for x,y,z in [get_xyz_wb(el, trn) for el in wpts] ])

    # Get Terrain Class : [Mid RGB, Velocity] from trn object
    trnWebots = trn.get_RGB_WeBots( maxvel = MAX_VELOCITY )

    # Colour of terrain classes – output in the form {terrain_type,r,g,b,vrf},{terrain_type,r,g,b,vrf}...
    # shift Firebrake colour by a number because blacks do not appear as dark with textures in Webots
    classes_str = ",".join( ["{{{},{},{},{},{}}}".format(   i[0],                                       # terrain class
                                                            i[1][0][0] if i[0] != 'Firebrake' else 50,  # red
                                                            i[1][0][1] if i[0] != 'Firebrake' else 50,  # green
                                                            i[1][0][2] if i[0] != 'Firebrake' else 50,  # blue
                                                            i[1][-1])                                   # vehicle velocity
                            for i in trnWebots.items()] )

    # Get starting direction of vehicle from waypoint0 to waypoint1 in radians
    x0, y0, z0 = get_xyz_wb( wpts[0], trn )
    x1, y1, z1 = get_xyz_wb( wpts[1], trn )
    dTheta = math.atan( (abs(y0 - y1) / abs(x0 - x1)) if abs(x0 - x1) != 0 else math.inf )
    dTheta = math.pi - dTheta if x0 > x1 else dTheta
    dz = 1 if y0 < y1 else -1

    # structure of .wbo file
    formatted =  """Vehicle Config File
Vehicle {{
    translation {} {} {}
    rotation {} {} {} {}
    wpts_count {}
    waypoints {}
    trn_count {}
    terrain {}
}}"""   .format(   tx, ty, tz,
                0, 0, dz, dTheta,
                str(len(wpts)),
                wpts_string,
                str(len(trnWebots.keys())),
                classes_str,
            )
    # Save waypoints as text file usable in WeBots
    try:
        with open('maps/elevationmap_vehicle_config.txt', "w") as f:
            f.write( formatted )
            f.close()
    except:
        raise ValueError('"maps/elevationmap_vehicle_config.txt" did not save!')

def elevation_per_distance( wpts : np.ndarray, trn : Terrain ):
    """Get the elevation per distance travelled, return as xs and ys lists."""
    # calculate translation values for vehicle (WeBots coordinate system)
    x0, y0, z0 = get_xyz_wb( wpts[0], trn )
    dist = 0.0
    dists = [0.0]
    elevs = [z0]
    for wpt in wpts[1:]:
        x1, y1, z1 = get_xyz_wb( wpt, trn ) # get new coords
        dist += math.sqrt(  pow( x0 - x1, 2.0 ) + 
                            pow( y0 - y1, 2.0 ) + 
                            pow( z0 - z1, 2.0 ) )
        dists.append( dist )
        elevs.append( z1 )
        x0 = x1
        y0 = y1
        z0 = z1
    return dists, elevs


# Main processing
if __name__ == '__main__':
    # check params ar acceptable...
    if not isPowerOfTwo(XDIMENSION) or not isPowerOfTwo(YDIMENSION):
        raise ValueError("""\n\n[ERROR]: XDIMENSION and YDIMENSION must be a power of two!
         WeBots will reformat the image dimensions when applied onto the terrain. As the calculations
         for terrain passability require equal and accurate grid squares, a reformatting of the image
         shape would lead to inaccurate estimations of path costs.\n\n""")
    else:
        # silently make sure pixel resolution is greater or equal to grid resolution
        PIXEL_RESOLUTION = XDIMENSION if PIXEL_RESOLUTION < XDIMENSION else PIXEL_RESOLUTION

        # Otherwise add 1 to each value so that grid squares = nodes - 1 (needed to scale image size in Webots)
        XDIMENSION += 1; YDIMENSION += 1
       
    if ADD_NOISE:
        samples = SAMPLES if SAMPLES <= np.mean([XDIMENSION, YDIMENSION]) else int(XDIMENSION/2)
        x_pts.extend( random.sample( range(0, XDIMENSION*XSPACING), samples) )
        y_pts.extend( random.sample( range(0, YDIMENSION*YSPACING), samples) )

        # delete values near starting area
        # for incr, (x, y) in enumerate(zip(x_pts, y_pts)):
        #     if x < H and y < H: del x_pts[incr], y_pts[incr]

    # Initialise terrain class 
    terrain = Terrain( )

    # Create intensity 2D numpy array using user defined params
    print("[INFO]: Creating Elevation Map...")
    (_,_), elevationCorners = terrain.elevation_map( x_pts, y_pts )

    print("[INFO]: Dividing Terrain Image into Grid Squares...")
    _ = terrain.image_map( pixelRes = PIXEL_RESOLUTION )

    # Generate output .wbo file using 2D numpy array
    print("[INFO]: Creating WeBots Map...")
    terrain.wbo_map( elevationCorners )

    # Slope Map generation
    print("[INFO]: Calculating Slope Map...")
    slope = terrain.slope_map( elevationCorners )

    # Index of Passability generation
    print("[INFO]: Calculating Index of Passability...")
    iop2dArr, vel2dArr = terrain.iop_map( )

    try:
        # create empty lists to overwrite if paths are found
        x_rt_greedy = y_rt_greedy = dists_greedy = elevs_greedy = []
        x_rt_astar = y_rt_astar  = dists_astar = elevs_astar = []
        y_wpts_greedy, x_wpts_greedy = START
        y_wpts_astar, x_wpts_astar = START

        # Get possible route using Greedy search approach as list [(x1,y1), (x2,y2) ...]
        # Don't pass border values as their slopes are not accurate due to kerneling method. 
        # Answers are returned as INDEX VALUES OF THE INPUT ARRAY!
        clip = lambda array2D : array2D[CORNER_SIZE:-CORNER_SIZE, CORNER_SIZE:-CORNER_SIZE]
        solutionRoute_greedyIndex = greedyRoute3D(  clip(terrain.tiles),            # terrain tile classes 2D array
                                                    maxvelocity=MAX_VELOCITY,       # max vehicle velocity from data sheet in km/h
                                                    maxslope=MAX_SLOPE_ANGLE,       # max permissible slope angles
                                                    start=START,                    # starting agent position
                                                    goal=END,                       # ending agent position
                                                    gridsize=(XSPACING,YSPACING) )  # size of each grid segment in [m]

        solutionRoute_astarIndex = astarRoute3D(clip(terrain.tiles),            # terrain tile classes 2D array
                                                maxvelocity=MAX_VELOCITY,       # max vehicle velocity from data sheet in km/h
                                                maxslope=MAX_SLOPE_ANGLE,       # max permissible slope angles
                                                start=START,                    # starting agent position
                                                goal=END,                       # ending agent position
                                                gridsize=(XSPACING,YSPACING) )  # size of each grid segment in [m]

        # if a path exists then continue...
        if solutionRoute_greedyIndex and solutionRoute_astarIndex:
            # shift results to account for lambda border clipping step shown above. Also modify
            # results to show solution in absolute coordinates rather than index values of the input
            transform_to_webots = lambda the_list : (np.array(the_list) + (CORNER_SIZE-1)) * XSPACING
            solutionRoute_greedy = transform_to_webots( solutionRoute_greedyIndex )
            solutionRoute_astar  = transform_to_webots( solutionRoute_astarIndex  )

            # Make set of waypoints for vehicle based on solution
            # Returns as [ (x1,y1), (x2,y2) ... ]
            waypoints_greedy = get_waypoints( route = solutionRoute_greedy )
            waypoints_astar = get_waypoints( route = solutionRoute_astar )

            # get elevation data per distance travelled to plot later
            dists_greedy, elevs_greedy = elevation_per_distance(  solutionRoute_greedy, trn = terrain)
            dists_astar, elevs_astar = elevation_per_distance(  solutionRoute_astar, trn = terrain)

            # Save waypoints and starting location for vehicle in config file (readable by Webots C code)
            wbo_vehicle_config( wpts = waypoints_astar if USE_WAYPOINTS else solutionRoute_astar,
                                trn = terrain )

            ################# CREATE FIGURES #################
            # reformat data to matplotlib readable version
            x_rt_greedy, y_rt_greedy = get_xys( solutionRoute_greedy )
            x_wpts_greedy, y_wpts_greedy = get_xys( waypoints_greedy )

            x_rt_astar, y_rt_astar = get_xys( solutionRoute_astar )
            x_wpts_astar, y_wpts_astar = get_xys( waypoints_astar )
    except: pass # continue silently after an error

    ##### PLOTTING HEADER #####
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(nrows=2, ncols=2)
    fig.suptitle(f'Terrain Heatmaps With Path Planning\n(Max Slope: {MAX_SLOPE_ANGLE} rad, Max Velocity: {MAX_VELOCITY} Km/h)', fontsize=16)
    
    ##### ELEVATION HEATMAP OUTPUT #####
    ax1.set(title="Elevation Heatmap")
    # ax1.plot(x_pts,y_pts,'ro')
    ax1.axis(   xmin=-XSPACING/2, xmax=XDIMENSION*XSPACING-XSPACING/2,
                ymin=-YSPACING/2, ymax=YDIMENSION*YSPACING-YSPACING/2)
    # ax1.set_xlabel('Meters'); ax1.set_ylabel('Meters')
    fig.colorbar(   ax1.pcolormesh(terrain.x_mesh,terrain.y_mesh, elevationCorners),
                    ax=ax1,)

    # ##### SLOPE HEATMAP OUTPUT #####
    ax2.set(title="Slope Heatmap")
    # plot Greedy
    ax2.plot(x_rt_greedy, y_rt_greedy,'g-')
    ax2.plot(x_wpts_greedy, y_wpts_greedy,'ko', label='_nolegend_')
    # plot A* 
    ax2.plot(x_rt_astar, y_rt_astar,'r-')
    ax2.plot(x_wpts_astar, y_wpts_astar,'bo', label='_nolegend_')
    # plot axes and legend
    ax2.axis(   xmin=-XSPACING/2, xmax=XDIMENSION*XSPACING-XSPACING/2,
                ymin=-YSPACING/2, ymax=YDIMENSION*YSPACING-YSPACING/2)
    ax2.legend(['Greedy', 'A*'])
    cbar = fig.colorbar(    ax2.pcolormesh(terrain.x_mesh,terrain.y_mesh, slope, vmax=MAX_SLOPE_ANGLE),
                            ax=ax2)
    cbar.cmap.set_over('white')

    ##### PLOT TERRAIN CLASSES IMAGE IN RGB #####
    ax3.set(title="Terrain Classes Image")
    ax3.imshow( cv2.cvtColor(terrain.image, cv2.COLOR_BGR2RGB) )
    # plot empty data and show key of colours and respective classes
    [  ax3.plot(np.NaN, np.NaN, '-', color=terrain.get_mid_mtlb(key), label=key) 
                        for c, key in enumerate(terrain.get_type_keys(drop='Slope')) ]
    # ax3.legend()

    ##### VELOCITY HEATMAP OUTPUT #####
    ax4.set(title="Vehicle Velocity Heatmap")
    # plot Greedy
    ax4.plot(x_rt_greedy, y_rt_greedy,'g-')
    ax4.plot(x_wpts_greedy, y_wpts_greedy,'ko', label='_nolegend_')
    # plot A* 
    ax4.plot(x_rt_astar, y_rt_astar,'r-')
    ax4.plot(x_wpts_astar, y_wpts_astar,'bo', label='_nolegend_')
    ax4.legend(['Greedy', 'A*'])
    
    # add in buffer row and col to correct heatmap scaling (so nodes are in center of tiles)
    vel2dArr = cv2.resize( vel2dArr, (XDIMENSION,YDIMENSION) )
    fig.colorbar( ax4.pcolormesh(terrain.x_mesh,terrain.y_mesh, vel2dArr), ax=ax4 )

    ##### ELEVATION OVER DISTANCE OUTPUT #####
    fig2, (ax5) = plt.subplots(nrows=1, ncols=1)
    ax5.set(title="Elevation Gain Over Distance Travelled")
    ax5.plot(dists_greedy, elevs_greedy,'g-')
    ax5.plot(dists_astar, elevs_astar,'r-')
    ax5.set_ylabel('Elevation [m]'); ax5.set_xlabel('Distance [m]')
    ax5.legend(['Greedy', 'A*'])

    plt.show()
    ###################################################