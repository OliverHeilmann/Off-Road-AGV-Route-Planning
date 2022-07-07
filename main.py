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
XDIMENSION = YDIMENSION = 256    # Max number of nodes in x.y dirs (MUST BE A POWER OF 2!)

XSPACING = YSPACING = 10     # The spacing between nodes in x, y dir [meters]
CORNER_SIZE = 1             # Number of corners to ignore for path planning (to not fall off edge of map)

XTRANSLATE = -(XDIMENSION-1)*XSPACING / 2.  # Offset for terrain in x dir
YTRANSLATE = -(YDIMENSION-1)*YSPACING / 2.  # Offset for terrain in y dir
ZTRANSLATE = 0                              # Offset for terrain in z dir

USE_WAYPOINTS = False    # Option to use fewer waypoints on route to minimise route complexity (blue dots on plots)

APPEARANCE = "TerrainFeatures"      # e.g. "SandyGround" with SCALE = 10, e.g. "CustomAppearance" with SCALE = 1 (see proto files)
SCALE = 1                           # Scale of appearance image over texture (in WeBots simulator)
PIXEL_RESOLUTION = 2048             # Pixel resolution of terrain feature image (MUST BE A POWER OF 2!)

#### KERNEL DENSITY ESTIMATOR PARAMS
H = 200    # Radius (h) defines how much affect each point has to KDE (higher H is more reach)

# x_pts = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,75,75,75,75,80,80,80,80,80,80,80,80,45,45,45,45,45,45,45,45]  # seed x points for elevation locations
# y_pts = [10,10,10,20,20,20,30,30,30,40,40,40,50,50,50,85,75,80,80,80,92,35,35,35,35,35,35,35,35,76,67,67,32,45,67]  # seed y points for elevation locations

x_pts = [168, 173, 173, 168, 167, 171, 173, 171, 170, 170, 170, 170, 171, 173, 201, 183, 211, 242, 250, 255, 308, 338, 372, 417, 408, 178, 178, 188, 178, 193, 205, 185, 186, 183, 183, 188, 260, 290, 346, 385, 408, 406, 347, 295, 248, 210, 203, 231, 238, 267, 403, 356, 270, 257, 103, 71, 75, 100, 315, 326, 351, 368, 438, 487, 525, 488, 578, 617, 521, 452, 375, 306, 258, 292, 307, 320, 286, 310, 166, 101, 97, 105, 101, 103, 107, 112, 111, 111, 77, 33, 18, 7, 7, 27, 60, 137, 137, 142, 252, 295, 311, 291, 225, 178, 131, 113, 58, 25, 37, 36, 33, 30, 45, 148, 122, 137, 188, 241, 253, 317, 407, 506, 542, 566, 567, 601, 647, 688, 686, 570, 470, 468, 381, 331, 325, 287, 251, 221, 280, 226, 226, 285, 410, 445, 498, 501, 481, 460, 483, 483, 263, 135, 118, 97, 51, 630]
y_pts = [699, 723, 758, 788, 815, 859, 894, 932, 994, 1005, 1069, 1102, 1128, 1190, 1222, 1259, 1240, 1255, 1284, 1284, 1308, 1328, 1355, 1402, 1437, 1305, 1340, 1415, 1480, 1540, 1578, 1503, 1425, 1387, 1372, 850, 902, 914, 937, 962, 983, 989, 962, 930, 914, 880, 865, 765, 729, 749, 854, 760, 647, 608, 725, 857, 993, 1115, 1027, 1194, 1200, 1142, 1138, 1138, 1107, 1029, 1035, 1130, 1238, 1267, 1253, 1184, 1132, 1068, 1045, 1073, 1153, 1169, 1230, 1283, 1377, 1435, 1500, 1534, 1580, 1594, 1490, 1355, 1267, 1099, 1045, 944, 844, 762, 724, 708, 708, 1519, 1462, 1493, 1539, 1654, 1713, 1773, 1853, 1904, 1893, 1775, 1669, 1562, 1410, 1319, 1207, 1392, 1710, 1809, 1659, 1649, 1768, 1760, 1667, 1528, 1493, 1453, 1339, 1265, 1223, 1168, 1135, 1313, 1387, 1450, 1523, 1493, 1397, 1372, 1338, 1314, 1203, 1105, 1013, 960, 1068, 1098, 1174, 1245, 1305, 1344, 1364, 1364, 1558, 1967, 2019, 2110, 2134, 1305]

x_pts.extend(x_pts)
y_pts.extend(y_pts)


ADD_NOISE = False       # include additional noise?
SAMPLES = 100           # Number of additional random samples used to generate heat map and terrain profile

#### PATH PLANNING PARAMS
START = (0,0)       # index value which agent starts at after including corner size
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

        self.elevationCorners = np.full( (XDIMENSION, YDIMENSION), 0.0 )
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

        # Now we must resize the elevation data by x-1,y-1 before adding to the Tile Class. This is
        # necessary because the number of grid squares is == number of nodes-1. We must interpolate
        # between the nodes to get the correct elevation data for the CENTER of each grid square/ tile
        # such that the image grid square data corresponds to the correct elevation data.
        elevationNodes = cv2.resize( self.elevationCorners, (XDIMENSION-1, YDIMENSION-1) )  # INTERPOLATION!
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
        slopeNodes = cv2.resize( self.slopeCorners, (XDIMENSION-1, YDIMENSION-1) )  # INTERPOLATION!
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

        iops = [obj.iop for row in self.tiles for obj in row]
        # A = np.array(iops)
        # A += abs(min(iops))
        # B = pow( [A] / np.linalg.norm([A], axis=-1)[:, np.newaxis], 2 )
        # B -= 1

        # AA = iops + abs(min(iops))
        # BB = np.linalg.norm( AA )
        # norm=AA/BB

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

def wbo_vehicle_config( elev : np.ndarray, wpts : np.ndarray ):
    """Save key Webots startup information in text file for C code."""

    # calculate translation values for vehicle (WeBots coordinate system)
    tx = str(wpts[0][0] + XTRANSLATE + (3*XSPACING)/2)
    ty = str(wpts[0][1] + YTRANSLATE + (3*YSPACING/2))
    tz = str(elev[CORNER_SIZE][CORNER_SIZE] + VEHICLE_HEIGHT/2)

    # put all waypoints into string and adjust for elevation map offset in WeBots
    index = lambda el : int((el / XSPACING) - (CORNER_SIZE-1)) + CORNER_SIZE
    wpts_string = ",".join( ['{{{},{},{}}}'.format( str(el[0] + XTRANSLATE + (3*XSPACING)/2),
                                                    str(el[1] + YTRANSLATE + (3*YSPACING/2)),
                                                    str(round(elev[ index(el[1]) ][ index(el[0]) ] + VEHICLE_HEIGHT/2,4)))
                                                    for el in wpts] )

    # structure of .wbo file
    formatted =  """Vehicle Config File
Vehicle {{
translation {} {} {}
rotation {} {} {} {}
count {}
waypoints {}
}}"""   .format(   tx, ty, tz,
                0, 0, -1, -0.85,
                str(len(wpts)),
                wpts_string,
            )
    # Save waypoints as text file usable in WeBots
    try:
        with open('maps/elevationmap_vehicle_config.txt', "w") as f:
            f.write( formatted )
            f.close()
    except:
        raise ValueError('"maps/elevationmap_vehicle_config.txt" did not save!')

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
        for incr, (x, y) in enumerate(zip(x_pts, y_pts)):
            if x < H and y < H: del x_pts[incr], y_pts[incr]

    # Initialise terrain class 
    terrain = Terrain( 1 )

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

        # Save waypoints and starting location for vehicle in config file (readable by Webots C code)
        wbo_vehicle_config( elev = elevationCorners, wpts = waypoints_astar if USE_WAYPOINTS else solutionRoute_astar )

        ################# CREATE FIGURES #################
        # reformat data to matplotlib readable version
        x_rt_greedy, y_rt_greedy = get_xys( solutionRoute_greedy )
        x_wpts_greedy, y_wpts_greedy = get_xys( waypoints_greedy )

        x_rt_astar, y_rt_astar = get_xys( solutionRoute_astar )
        x_wpts_astar, y_wpts_astar = get_xys( waypoints_astar )
    else:
        # no path, create empty lists
        x_rt_greedy = y_rt_greedy = x_wpts_greedy = y_wpts_greedy = []
        x_rt_astar = y_rt_astar = x_wpts_astar = y_wpts_astar = []

    ##### PLOTTING HEADER #####
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(nrows=2, ncols=2)
    fig.suptitle(f'Terrain Heatmaps With Path Planning\n(Max Slope: {MAX_SLOPE_ANGLE} rad, Max Velocity: {MAX_VELOCITY} Km/h)', fontsize=16)
    
    ##### ELEVATION HEATMAP OUTPUT #####
    ax1.set(title="Elevation Heatmap")
    ax1.plot(x_pts,y_pts,'ro')
    ax1.axis(   xmin=-XSPACING/2, xmax=XDIMENSION*XSPACING-XSPACING/2,
                ymin=-YSPACING/2, ymax=YDIMENSION*YSPACING-YSPACING/2)
    # ax1.set_xlabel('Meters'); ax1.set_ylabel('Meters')
    fig.colorbar(   ax1.pcolormesh(terrain.x_mesh,terrain.y_mesh, elevationCorners),
                    ax=ax1,)

    ##### SLOPE HEATMAP OUTPUT #####
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
    fig.colorbar(   ax2.pcolormesh(terrain.x_mesh,terrain.y_mesh, slope),
                    ax=ax2,)
    
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

    plt.show()
    ###################################################