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
# For scalene profiling use the following cmd:
#   --> /Users/Oliver/opt/anaconda3/envs/DISS/bin/python -m scalene main.py 

# add 'search' directory to path (make sure you launch file from the working
# directory rather than sub-dirs)
import sys
sys.path.append('./search')

# suppress deprecation warnings (for matplotlib)
import warnings
warnings.filterwarnings( "ignore" )
import matplotlib.pyplot as plt

from os import walk
import numpy as np
import time
import math
import random
import cv2

from scipy import stats
from maps.vehicles import Vehicles
from maps.landtypes import Tile, LandTypes
from maps.dem import DEM
from greedysearch import greedyRoute3D
from astarsearch import astarRoute3D
from dijkstrasearch import dijkstraRoute3D

############################## SETUP ###################################
#### WEBOTS VEHICLE PROPERTIES
VEHICLE = "MOOSE"      #  Choose from "MOOSE", "HUMAN" or "MOTOCROSS BIKE"

vehicleInfo = Vehicles( VEHICLE )                  # Get the vehicle information from the class database in vehicles.py
MAX_SLOPE_ANGLE =  vehicleInfo["max_slope_angle"]  # Maximum permissible slope angle for vehicle in radians (0.65 for Moose)
VEHICLE_LENGTH  =  vehicleInfo["length"]           # Vehicle length in meters (2.964 for moose)
VEHICLE_HEIGHT  =  vehicleInfo["height"]           # Vehicle height in meters (1.145 for moose)
MAX_VELOCITY    =  vehicleInfo["max_velocity"]     # Maximum Vehicle velocity in km/h  (30.0 for moose)

#### WEBOTS TERRAIN MAP PARAMS
FOLDERPATH = 'maps/Colorado2'   # path to folder with TIFF and PNG files. Set USEDEM to True if you want
                                # to use the DEM TIFF file, else set to False to create synthetic elevation
                                # maps. A path to a valid PNG terrain file is required regardless in order
                                # to apply a texture to the resultant terrain.
USEDEM = True           # If set to true, real DEM data is used for path planning and Webots. 
                        # If false, create random terrain (or user defined), see 'KERNEL DENSITY 
                        # ESTIMATOR PARAMS' below for more configuration options if this option
                        # is selected.
SAVEMAP = False          # If true then save the output elevation map else, only use it for path
                        # planning and plotting graphs. If it is not saved, running the WeBots 
                        # application will import the previous elevation map instead. This is 
                        # useful where one wishes to test the accuracy of path planning at differing
                        # resolutions while maintaining the same terrain and elevation details.

XDIMENSION = YDIMENSION = 512   # Max number of nodes in x.y dirs (MUST BE A POWER OF 2!)
XSPACING = YSPACING = 4         # The spacing between nodes in x, y dir [meters]
CORNER_SIZE = 1                 # Number of corners to ignore for path planning (to not fall off edge of map)

XTRANSLATE = -XDIMENSION*XSPACING / 2.    # Offset for terrain in x dir
YTRANSLATE = -YDIMENSION*YSPACING / 2.    # Offset for terrain in y dir
ZTRANSLATE = 0                              # Offset for terrain in z dir

APPEARANCE = "TerrainMatte"         # e.g. "SandyGround" with SCALE = 10, e.g. "TerrainSandy" or "TerrainMatte" with SCALE = 1 (see proto files)
SCALE = 1                           # Scale of appearance image over texture (in WeBots simulator)
PIXEL_RESOLUTION = 2048             # Pixel resolution of terrain feature image (MUST BE A POWER OF 2!), images are 16384x16384

INTERP = cv2.INTER_CUBIC            # Method for scaling up/down data (elevation, DEM and slope)

#### KERNEL DENSITY ESTIMATOR PARAMS
H = 200                      # Radius (h) defines how much affect each point has to KDE (higher H is more reach)
ELEVATION_SCALING = 15       # multiply elevation points by "n" (larger "n" is higher peaks, 0 < n < 1 is smaller peaks)

x_pts = [724, 958, 910, 780, 678, 674, 808, 950, 837, 785, 860, 626, 603, 591, 668, 713, 648, 768, 674, 701, 561, 613, 629, 39, 46, 230, 211, 119, 162, 150, 143, 329, 57, 85, 53, 159, 266, 53, 155, 222, 83, 970, 1086, 1220, 1253, 1217, 1081, 1023, 1210, 1253, 1265, 1250, 1201, 1178, 1257, 1253, 1222, 1225, 1223, 1120, 994, 864, 813, 914, 971, 1067, 1246, 1216, 1161, 1122, 1048, 982, 939, 886, 911, 516, 551, 730, 645, 151, 41, 78, 34, 11, 54, 26, 78, 96, 106, 118, 130, 146, 155, 188, 233, 279, 315, 355, 386, 420, 430, 463, 507, 540, 578, 585, 521, 481, 458, 430, 400, 343, 243, 382, 55, 85, 135, 256, 365, 448, 706, 674, 705, 786, 798, 753, 721, 752, 738, 922, 1001, 1108, 1093, 951, 852, 868, 963, 1128, 1001, 1151, 1071, 613, 468, 638, 728, 418, 328]
y_pts = [14, 405, 555, 595, 520, 378, 308, 348, 510, 406, 397, 238, 126, 49, 27, 157, 162, 76, 106, 60, 24, 17, 45, 227, 65, 25, 87, 139, 215, 73, 27, 35, 144, 79, 22, 401, 390, 557, 672, 642, 644, 71, 35, 119, 218, 305, 379, 348, 377, 305, 186, 114, 39, 19, 27, 62, 14, 12, 50, 17, 22, 19, 32, 60, 37, 18, 255, 335, 417, 419, 392, 421, 456, 394, 349, 775, 714, 624, 667, 320, 795, 708, 662, 614, 660, 722, 1207, 1172, 1151, 1105, 1065, 1017, 999, 971, 959, 965, 979, 1004, 1038, 1087, 1119, 1165, 1225, 1244, 1261, 1280, 1245, 1232, 1215, 1165, 1127, 1080, 1063, 1194, 1192, 1094, 985, 932, 963, 1062, 1182, 1110, 1084, 1093, 1154, 1179, 1140, 1132, 1110, 915, 898, 914, 1015, 1013, 973, 929, 844, 864, 960, 970, 955, 928, 888, 848, 917, 785, 699]
x_pts = y_pts = []

ADD_NOISE = False       # include additional noise?
SAMPLES = 110           # Number of additional random samples used to generate heat map and terrain profile

#### PATH PLANNING PARAMS
# note that min index value is 0 and max is "XDIMENSION - corner size"...
START = (50,320)				# index value which agent starts at after including corner size (row, col)
END   = (345,220)	            # index value which agent ends at after including corner size, set to None for ending at top, right corner (row, col)
                            # START = (50,320)
                            # Colorado1, Louisiana1 END = (455,420)
                            # Colorado2 END = (345,220)
IOP_ORDER        = 1        # Choose an odd number e.g. [1,3,5,7,9...]. This number changes the order of the equation
                            # i.e. 1 = 1st Order Equation with a linearly changing velocity vs IOP value. 3 = 3rd order
                            # which gives a cubic shape. All have min = (0,-1) and max = (1, MAX_VELOCITY)
RSQ_THRESHOLD    = 0.9999   # R-Squared value for determining waypoints (lower val ∝ less waypoints)
USE_WAYPOINTS    = True     # Option to use fewer waypoints on route to minimise route complexity (blue dots on plots)
SHOW_WAYPOINTS   = False    # Show vehicle waypoints which will be used in Webots simulator?

OBSTACLE_PADDING = True     # If true, use padding if vehicle is larger than tile size, else, no padding necessary
INCREASE_PADDING = 1        # Increase kernel dilate size by X e.g. X=1, kernel = [n+1, m+1]... If == 0, no padding
                            # is applied unless the vehicle is larger than the tile size.
SHOW_PADDING     = False    # During runtime, pause and show user before and after dilation of image mask (showing obstacle regions with padding)

SHOW_OUTOFBOUNDS = True     # If true, then mark areas which are 'No Go' on Slope and passability maps i.e. block out in white
SHOW_LABELS      = True     # If true then add graph labels to axes

#######################################################################

class Terrain( Tile, LandTypes, DEM ):
    """Terrain class holds all functions relating to terrain generation."""
    def __init__( self, DEMpath : str = None, PNGpath : str = None ):
        # initialise inherited land type class
        landtypes = LandTypes.__init__( self, vrf = vehicleInfo["vrf"] )

        # initialise inherited digital elevation map class with location of TIFF file
        if USEDEM:
            DEM.__init__( self, impath = DEMpath,
                                shape = (YDIMENSION, XDIMENSION)    )

        # store paths to allow for class-wide access
        self.DEMpath = DEMpath
        self.PNGpath = PNGpath

        # make a 2D array of tile objects, one for each node which contains
        # terrain "traits" or "attributes"...
        self.tiles = np.zeros( (YDIMENSION-1, XDIMENSION-1), dtype=object )
        for iy, ix in np.ndindex( self.tiles.shape ):
            self.tiles[ iy, ix ] = Tile( landObj = landtypes )  # pass the initialised landtypes object

        # prepare to store values, create variable names
        x_grid = np.arange(0,XDIMENSION*XSPACING,XSPACING)
        y_grid = np.arange(0,YDIMENSION*YSPACING,YSPACING)
        self.x_mesh, self.y_mesh = np.meshgrid(x_grid,y_grid)
        self.elevationCorners = np.ndarray
        self.slopeCorners = np.ndarray
        self.image = np.ndarray

    def kde_quartic( self, d, H ):
        """Function to calculate intensity with quartic kernel."""
        return (15/16)*(1-(d/H)**2)**2

    def elevation_map( self, xs : list, ys : list ):
        """Take input points and return an intensity map as a 2D numpy array."""
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
        elevationNodes = cv2.resize(self.elevationCorners.astype('float32'),
                                    (YDIMENSION-1, XDIMENSION-1), 
                                    interpolation = INTERP )  # interpolation method!
        for iy, ix in np.ndindex( elevationNodes.shape ):
            self.tiles[ iy, ix ].elevation = elevationNodes[ iy, ix ]
        return (self.x_mesh, self.y_mesh), self.elevationCorners

    def dem_to_tile( self ):
        """Get real DEM data and assign segments to corresponding tile class elevation attributes."""
        # resize shape to shape-1 via interpolation to get corresponding values for tiles (not corners!)
        demHeights = self.resizeDEM( shape = (YDIMENSION-1, XDIMENSION-1),  # shape to resize to
                                     interp = INTERP )                      # interpolation method
        for iy, ix in np.ndindex( demHeights.shape ):
            self.tiles[ iy, ix ].elevation = demHeights[ iy, ix ]

        # return elevation corners of shape (YDIMENSION, XDIMENSION) i.e. not tiles!
        return self.imgNpy

    def wbo_map( self, elevationCorners : np.ndarray ):
        """Create .wbo WeBots readable terrain map using intensity map 2D numpy array."""

        # If USEDEM is true, get DEM heights, else convert passed generated elevation data --> convert into .wbo format
        heights =  self.wb_heights() if USEDEM else ",".join( [",".join(item) for item in np.round(elevationCorners,2).astype(str)] )

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
        if SAVEMAP:
            try:
                with open('maps/WEBOTS_elevations.wbo', 'w') as f:
                    f.write( formatted )
                    f.close()
            except:
                raise ValueError('"WEBOTS_elevations.wbo" did not save!')

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
        slopeNodes = cv2.resize(self.slopeCorners.astype('float32'),
                                (YDIMENSION-1, XDIMENSION-1), 
                                interpolation = INTERP )  # interpolation method!
        for iy, ix in np.ndindex( slopeNodes.shape ):
            self.tiles[ iy, ix ].slope = slopeNodes[ iy, ix ]
        return self.slopeCorners

    def image_map( self, pixelRes : int = 2048, show : bool = False ):
        """Divide terrain image into same number of grid squares as other terrain features."""
        # Load an color image in BGR, reformat and get key params
        self.image = cv2.imread( self.PNGpath, cv2.IMREAD_UNCHANGED )
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
        if show:
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

                # If slope type then append None placeholder, we handle this differently. Here,
                # we use a block sliding down a ramp model, where the steeper the angle, the
                # greater the opposing force required to continue moving up the ramp. As slope
                # is in range ±90 deg (but we assume 0 to 90), this means slope IOP is in range
                # 0 to 1, with a sinusoidal curve profile
                if feature == "Slope": tile.iop += math.sin(tile.slope) * vrf   # updating iop total 
                else:
                    image_hsv = cv2.cvtColor( tile.image, cv2.COLOR_BGR2HSV )   # get hsv of tile image
                    lower, upper = self.get_colour_range( feature )             # get colour range
                    image_mask = cv2.inRange( image_hsv, lower, upper )         # create mask
                    tile.iop += getPctArea(image_mask) * vrf                    # updating iop total

            # Set IOP values to be in range -1 < x < +1
            if tile.iop > 1: tile.iop = 1
            elif tile.iop < -1: tile.iop = -1

            # calculate estimated velocity at tile using IOP in km/h
            tile.velocity = 0.5 * MAX_VELOCITY * ( 1 + (tile.iop)**IOP_ORDER )

        # get all tile velocities and put into list
        iops = [obj.iop for row in self.tiles for obj in row]
        velocities = [obj.velocity for row in self.tiles for obj in row]

        # scaled = 2.*(iops - np.min(iops))/np.ptp(iops)-1
        return np.array(iops).reshape(-1, XDIMENSION-1), np.array(velocities).reshape(-1, XDIMENSION-1)   # return 2D array of iops

    def pad_obstacles( self, showDilate : bool = False ):
        """Depending on vehicle size in relation to tile size, pad obstacles to avoid collisions."""
        slopeNodes = np.zeros( self.tiles.shape )
        for iy, ix in np.ndindex( self.tiles.shape ):
            # create mask of passable and impassable areas (0 = passable, 1 = impassable)
            if self.tiles[ iy, ix ].isobstacle(max_slope = MAX_SLOPE_ANGLE):
                slopeNodes[ iy, ix ] = 1

        # user to decide whether padding should be applied
        if OBSTACLE_PADDING:
            # determine kernel size based on vehicle params, if <= 1, skip this step due to
            # tile size being larger than vehicle max length.
            tiles_to_cover = math.ceil( VEHICLE_LENGTH / ((XSPACING+YSPACING)/2.0) ) + INCREASE_PADDING
            if tiles_to_cover > 1.:
                # dilate numpy 2d array by increasing the 1s i.e. obstacles boundaries
                kernel = np.ones((tiles_to_cover,tiles_to_cover), np.uint8)
                img_dilation = cv2.dilate(slopeNodes, kernel, iterations=1)

                for iy, ix in np.ndindex( img_dilation.shape ):
                    # if dilated image == 1 i.e. obstacle, set var to True, else False
                    self.tiles[ iy, ix ].obstacle = True if img_dilation[ iy, ix ] == 1 else False

                if showDilate:
                    cv2.imshow('Source', slopeNodes)
                    cv2.imshow('Dilated', img_dilation)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                return img_dilation # return dilated obstacle mask        
        
        # if reached here, then just return the original image mask
        print(' No Padding Applied...', end='')
        return slopeNodes   # obstacle mask

def isPowerOfTwo( n ):
    """Function to check if x is power of 2."""
    if (n == 0): return False
    while (n != 1):
        if (n % 2 != 0): return False
        n = n // 2
    return True

def getFilesFromFolder( path : str = None, tiff : str = None, png : str = None ):
    """Get TIFF and PNG files from folder and return their directories."""
    try:
        filenames = next(walk(path), (None, None, []))[2]  # [] if no file
        for file in filenames:
            # get file extension and make lower case
            extension = (file.split(".")[1]).lower()

            # if tiff and png files are found, assign them unless user has pre-declared it
            if extension == 'tiff' and not tiff: tiff = file
            elif extension == 'png' and not png: png = file
        # return the full path by joining folder path and file names
        return '/'.join((path, tiff)), '/'.join((path, png))

    except:
        raise ValueError("""\n\n[ERROR]: Must provide a valid folder directory to access TIFF 
         and PNG files. Check that the provided path is a valid directory and that both files
         are contained within it.\n\n""")

def get_xys( route ):
    xs = [coord[0] + 1.5*XSPACING for coord in route]
    ys = [coord[1] + 1.5*YSPACING for coord in route]
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
    trnWebots = trn.get_RGB_WeBots( maxvel = MAX_VELOCITY, iop_order = IOP_ORDER )

    # Colour of terrain classes – output in the form {terrain_type,r,g,b,vrf},{terrain_type,r,g,b,vrf}...
    # shift Firebreak colour by a number because blacks do not appear as dark with textures in Webots
    classes_str = ",".join( ["{{{},{},{},{},{}}}".format(   i[0],                                       # terrain class
                                                            i[1][0][0] if i[0] != 'Firebreak' else 50,  # red
                                                            i[1][0][1] if i[0] != 'Firebreak' else 50,  # green
                                                            i[1][0][2] if i[0] != 'Firebreak' else 50,  # blue
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
        with open('maps/WEBOTS_vehicle_config.txt', "w") as f:
            f.write( formatted )
            f.close()
    except:
        raise ValueError('"maps/WEBOTS_vehicle_config.txt" did not save!')

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

def velocity_per_route( route : list, trn : Terrain ):
    """Return the iop values at each stage of a route as list."""
    return [trn.tiles[r][c].velocity for r, c in route]

def apply_padding_mask( source : np.ndarray,  mask : np.ndarray, min = None, max = None):
    """Set all obstacle values (including padding from mask) to larger than max slope."""
    for iy, ix in np.ndindex( source.shape ):
        # if obstacle, then make slope > max slope
        if mask[ iy, ix ] == 1:
            if min != None: source[ iy, ix ] = min - 1
            elif max != None: source[ iy, ix ] = max + 1
    return source

# Main processing
if __name__ == '__main__':
    # time script execution
    starttime = time.time()

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
    
    # Get TIFF and PNG files from user defined folder
    print("[INFO]: Accessing TIFF and PNG Files From Folder...", end = '')
    start = time.time()
    tiff, png = getFilesFromFolder( path = FOLDERPATH )
    print( f" {round((time.time() - start), 2)} s")

    # Initialise terrain class 
    print("[INFO]: Initialising Script, Preparing Variables and Classes...", end = '')
    start = time.time()
    terrain = Terrain( DEMpath = tiff, PNGpath = png )
    print( f" {round((time.time() - start), 2)} s")

    # Either Create intensity 2D numpy array using user defined params or get DEM data
    if USEDEM:
        print("[INFO]: Getting Digital Elevation Map (DEM) data...", end = '')
        start = time.time()
        elevationCorners = terrain.dem_to_tile()
        print( f" {round((time.time() - start), 2)} s")
    else:
        if ADD_NOISE:
            samples = SAMPLES if SAMPLES <= np.mean([XDIMENSION, YDIMENSION]) else int(XDIMENSION/2)
            x_pts.extend( random.sample( range(0, XDIMENSION*XSPACING), samples) )
            y_pts.extend( random.sample( range(0, YDIMENSION*YSPACING), samples) )
            
        print("[INFO]: Creating Elevation Map...", end = '')
        start = time.time()
        (_,_), elevationCorners = terrain.elevation_map( x_pts, y_pts )
        print( f" {round((time.time() - start), 2)} s")

    print("[INFO]: Dividing Terrain Image into Grid Squares...", end = '')
    start = time.time()
    _ = terrain.image_map( pixelRes = PIXEL_RESOLUTION, show = False )
    print( f" {round((time.time() - start), 2)} s")

    # Generate output .wbo file using 2D numpy array
    print("[INFO]: Creating WeBots Map...", end = '')
    start = time.time()
    terrain.wbo_map( elevationCorners )
    print( f" {round((time.time() - start), 2)} s")

    # Slope Map generation
    print("[INFO]: Calculating Slope Map...", end = '')
    start = time.time()
    slopeCorners = terrain.slope_map( elevationCorners )
    print( f" {round((time.time() - start), 2)} s")

    # Index of Passability generation
    print("[INFO]: Calculating Passability Map...", end = '')
    start = time.time()
    iop2dArr, vel2dArr = terrain.iop_map( )
    print( f" {round((time.time() - start), 2)} s")

    # Pad obstacles if vehicle size is larger than the grid square (tile) size
    print("[INFO]: Padding Obstacles According to Vehicle Dimensions...", end = '')
    start = time.time()
    padded_obstacle_mask = terrain.pad_obstacles( showDilate = SHOW_PADDING )
    print( f" {round((time.time() - start), 2)} s")

    try:
        # create empty lists to overwrite if paths are found
        x_rt_greedy = y_rt_greedy = dists_greedy = elevs_greedy = velocity_greedy= []
        x_rt_astar = y_rt_astar  = dists_astar = elevs_astar = velocity_astar = []
        x_rt_dijkstra = y_rt_dijkstra  = dists_dijkstra = elevs_dijkstra = velocity_dijkstra = []

        # add starting and ending positions now so that they are on map even if
        # no route is found with path planners 
        shift = lambda n : (n*YSPACING) + 1.5*YSPACING
        y_wpts_greedy, x_wpts_greedy  = ( (shift(START[0]), shift(END[0])),  (shift(START[1]), shift(END[1])) )
        y_wpts_astar , x_wpts_astar   = ( (shift(START[0]), shift(END[0])),  (shift(START[1]), shift(END[1])) )
        y_wpts_dijkstra , x_wpts_dijkstra   = ( (shift(START[0]), shift(END[0])),  (shift(START[1]), shift(END[1])) )

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

        solutionRoute_dijkstraIndex = dijkstraRoute3D(clip(terrain.tiles),            # terrain tile classes 2D array
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
            solutionRoute_dijkstra  = transform_to_webots( solutionRoute_dijkstraIndex  )

            # Make set of waypoints for vehicle based on solution
            # Returns as [ (x1,y1), (x2,y2) ... ]
            waypoints_greedy = get_waypoints( route = solutionRoute_greedy )
            waypoints_astar = get_waypoints( route = solutionRoute_astar )
            waypoints_dijkstra = get_waypoints( route = solutionRoute_dijkstra )

            # get elevation data per distance travelled to plot later
            dists_greedy, elevs_greedy = elevation_per_distance(  solutionRoute_greedy, trn = terrain)
            dists_astar, elevs_astar = elevation_per_distance(  solutionRoute_astar, trn = terrain)
            dists_dijkstra, elevs_dijkstra = elevation_per_distance(  solutionRoute_dijkstra, trn = terrain)

            # get iop data per waypoint travelled to plot later
            velocity_greedy = velocity_per_route(  solutionRoute_greedyIndex, trn = terrain)
            velocity_astar = velocity_per_route(  solutionRoute_astarIndex, trn = terrain)
            velocity_dijkstra = velocity_per_route(  solutionRoute_dijkstraIndex, trn = terrain)

            # Save waypoints and starting location for vehicle in config file (readable by Webots C code)
            wbo_vehicle_config( wpts = waypoints_astar if USE_WAYPOINTS else solutionRoute_astar,
                                trn = terrain )

            ############################### CREATE FIGURES BELOW ###############################
            # reformat data to matplotlib readable version
            x_rt_greedy, y_rt_greedy = get_xys( solutionRoute_greedy )
            x_wpts_greedy, y_wpts_greedy = get_xys( waypoints_greedy )

            x_rt_astar, y_rt_astar = get_xys( solutionRoute_astar )
            x_wpts_astar, y_wpts_astar = get_xys( waypoints_astar )

            x_rt_dijkstra, y_rt_dijkstra = get_xys( solutionRoute_dijkstra )
            x_wpts_dijkstra, y_wpts_dijkstra = get_xys( waypoints_dijkstra )

    except: pass # continue silently after an no route found edge case error...

    # Consider end time before plotting
    endtime = round((time.time() - starttime), 2)
    print(f"[INFO]: Algorithm Finished After: {endtime} [s]!")

    ##### PLOTTING HEADER #####
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(nrows=2, ncols=2, figsize=(9.5,8.4))
    fig.suptitle('Terrain Heatmaps With Path Planning\n(Vehicle: {}, Terrain: {}, Slope-Max: {} deg, V-Max: {} Km/h)'.format(
                                                                    VEHICLE,
                                                                    (FOLDERPATH.split("/")[1]).upper(),
                                                                    round(MAX_SLOPE_ANGLE/math.pi * 180,2),
                                                                    MAX_VELOCITY), 
                                                                    fontsize=14)
    
    # add starting and ending positions now so that they are on map even if
    # no route is found with path planners 
    shift = lambda n : (n*YSPACING) + 1.5*YSPACING
    y_wpts_greedy, x_wpts_greedy  = ( (shift(START[0]), shift(END[0])),  (shift(START[1]), shift(END[1])) )
    y_wpts_astar , x_wpts_astar   = ( (shift(START[0]), shift(END[0])),  (shift(START[1]), shift(END[1])) )

    ##### ELEVATION HEATMAP OUTPUT #####
    ax1.set(title="Elevation Heatmap")
    # ax1.plot(x_pts,y_pts,'ro')
    ax1.axis(   xmin=0, xmax=(XDIMENSION-1)*XSPACING,
                ymin=0, ymax=(YDIMENSION-1)*YSPACING)
    elevationTiles = cv2.resize( elevationCorners, (YDIMENSION-1,XDIMENSION-1) )
    cbar0 = fig.colorbar(   ax1.pcolormesh( terrain.x_mesh,
                            terrain.y_mesh,
                            elevationTiles,
                            rasterized=True),
                        ax=ax1,)
   
    # ##### SLOPE HEATMAP OUTPUT #####
    ax2.set(title="Slope Heatmap")
    # plot Greedy
    ax2.plot(x_rt_greedy, y_rt_greedy,'g-')
    if SHOW_WAYPOINTS: ax2.plot(x_wpts_greedy, y_wpts_greedy,'ko', label='_nolegend_')

    # plot A* 
    ax2.plot(x_rt_astar, y_rt_astar,'r-')
    if SHOW_WAYPOINTS: ax2.plot(x_wpts_astar, y_wpts_astar,'bo', label='_nolegend_')

    # plot Start and End Points
    ax2.plot(shift(START[1]), shift(START[0]),'bo', label='_nolegend_')
    ax2.plot(shift(END[1]), shift(END[0]),'ro', label='_nolegend_')

    # plot axes and legend
    ax2.axis(   xmin=0, xmax=(XDIMENSION-1)*XSPACING,
                ymin=0, ymax=(YDIMENSION-1)*YSPACING)
    # ax2.legend(['Greedy', 'A*'])
    slopeTiles = cv2.resize( slopeCorners, (YDIMENSION-1,XDIMENSION-1) )
    cbar1 = fig.colorbar(   ax2.pcolormesh( terrain.x_mesh,
                                            terrain.y_mesh,
                                            slopeTiles,
                                            vmax= MAX_SLOPE_ANGLE,
                                            rasterized=True),
                            ax=ax2 )
    # get coverage of non-passable tiles (of total i.e. as %)
    r, c = slopeTiles.shape
    coverageObstacles = round(100*(slopeTiles > MAX_SLOPE_ANGLE).sum() / (r * c),2)
    print(f"[TEST INFO]: Slopes > Vehicle Max: {coverageObstacles} %")

    ##### PLOT TERRAIN CLASSES IMAGE IN RGB #####
    ax3.set(title="Terrain Classes Image")
    ax3.imshow( cv2.cvtColor(terrain.image, cv2.COLOR_BGR2RGB) )
    # plot empty data and show key of colours and respective classes
    [  ax3.plot(np.NaN, np.NaN, '-', color=terrain.get_mid_mtlb(key), label=key) 
                        for c, key in enumerate(terrain.get_type_keys(drop='Slope')) ]

    ##### VELOCITY HEATMAP OUTPUT #####
    ax4.set(title="Vehicle Velocity Heatmap")
    # plot Greedy
    ax4.plot(x_rt_greedy, y_rt_greedy,'g-')
    if SHOW_WAYPOINTS: ax4.plot(x_wpts_greedy, y_wpts_greedy,'ko', label='_nolegend_')
    
    # plot A* 
    ax4.plot(x_rt_astar, y_rt_astar,'r-')
    if SHOW_WAYPOINTS: ax4.plot(x_wpts_astar, y_wpts_astar,'bo', label='_nolegend_')

    # plot Start and End Points
    ax4.plot(shift(START[1]), shift(START[0]),'bo', label='_nolegend_')
    ax4.plot(shift(END[1]), shift(END[0]),'ro', label='_nolegend_')

    # plot axes and legend
    ax4.axis(   xmin=0, xmax=(XDIMENSION-1)*XSPACING,
                ymin=0, ymax=(YDIMENSION-1)*YSPACING)
    ax4.legend(['Greedy', 'A*'])
    # apply padding as per vehicle size vs tile size requirements
    vel2dArr = apply_padding_mask( source = vel2dArr, mask = padded_obstacle_mask, min = 0 )
    cbar2 = fig.colorbar(   ax4.pcolormesh( terrain.x_mesh,
                                            terrain.y_mesh,
                                            vel2dArr,
                                            vmin=0.001,
                                            rasterized=True ), 
                            ax=ax4 )
    # get coverage of non-passable tiles (of total i.e. as %)
    r, c = vel2dArr.shape
    coverageObstacles = round(100*(vel2dArr <= 0).sum() / (r * c),2)
    print(f"[TEST INFO]: Coverage of Non-Passable Terrain: {coverageObstacles} %")

    ##### GRAPH PLOTTING USER OPTIONS #####
    # Set thresholds for heatmaps i.e. slopes > vehicle slope max marked as white
    if SHOW_OUTOFBOUNDS:
        cbar1.cmap.set_over('white')
        cbar2.cmap.set_under('white')

    # Add labels to graphs
    if SHOW_LABELS:
        # plot 0,0
        ax1.set_xlabel('X Position [m]'); ax1.set_ylabel('Y Position [m]')
        cbar0.ax.set_ylabel('Elevation [m]', rotation=270, labelpad=15,)

        # plot 0,1
        ax2.set_xlabel('X Position [m]'); ax2.set_ylabel('Y Position [m]')
        cbar1.ax.set_ylabel('Slope [rad]', rotation=270, labelpad=15,)

        # plot 1,0
        ax3.set_xlabel('X Position [pixels]'); ax3.set_ylabel('Y Position [pixels]')
        
        # plot 1,1
        ax4.set_xlabel('X Position [m]'); ax4.set_ylabel('Y Position [m]')
        cbar2.ax.set_ylabel('Vehicle Velocity [km/h]', rotation=270, labelpad=15,)

    ##### ELEVATION AND IOP OVER DISTANCE OUTPUT #####
    fig2, (ax5, ax6) = plt.subplots(nrows=2, ncols=1, figsize=(9.5,8.))
    ax5.set(title="Elevation Change Over Distance Travelled")
    ax5.plot(dists_greedy, elevs_greedy,'g-')
    ax5.plot(dists_astar, elevs_astar,'r-')
    # ax5.axis(   xmin=0, xmax=1600, ymin=300, ymax=600)
    ax5.set_ylabel('Elevation [m]'); ax5.set_xlabel('Distance [m]')
    ax5.legend(['Greedy', 'A*'])

    ax6.set(title="Passability Change Over Distance Travelled")
    ax6.plot(dists_greedy, velocity_greedy,'g-')
    ax6.plot(dists_astar, velocity_astar,'r-') 
    # ax5.axis(   xmin=0, xmax=1600, ymin=300, ymax=600)
    ax6.set_ylabel('Velocity [km/h]'); ax5.set_xlabel('Distance [m]')
    ax6.legend(['Greedy', 'A*'])

    ##### SAVING RESULTANT PLOTS #####
    fig.tight_layout(pad=0.6, w_pad=0.2, h_pad=0.2)
    fig.savefig('results/passability.png')
    fig2.tight_layout(pad=0.6, w_pad=0.2, h_pad=1.2)
    fig2.savefig('results/elevation_gain.png')

    plt.show()
    ###################################################