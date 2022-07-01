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
import matplotlib.pyplot as plt
import numpy as np
import math
import random
import sys
import cv2

sys.path.append('./search')     # add 'search' directory to path
from greedysearch import greedyRoute3D
from astarsearch import astarRoute3D
from scipy import stats

############################## SETUP ###################################
#### WEBOTS VEHICLE PROPERTIES
MAX_SLOPE_ANGLE = 0.65      # Maximum permissible slope angle for vehicle in radians
VEHICLE_LENGTH = 2.964      # Vehicle length in meters
VEHICLE_HEIGHT = 1.145      # Vehicle height in meters
RSQ_THRESHOLD = 0.999999    # R-Squared value for determining waypoints (lower val ∝ less waypoints)

#### WEBOTS ELEVATION MAP PARAMS
XDIMENSION = 128    # Max number of nodes in x dir (MUST BE A POWER OF 2!)
YDIMENSION = 128    # Max number of nodes in y dir (MUST BE A POWER OF 2!)

XSPACING = YSPACING = 1    # The spacing between nodes in x, y dir [meters]
CORNER_SIZE = 2            # Number of corners to ignore for path planning (to not fall off edge of map)

XTRANSLATE = -(XDIMENSION-1)*XSPACING / 2.  # Offset for terrain in x dir
YTRANSLATE = -(YDIMENSION-1)*YSPACING / 2.  # Offset for terrain in y dir
ZTRANSLATE = 0                              # Offset for terrain in z dir

USE_WAYPOINTS = False    # Option to use fewer waypoints on route to minimise route complexity (blue dots on plots)

APPEARANCE = "TerrainFeatures"      # e.g. "SandyGround" with SCALE = 10, e.g. "CustomAppearance" with SCALE = 1 (see proto files)
SCALE = 1                           # Scale of appearance image over texture (in WeBots simulator)
PIXEL_RESOLUTION = 2048             # Pixel resolution of terrain feature image (MUST BE A POWER OF 2!)

#### KERNEL DENSITY ESTIMATOR PARAMS
H = 10    # Radius (h) defines how much affect each point has to KDE (higher H is more reach)

x_pts = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,75,75,75,75,80,80,80,80,80,80,80,80,45,45,45,45,45,45,45,45]  # seed x points for elevation locations
y_pts = [10,10,10,20,20,20,30,30,30,40,40,40,50,50,50,85,75,80,80,80,92,35,35,35,35,35,35,35,35,76,67,67,32,45,67]  # seed y points for elevation locations

ADD_NOISE = False   # include additional noise?
SAMPLES = 85       # Number of additional random samples used to generate heat map and terrain profile

#######################################################################

class Terrain:
    """Terrain class holds all functions relating to terrain generation."""
    def __init__( self, *args, **kwargs ):
        self.x_mesh = list
        self.y_mesh = list
        self.elevationMap = np.ndarray
        self.slopeMap = np.ndarray
        self.image = np.ndarray
        self.imageMap = np.ndarray
        self.larea = np.ndarray
        self.iop = np.array  

    def kde_quartic( self, d, H ):
        """Function to calculate intensity with quartic kernel."""
        dn=d/H
        return (15/16)*(1-dn**2)**2

    def elevation_map( self, xs : list, ys : list ):
        """Take input points and return an intensity map as a 2D numpy array."""
        # Construct grid
        x_grid = np.arange(0,XDIMENSION*XSPACING,XSPACING)
        y_grid = np.arange(0,YDIMENSION*YSPACING,YSPACING)
        self.x_mesh, self.y_mesh = np.meshgrid(x_grid,y_grid)

        # Grid center points
        xc=self.x_mesh+(XSPACING/2)
        yc=self.y_mesh+(YSPACING/2)

        self.elevationMap = np.full( (XDIMENSION, YDIMENSION), 0.0 )
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
                self.elevationMap[j][k] = p_total
        
        # Turn intensity list into numpy array and then set border heights to 0
        self.elevationMap[0,:-1] = self.elevationMap[:-1,-1] = self.elevationMap[:-1,0] = self.elevationMap[-1,:-1] = 0
        return (self.x_mesh, self.y_mesh), self.elevationMap

    def wbo_map( self, elevationMap : np.ndarray ):
        """Create .wbo WeBots readable terrain map using intensity map 2D numpy array."""

        # convert numpy array into string format usable by .wbo file format
        heights = ",".join( [",".join(item) for item in np.round(elevationMap,2).astype(str)] )

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

    def slope_map( self, elev : np.ndarray ):
        """Calculate the slope map using previously generated terrain elevation data."""
        rows, cols = elev.shape
        self.slopeMap = np.zeros([rows, cols])
        elev = np.pad(elev, 1, mode='symmetric')   # add padding for kernel
        for i in range(1,rows+1):
            for j in range(1,cols+1):
                slope_we = ((elev[i+1][j-1] + 2*elev[i][j-1] + elev[i-1][j-1]) -    \
                            (elev[i+1][j+1] + 2*elev[i][j+1] + elev[i-1][j+1]))/    \
                            8 * XSPACING

                slope_sn = ((elev[i+1][j+1] + 2*elev[i+1][j] + elev[i+1][j-1]) -    \
                            (elev[i-1][j+1] + 2*elev[i-1][j] + elev[i-1][j-1]))/    \
                            8 * YSPACING

                self.slopeMap[i-1][j-1] = np.arctan( math.sqrt(slope_we**2 + slope_sn**2) )
        return self.slopeMap

    def image_map( self, imageDir = "webots_moose/protos/textures/TerrainFeatures.png", pixelRes = 2048, check = False ):
        """Divide terrain image into same number of grid squares as other terrain features."""
        # Load an color image in BGR, reformat and get key params
        self.image = cv2.imread( imageDir, cv2.IMREAD_UNCHANGED )
        self.image = cv2.resize( self.image, (pixelRes, pixelRes), interpolation = cv2.INTER_AREA )
        row, col, _ = self.image.shape
        
        # add alpha channel to image so that WeBots doesn't perform image interpolation
        rgba = cv2.cvtColor( self.image, cv2.COLOR_RGB2RGBA )

        # resave image to correct dimensions for WeBots simulator
        cv2.imwrite("webots_moose/protos/textures/TerrainFeaturesScaled.png", rgba)
        
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
        # imgMap is a 2D array with the corresponding image section contained within
        # e.g. imgMap[0][0] would contain an image of size MxN pixels
        self.imageMap = np.zeros( (YDIMENSION, XDIMENSION) , dtype=object)
        for rn, r in enumerate(range(row,0,-M)):
            for cn, c in enumerate(range(0,col,N)):
                self.imageMap[rn][cn] = self.image[r-M:r, c:c+N]

        # check segmentation has worked...
        if check:
            cv2.imshow( 'Main', self.image )
            for row in self.imageMap:
                for col in row:
                    cv2.imshow( 'Tile', col )
                    cv2.waitKey(0)
                    cv2.destroyWindow("Tile")
            cv2.destroyAllWindows()
        return self.imageMap

    def iop_map( self, params = [] ):
        """Calculate coverage areas of terrain features..."""



        pass

    def wbo_vehicle_config( self, elev : np.ndarray, wpts : np.ndarray ):
        """Save key Webots startup information in text file for C code."""

        # calculate translation values for vehicle (WeBots coordinate system)
        tx = str(wpts[0][0] + XTRANSLATE + XSPACING)
        ty = str(wpts[0][1] + YTRANSLATE + YSPACING)
        tz = str(elev[CORNER_SIZE][CORNER_SIZE] + VEHICLE_HEIGHT/2)

        # put all waypoints into string and adjust for elevation map offset in WeBots
        index = lambda el : int((el / XSPACING) - (CORNER_SIZE-1)) + CORNER_SIZE
        wpts_string = ",".join( ['{{{},{},{}}}'.format( str(el[0] + XTRANSLATE + XSPACING),
                                                        str(el[1] + YTRANSLATE + YSPACING),
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

    def get_features( self, r : int, c : int ):
        """Return a dictionary of grid features for the given row and column."""
        keys = [ "elevation", "slope", "image", "larea", "iop" ]
        values = [  self.elevationMap[r][c],
                    self.slopeMap[r][c],
                    self.imageMap[r][c], 
                    [],
                    0                       ]
        return { k : v for k, v in zip(keys, values) }


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
    (x_mesh, y_mesh), intensity2DArr = terrain.elevation_map( x_pts, y_pts )

    print("[INFO]: Dividing Terrain Image into Grid Squares...")
    _ = terrain.image_map( pixelRes = PIXEL_RESOLUTION )

    # Generate output .wbo file using 2D numpy array
    print("[INFO]: Creating WeBots Map...")
    terrain.wbo_map( intensity2DArr )

    # Slope Map generation
    print("[INFO]: Calculating Slope Map...")
    slope = terrain.slope_map( intensity2DArr )

    # Get possible route using Greedy search approach as list [(x1,y1), (x2,y2) ...]
    # Don't pass border values as their slopes are not accurate due to kerneling method. 
    # Answers are returned as INDEX VALUES OF THE INPUT ARRAY!
    clip = lambda array : array[CORNER_SIZE:-CORNER_SIZE, CORNER_SIZE:-CORNER_SIZE]
    solutionRoute_greedyIndex = greedyRoute3D(  clip(intensity2DArr),     # map_array of heights
                                                clip(slope),              # array of slope angles
                                                maxslope=MAX_SLOPE_ANGLE,       # max permissible slope angles
                                                gridsize=(XSPACING,YSPACING) )  # size of each grid segment in [m]

    solutionRoute_astarIndex = astarRoute3D(clip(intensity2DArr),     # map_array of heights
                                            clip(slope),              # array of slope angles
                                            maxslope=MAX_SLOPE_ANGLE,       # max permissible slope angles
                                            gridsize=(XSPACING,YSPACING) )  # size of each grid segment in [m]

    # if a path exists then continue...
    if solutionRoute_greedyIndex and solutionRoute_astarIndex:
        # shift results to account for lambda border clipping step shown above. Also modify
        # results to show solution in absolute coordinates rather than index values of the input
        solutionRoute_greedy = (np.array(solutionRoute_greedyIndex) + (CORNER_SIZE-1)) * XSPACING
        solutionRoute_astar  = (np.array(solutionRoute_astarIndex ) + (CORNER_SIZE-1)) * XSPACING

        # Make set of waypoints for vehicle based on solution
        # Returns as [ (x1,y1), (x2,y2) ... ]
        waypoints_greedy = get_waypoints( route = solutionRoute_greedy )
        waypoints_astar = get_waypoints( route = solutionRoute_astar )

        # Save waypoints and starting location for vehicle in config file (readable by Webots C code)
        terrain.wbo_vehicle_config( elev = intensity2DArr, wpts = waypoints_astar if USE_WAYPOINTS else solutionRoute_astar )

        ################# CREATE FIGURES #################
        # reformat data to matplotlib readable version
        x_rt_greedy, y_rt_greedy = get_xys( solutionRoute_greedy )
        x_wpts_greedy, y_wpts_greedy = get_xys( waypoints_greedy )

        x_rt_astar, y_rt_astar = get_xys( solutionRoute_astar )
        x_wpts_astar, y_wpts_astar = get_xys( waypoints_astar )

        # HEADER
        fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
        fig.suptitle(f'Elevation and Slope Heatmaps With Path Planning\n(Vehicle Max Slope: {MAX_SLOPE_ANGLE} [rad])', fontsize=16)
        
        # ELEVATION HEATMAP OUTPUT
        ax1.set(title="Elevation Heatmap")
        ax1.plot(x_pts,y_pts,'ro')
        ax1.axis(   xmin=-XSPACING/2, xmax=XDIMENSION*XSPACING-XSPACING/2,
                    ymin=-YSPACING/2, ymax=YDIMENSION*YSPACING-YSPACING/2)
        fig.colorbar( ax1.pcolormesh(x_mesh,y_mesh,intensity2DArr), ax=ax1 )

        # SLOPE HEATMAP OUTPUT
        ax2.set(title="Slope Heatmap")

        # plot Greedy
        ax2.plot(x_rt_greedy, y_rt_greedy,'g-')
        ax2.plot(x_wpts_greedy, y_wpts_greedy,'ko', label='_nolegend_')
        # plot A* 
        ax2.plot(x_rt_astar, y_rt_astar,'r-')
        ax2.plot(x_wpts_astar, y_wpts_astar,'bo', label='_nolegend_')

        # plot axes and legend
        ax1.axis(   xmin=-XSPACING/2, xmax=XDIMENSION*XSPACING-XSPACING/2,
                    ymin=-YSPACING/2, ymax=YDIMENSION*YSPACING-YSPACING/2)
        ax2.legend(['Greedy', 'A*'])
        fig.colorbar( ax2.pcolormesh(x_mesh, y_mesh, slope), ax=ax2 )
        plt.show()
        ###################################################