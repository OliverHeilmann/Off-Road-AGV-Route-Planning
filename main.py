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
sys.path.append('./search')     # add 'search' directory to path
from greedysearch import greedyRoute3D
from astarsearch import astarRoute3D
from scipy import stats

############################## SETUP ###################################
#### WEBOTS VEHICLE PROPERTIES
MAX_SLOPE_ANGLE = 0.19      # Maximum permissible slope angle for vehicle as ratio of Rise/Run
VEHICLE_LENGTH = 2.964      # Vehicle length in meters
VEHICLE_HEIGHT = 1.145      # Vehicle height in meters
RSQ_THRESHOLD = 0.999999    # R-Squared value for determining waypoints (lower val ∝ less waypoints)

#### WEBOTS ELEVATION MAP PARAMS
XDIMENSION = 100    # Max number of nodes in x dir
YDIMENSION = 100    # Max number of nodes in y dir

XSPACING = YSPACING = 1    # The spacing between nodes in x, y dir [meters]
CORNER_SIZE = 1             # Number of corners to ignore for path planning (to not fall off edge of map)

XTRANSLATE = -round(XDIMENSION*XSPACING / 2.)   # Offset for terrain in x dir
YTRANSLATE = -round(YDIMENSION*YSPACING / 2.)   # Offset for terrain in y dir
ZTRANSLATE = 0                                  # Offset for terrain in z dir

USE_WAYPOINTS = True    # Option to use fewer waypoints on route to minimise route complexity (blue dots on plots)

SCALE = 10  # Scale of appearance image over texture (in WeBots simulator)

#### KERNEL DENSITY ESTIMATOR PARAMS
H = 8    # Radius (h) defines how much affect each point has to KDE (higher H is more reach)

x_pts = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,75,75,75,75,80,80,80,80,80,80,80,80,45,45,45,45,45,45,45,45]  # seed x points for elevation locations
y_pts = [10,10,10,20,20,20,30,30,30,40,40,40,50,50,50,85,75,80,80,80,92,35,35,35,35,35,35,35,35,76,67,67,32,45,67]  # seed y points for elevation locations

# x_pts = [125, 125, 125, 125, 125, 125, 125, 125, 125,125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,125, 125,\
#         125, 125, 125, 125, 125, 125, 125, 125, 125,125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,125, 125] # seed x points for elevation locations
# y_pts = [115, 115, 115, 115, 115, 115, 130, 130, 130, 130, 130, 130, 145, 145, 145, 145, 145, 145, 155, 155, 155, 155,\
#         165, 165, 165, 165, 165, 190, 190, 190, 190, 190, 190, 190, 190, 190, 200, 200, 200, 200, 200, 200, 200, 200] # seed y points for elevation locations


ADD_NOISE = True   # include additional noise?
SAMPLES = 99       # Number of additional random samples used to generate heat map and terrain profile

#######################################################################

def kde_quartic(d,H):
    """Function to calculate intensity with quartic kernel."""
    dn=d/H
    P=(15/16)*(1-dn**2)**2
    return P

def intensity_map( xs : list, ys : list ):
    """Take input points and return an intensity map as a 2D numpy array."""
    # Construct grid
    x_grid=np.arange(0,XDIMENSION*XSPACING,XSPACING)
    y_grid=np.arange(0,YDIMENSION*YSPACING,YSPACING)
    x_mesh,y_mesh=np.meshgrid(x_grid,y_grid)

    # Grid center points
    xc=x_mesh+(XSPACING/2)
    yc=y_mesh+(YSPACING/2)

    intensity_list=[]
    for j in range(len(xc)):
        intensity_row=[]
        for k in range(len(xc[0])):
            kde_value_list=[]
            for i in range(len(xs)):
                # Calculate distance
                d=math.sqrt((xc[j][k]-xs[i])**2+(yc[j][k]-ys[i])**2) 
                if d<=H:
                    p=kde_quartic(d,H)
                else:
                    p=0
                kde_value_list.append(p)
            # Sum all intensity values
            p_total=sum(kde_value_list)
            intensity_row.append(p_total)
        intensity_list.append(intensity_row)
    
    # Turn intensity list into numpy array and then set border heights to 0
    intensityArr = np.array(intensity_list) 
    intensityArr[0,:-1] = intensityArr[:-1,-1] = intensityArr[:-1,0] = intensityArr[-1,:-1] = 0
    return (x_mesh, y_mesh), intensityArr

def wbo_map( intensityMap : np.ndarray ):
    """Create .wbo WeBots readable terrain map using intensity map 2D numpy array."""

    # convert numpy array into string format usable by .wbo file format
    heights = ",".join( [",".join(item) for item in np.round(intensityMap,2).astype(str)] )

    # structure of .wbo file
    formatted =  """#VRML_OBJ R2022a utf8
DEF TERRAIN Solid {{
    translation {} {} {}
    children [
        Shape {{
            appearance SandyGround {{
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
}}  """.format( XTRANSLATE,
                YTRANSLATE,
                ZTRANSLATE,
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

def slope_map( elev : np.ndarray ):
    """Calculate the slope map using previously generated terrain elevation data."""
    rows, cols = elev.shape
    slope = np.zeros([rows, cols])
    elev = np.pad(elev, 1, mode='symmetric')   # add padding for kernel
    for i in range(1,rows+1):
        for j in range(1,cols+1):
            slope_we = ((elev[i+1][j-1] + 2*elev[i][j-1] + elev[i-1][j-1]) -    \
                        (elev[i+1][j+1] + 2*elev[i][j+1] + elev[i-1][j+1]))/    \
                        8 * XSPACING

            slope_sn = ((elev[i+1][j+1] + 2*elev[i+1][j] + elev[i+1][j-1]) -    \
                        (elev[i-1][j+1] + 2*elev[i-1][j] + elev[i-1][j-1]))/    \
                        8 * YSPACING

            slope[i-1][j-1] = np.arctan( math.sqrt(slope_we**2 + slope_sn**2) )
    return slope

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

def wbo_vehicle_config( heightArr : np.ndarray, points : np.ndarray ):
    """Save key Webots startup information in text file for C code."""

    # calculate translation values
    tx = str(points[0][0] - ((XDIMENSION*XSPACING)/2) + XSPACING)
    ty = str(points[0][1] - ((YDIMENSION*YSPACING)/2) + YSPACING)
    tz = str(heightArr[int(points[0][0]/XSPACING)][int(points[0][1]/YSPACING)] + VEHICLE_HEIGHT/2)

    # put all waypoints into string and adjust for elevation map offset in WeBots
    wpts_string = ",".join( ['{{{},{},{}}}'.format( str(el[0] - ((XDIMENSION*XSPACING)/2) + XSPACING),
                                                    str(el[1] - ((YDIMENSION*YSPACING)/2) + YSPACING),
                                                    str(round(heightArr[int(el[1]/XSPACING)][int(el[0]/YSPACING)] + VEHICLE_HEIGHT/2,4)))
                                                    for el in points] )

    # structure of .wbo file
    formatted =  """Vehicle Config File
Vehicle {{
    translation {} {} {}
    rotation {} {} {} {}
    count {}
    waypoints {}
}}""".format(   tx, ty, tz,
                0, 0, -1, -0.85,
                str(len(points)),
                wpts_string,
            )
    # Save waypoints as text file usable in WeBots
    try:
        with open('maps/elevationmap_vehicle_config.txt', "w") as f:
            f.write( formatted )
            f.close()
    except:
        raise ValueError('"maps/elevationmap_vehicle_config.txt" did not save!')

def get_xys( route ):
    xs = [coord[0]+XSPACING for coord in route]
    ys = [coord[1]+YSPACING for coord in route]
    return xs, ys


# Main processing
if __name__ == '__main__':
    if ADD_NOISE:
        samples = SAMPLES if SAMPLES <= np.mean([XDIMENSION, YDIMENSION]) else int(XDIMENSION/2)
        x_pts.extend( random.sample( range(0, XDIMENSION*XSPACING), samples) )
        y_pts.extend( random.sample( range(0, YDIMENSION*YSPACING), samples) )

        # delete values near starting area
        for incr, (x, y) in enumerate(zip(x_pts, y_pts)):
            if x < H and y < H: del x_pts[incr], y_pts[incr]

    # Create intensity 2D numpy array using user defined params
    (x_mesh, y_mesh), intensity2DArr = intensity_map( x_pts, y_pts )

    # Generate output .wbo file using 2D numpy array
    wbo_map( intensity2DArr )

    # Slope Map generation
    slope = slope_map( intensity2DArr )

    # Get possible route using Greedy search approach as list [(x1,y1), (x2,y2) ...]
    # # Don't pass border values as their slopes are not accurate due to kerneling method
    clip = lambda array : array[CORNER_SIZE:-CORNER_SIZE, CORNER_SIZE:-CORNER_SIZE]
    solutionRoute_greedy = greedyRoute3D(   clip(intensity2DArr),     # map_array of heights
                                            clip(slope),              # array of slope angles
                                            maxslope=MAX_SLOPE_ANGLE,       # max permissible slope angles
                                            gridsize=(XSPACING,YSPACING) )  # size of each grid segment in [m]

    solutionRoute_astar = astarRoute3D( clip(intensity2DArr),     # map_array of heights
                                        clip(slope),              # array of slope angles
                                        maxslope=MAX_SLOPE_ANGLE,       # max permissible slope angles
                                        gridsize=(XSPACING,YSPACING) )  # size of each grid segment in [m]

    # if a path exists then continue
    if solutionRoute_greedy and solutionRoute_astar:

        # Make set of waypoints for vehicle based on solution
        # Returns as [ (x1,y1), (x2,y2) ... ]
        waypoints_greedy = get_waypoints( route = solutionRoute_greedy )
        waypoints_astar = get_waypoints( route = solutionRoute_astar )

        # Save waypoints and starting location for vehicle in config file (readable by Webots C code)
        wbo_vehicle_config( heightArr = intensity2DArr, points = waypoints_astar if USE_WAYPOINTS else solutionRoute_astar )

        ################# CREATE FIGURES #################
        # reformat data to matplotlib readable version
        x_rt_greedy, y_rt_greedy = get_xys( solutionRoute_greedy )
        x_wpts_greedy, y_wpts_greedy = get_xys( waypoints_greedy )

        x_rt_astar, y_rt_astar = get_xys( solutionRoute_astar )
        x_wpts_astar, y_wpts_astar = get_xys( waypoints_astar )

        # HEADER
        fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
        fig.suptitle(f'Elevation and Slope Heatmaps With Path Planning\n(Max Slope: {MAX_SLOPE_ANGLE})', fontsize=16)
        
        # ELEVATION HEATMAP OUTPUT
        ax1.set(title="Elevation Heatmap")
        ax1.plot(x_pts,y_pts,'ro')
        ax1.axis(xmin=-XSPACING/2, xmax=XDIMENSION*XSPACING-XSPACING, ymin=-YSPACING/2, ymax=YDIMENSION*YSPACING-YSPACING)
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
        ax2.axis(xmin=-XSPACING/2, xmax=XDIMENSION*XSPACING-XSPACING, ymin=-YSPACING/2, ymax=YDIMENSION*YSPACING-YSPACING)
        ax2.legend(['Greedy', 'A*'])
        fig.colorbar( ax2.pcolormesh(x_mesh, y_mesh, slope), ax=ax2 )
        plt.show()
        ###################################################