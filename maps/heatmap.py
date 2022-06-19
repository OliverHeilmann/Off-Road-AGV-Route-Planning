"""
Description:
    -   Creates WeBots readable terrain elevation map of .wbo file format. This approach uses
        randomly generated points [x,y] and a quartic regression function to determine point
        densities, called Kernel Density Estimation (KDE); higher point densities correspond 
        to higher elevations.
    -   Using the output terrain elevation map, a slope map is generated using the method shown
        in the imgs folder (https://www.mdpi.com/2220-9964/10/11/785).

By Oliver Heilmann
Modified from https://www.geodose.com/2018/01/creating-heatmap-in-python-from-scratch.html
"""
import matplotlib.pyplot as plt
import numpy as np
import math
import random
import pickle
import sys     
sys.path.append('/Users/Oliver/Documents/CODING/Python_Prgms/WeBots_ElevationMap/search')   
from greedysearch import get_solution
from scipy import stats

############################## SETUP ###################################
# VEHICLES
MAX_SLOPE_ANGLE = .29   # Maximum permissible slope angle for vehicle as ratio of Rise/Run
VEHICLE_LENGTH = 2.964  # Vehicle length in meters
RSQ_THRESHOLD = 0.999999    # R-Squared value for determining waypoints (lower val ∝ less waypoints)

# MAPS
XDIMENSION = 100 # Max number of nodes in x dir
YDIMENSION = 100 # Max number of nodes in y dir

XSPACING = YSPACING = 1    # The spacing between nodes in x, y dir [meters]

XTRANSLATE = -round(XDIMENSION / 2.) # Offset for terrain in x dir
YTRANSLATE = -round(YDIMENSION / 2.) # Offset for terrain in y dir
ZTRANSLATE = 0                       # Offset for terrain in z dir

ROUGHNESS = 1   # Material Roughness    
SAMPLES = 60   # Number of samples used to generate heat map and terrain profile

#DEFINE GRID SIZE AND RADIUS(h)
GRID_SIZE=1
H=15

#POINT DATASET (FOR CREATING PEAKS)
x=random.sample(range(0, XDIMENSION), SAMPLES)
x.extend( [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50] )  # Add more data points to emphasize areas to elevate further

y=random.sample(range(0, YDIMENSION), SAMPLES)
y.extend( [10,10,10,20,20,20,30,30,30,40,40,40,50,50,50] )  # Add more data points to emphasize areas to elevate further
#######################################################################


#FUNCTION TO CALCULATE INTENSITY WITH QUARTIC KERNEL
def kde_quartic(d,H):
    dn=d/H
    P=(15/16)*(1-dn**2)**2
    return P


#REFORMAT CALCULATED HEIGHTS INTO THE .WBO FILE FORMAT READY FOR WEBOTS
def wbo_format( output_string ):
    formatted =  """#VRML_OBJ R2022a utf8
Solid {{
    translation {} {} {}
    children [
        Shape {{
            appearance PBRAppearance {{
                roughness {}
            }}
            geometry ElevationGrid {{
                height [{}]
                xDimension {}
                yDimension {}
            }}
        }}
    ]
name "ELE_MOD"
}}  """.format( XTRANSLATE,
                YTRANSLATE,
                ZTRANSLATE,
                ROUGHNESS,
                output_string,
                XDIMENSION,
                YDIMENSION,
                )

    with open('maps/elevationmap_heatmap.wbo', 'w') as f:
        f.write( formatted )


#CALCULATE THE SLOPE MAP USING PREVIOUSLY GENERATED TERRAIN ELEVATION DATA
def slope_map( elev : np.array ):
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
    return slope#[1:-1, 1:-1]    # drop borders as the slopes are impacted NaN neighbours


#CREATE A SET OF WAYPOINTS FOR VEHICLE TO DRIVE TOWARD
def waypoints( x_arr : np.array, y_arr : np.array):
    x_wpts = [ x_arr[0] ]
    y_wpts = [ y_arr[0] ]

    x_prev = x_arr[0]
    y_prev = y_arr[0]

    xs, ys = [], []
    for x_curr, y_curr in zip(x_arr, y_arr):
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
            x_wpts.append( x_prev )
            y_wpts.append( y_prev )
            xs, ys = [x_prev, x_curr], [y_prev, y_curr]

        # update previous values to equal current before new loop
        x_prev, y_prev = x_curr, y_curr
    x_wpts.append(x_curr)
    y_wpts.append(y_curr)
    return x_wpts, y_wpts


#MAIN PROCESSING
if __name__ == '__main__':
    #CONSTRUCT GRID
    x_grid=np.arange(0,XDIMENSION,GRID_SIZE)
    y_grid=np.arange(0,YDIMENSION,GRID_SIZE)
    x_mesh,y_mesh=np.meshgrid(x_grid,y_grid)

    #GRID CENTER POINT
    xc=x_mesh+(GRID_SIZE/2)
    yc=y_mesh+(GRID_SIZE/2)

    intensity_list=[]
    for j in range(len(xc)):
        intensity_row=[]
        for k in range(len(xc[0])):
            kde_value_list=[]
            for i in range(len(x)):
                #CALCULATE DISTANCE
                d=math.sqrt((xc[j][k]-x[i])**2+(yc[j][k]-y[i])**2) 
                if d<=H:
                    p=kde_quartic(d,H)
                else:
                    p=0
                kde_value_list.append(p)
            #SUM ALL INTENSITY VALUE
            p_total=sum(kde_value_list)
            intensity_row.append(p_total)
        intensity_list.append(intensity_row)

    # Generate output .wbo file
    heights = ",".join( [",".join(item) for item in np.round(intensity_list,2).astype(str)] )
    wbo_format( heights )

    # Slope Map generation
    intensity=np.array(intensity_list)
    slope = slope_map( intensity )

    # Save slope 2D array as pickle file for use in path planning algorithm
    with open('maps/elevationmap_2darray.pickle', "wb") as f:
        pickle.dump(slope[1:-1, 1:-1], f)   # drop borders of slope map as gradients are inaccurate
    solutionArr = get_solution('maps/elevationmap_2darray.pickle', maxslope=MAX_SLOPE_ANGLE)

    # unpack solution for plotting
    ys = [coord[0]+1 for coord in solutionArr[::-1]]
    xs = [coord[1]+1 for coord in solutionArr[::-1]]

    # Make set of waypoints for vehicle based on solution
    wpts_x, wpts_y = waypoints( x_arr = xs, y_arr = ys)

    # Save waypoints as text file usable in WeBots
    wpts_string = ",".join( ['{' + str(el[0] - (XDIMENSION/2)) + ','    \
                                 + str(el[1] - (YDIMENSION/2)) + '}'    \
                                    for el in zip(wpts_x,wpts_y)] )
    with open('maps/waypoints.txt', "w") as f:
        f.write( wpts_string )

    #ELEVATION HEATMAP OUTPUT
    fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
    fig.suptitle(f'Elevation and Slope Heatmaps With Path Planning\n(Type: Greedy, Max Slope: {MAX_SLOPE_ANGLE})', fontsize=16)
    ax1.set(title="Elevation Heatmap")
    ax1.plot(x,y,'ro')
    fig.colorbar( ax1.pcolormesh(x_mesh,y_mesh,intensity), ax=ax1 )

    #SLOPE HEATMAP OUTPUT  
    ax2.set(title="Slope Heatmap")
    ax2.plot(xs, ys,'r-')
    ax2.plot(wpts_x, wpts_y,'bo')
    fig.colorbar( ax2.pcolormesh(x_mesh, y_mesh, slope), ax=ax2 )
    plt.show()