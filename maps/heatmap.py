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
from sys import platlibdir
import matplotlib.pyplot as plt
import numpy as np
import math
import random

############################## SETUP ###################################
XDIMENSION = 100 # Max number of nodes in x dir
YDIMENSION = 100 # Max number of nodes in y dir

XSPACING = YSPACING = 3    # The spacing between nodes in x, y dir [meters]

XTRANSLATE = -round(XDIMENSION / 2.) # Offset for terrain in x dir
YTRANSLATE = -round(YDIMENSION / 2.) # Offset for terrain in y dir
ZTRANSLATE = 0                       # Offset for terrain in z dir

ROUGHNESS = 1   # Material Roughness    
SAMPLES = 50   # Number of samples used to generate heat map and terrain profile

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

    with open('elevationmap_heatmap.wbo', 'w') as f:
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
    return slope


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

    #ELEVATION HEATMAP OUTPUT
    fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
    ax1.set(title="Elevation Heatmap")
    ax1.plot(x,y,'ro')
    fig.colorbar( ax1.pcolormesh(x_mesh,y_mesh,intensity), ax=ax1 )

    #SLOPE HEATMAP OUTPUT  
    ax2.pcolormesh(x_mesh, y_mesh, slope)
    ax2.set(title="Slope Heatmap")
    ax2.plot(x,y,'ro')
    fig.colorbar( ax2.pcolormesh(x_mesh, y_mesh, slope), ax=ax2 )
    plt.show()