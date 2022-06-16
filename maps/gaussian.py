"""
Description:
    -   Creates WeBots readable terrain elevation map of .wbo file format. This approach uses
        Gaussian distributions with noise.

By Oliver Heilmann
"""
import numpy as np
import random

############################## SETUP ###################################
XDIMENSION = 300 # Max number of nodes in x dir
YDIMENSION = 300 # Max number of nodes in y dir

XSPACING = 3    # The spacing between nodes in x dir [meters]
YSPACING = 3    # The spacing between nodes in y dir [meters]

XTRANSLATE = -round(XDIMENSION / 2.) # Offset for terrain in x dir
YTRANSLATE = -round(YDIMENSION / 2.) # Offset for terrain in y dir
ZTRANSLATE = 0                      # Offset for terrain in z dir

ROUGHNESS = 1                       # Material Roughness     

BASE_HEIGHT = 0     # Base height of all nodes
MAX_DIFF = 0.4        # Max height difference between nodes
RADIUS = 3          # Radius of cells to average height across (larger == smoother height diffs)
RESMOOTH = 2        # Number of times the peaks are resmoothed (averaged with neighbours)

GAUSSIAN_PEAKS = 5  # Number of Gaussian peaks to generate
GAUSSIAN_SCALE = 9 # Size of Gaussian scatter (i.e. spread over range +-15)
#######################################################################


def wbo_format( output_string ):
    """Reformat calculated heights into the .wbo file format ready for WeBots."""
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

    with open('maps/elevationmap_gaussian.wbo', 'w') as f:
        f.write( formatted )


def neighbours(mat, radius, row_number, column_number):
    """Get neighbour cells of matrix and return as array."""
    return [[mat[i][j] if  i >= 0 and i < len(mat) and j >= 0 and j < len(mat[0]) else 0
                for j in range(column_number-1-radius, column_number+radius)]
                    for i in range(row_number-1-radius, row_number+radius)]


if __name__ == '__main__':
    map_array = np.zeros( [XDIMENSION, YDIMENSION] )
    node_heights = np.random.rand( XDIMENSION, YDIMENSION ) + BASE_HEIGHT

    # create mountain peaks
    num_peaks = random.randint( int((XDIMENSION+YDIMENSION)/4), int((XDIMENSION+YDIMENSION)/2) )
    max_peak_height = MAX_DIFF * int((XDIMENSION+YDIMENSION)/2)

    # Gaussian peak generation
    _loc = 1
    rVal = [random.randint(int((XDIMENSION+YDIMENSION)/4), int((XDIMENSION+YDIMENSION)/2)) for step in range(2)]
    gaus = np.random.normal(loc=_loc, scale=GAUSSIAN_SCALE, size=(rVal[0], rVal[1]))

    for count in range(GAUSSIAN_PEAKS):
        # Gaussian noise added
        noise = gaus + np.random.normal(_loc, .1, gaus.shape)
        # Add Gaussian values into node_heights
        r = node_heights.shape[0] - gaus.shape[0]
        c = node_heights.shape[1] - gaus.shape[1]
        node_heights[r:r+gaus.shape[0], c:c+gaus.shape[1]] += gaus

        # choose x,y's to assign peak locations to
        xvals = random.sample(range(0, XDIMENSION), num_peaks)
        yvals = random.sample(range(0, YDIMENSION), num_peaks)
        peak_vals = [ random.uniform(0, max_peak_height) for _ in range(num_peaks) ]
        for peak in zip(xvals, yvals, peak_vals):
            node_heights[peak[0]][peak[1]] = peak[2]

    node_num = 0
    for resmooth in range(RESMOOTH):
        for i, row in enumerate(map_array):
            
            for j, col in enumerate(row):

                # get current neighbours of current cell 
                curr_neighbour_mat = neighbours(node_heights, RADIUS, i+1,j+1)

                # Get prev node height and choose random height between 
                min_height = np.min(curr_neighbour_mat)
                height_diff = random.uniform(   MAX_DIFF - BASE_HEIGHT,
                                                MAX_DIFF - BASE_HEIGHT  )
                node_heights[i][j] = np.mean(curr_neighbour_mat) + height_diff - node_heights[0][0]

                node_num += 1

    # Generate output
    heights = ",".join( [",".join(item) for item in np.round(node_heights,2).astype(str)] )
    wbo_format( heights )