"""
Colour values shown below were taken from the choose colours.py script. If you wish to change them,
simply run the script with the path to your chosen image. Click the colour on HSV window and observe
the results shown in the terminal.

By Oliver Heilmann
"""
import numpy as np
import cv2

class LandTypes:
    """Class to hold land classification and corresponding colour information."""
    def __init__( self, ):
        # outline colours and their ranges as [type : [mid, lower, upper, VRF]]
        # where VRF is the vegetation roughness factor, an indiction of the 
        # velocity a vehicle can pass through the terrain type.
        self.classes = {"Firebrake" : [ [0, 0, 56]    , [-15, -15, 16], [15, 15, 96]  ,  0.3],
                        "Open Area" : [ [56, 106, 255], [41, 91, 215] , [71, 121, 295],  0.5],
                        "River"     : [ [84, 165, 255], [69, 150, 215], [99, 180, 295],  -1.0],    
                        "Swamp"     : [ [10, 198, 240], [-5, 183, 200], [25, 213, 280],  -0.8],
                        "Forest"    : [ [66, 255, 118], [51, 240, 78] , [81, 270, 158],  -0.8],
                        "Orchard"   : [ [164, 81, 212], [149, 66, 172], [179, 96, 252],  -0.5],
                        "Slope"     : [ [            ], [            ], [            ],  -0.4],
                        }
    def get_type_keys( self, drop = None ):
        """Return all land type keys as a list of strings."""
        keys = list(self.classes.keys())
        if drop: keys.remove( drop )
        return keys

    def get_mid_mtlb( self, key : str ):
        """Return the mid colour of key as tuple of floats in range 0 to 1."""
        r = self.classes[key][0][0] / 255.
        g = self.classes[key][0][1] / 255.
        b = self.classes[key][0][2] / 255.
        return  (r, g, b)

    def get_type_info( self, key : str ):
        """Return colours a list of structure [mid, lower, upper, VRF]."""
        return self.classes[ key ]
    
    def get_colour_range( self, key : str ):
        """Return colours a numpy array of structure [lower, upper]."""
        return np.array( self.classes[ key ][1:3] )

    def __str__( self ):
        """Return the terrain classes as list of keys."""
        return f"{list(self.classes.keys())}"


class Tile( LandTypes ):
    """Each section of the terrain has a set of attributes/ traits. Use this class to check if passable."""
    def __init__( self ):
        # initialise inherited land type class
        LandTypes.__init__( self )

        # initialise the traits contained in the tile
        self.elevation = float
        self.slope = float
        self.image = np.ndarray
        self.iop = 0

    def traitCoverage( self, land_class : str ):
        """Check coverage area of land class, return value between 0 and 1."""
        image_hsv = cv2.cvtColor( self.image, cv2.COLOR_BGR2HSV )   # get hsv of image
        lower, upper = self.get_colour_range( land_class )          # get colour range
        image_mask = cv2.inRange( image_hsv, lower, upper )         # create mask
        return cv2.countNonZero(image_mask) / (self.image.size/self.image.shape[-1])    # ratio of colour to whole image

    def isobstacle( self, max_slope = 0.5, vehicle_type = "land", passable = True ):
        """Check if vehicle can pass this tile with given attributes."""
        if vehicle_type == "land":  # cannot travel over water
            passable = True if self.traitCoverage( "River" ) < 0.5 else False
        elif vehicle_type == "water":   # cannot travel over land
            passable = True if self.traitCoverage( "River" ) >= 0.5 else False
        return False if max_slope >= self.slope and passable else True
        
    def __str__( self ):
        """Returns all the trait variable names and their values."""
        return f"{ vars( self ) }"


if __name__ == "__main__":
    tile = Tile()
    land_types = LandTypes()
    print(land_types)