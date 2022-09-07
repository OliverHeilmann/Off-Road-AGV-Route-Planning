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
    def __init__( self, vrf : list = None ):
        # HSV colours and their ranges as [type : [rgb, mid, lower, upper, VRF]]
        # where VRF is the vegetation roughness factor, an indiction of the 
        # velocity a vehicle can pass through the terrain type.
        # ------TYPE--------------RGB------------MID HSV--------LOWER HSV-------UPPER HSV ---------VEHICLE VRF----
        self.classes = {
            "Road"      : [ [0, 0, 0]      , [0, 0, 0]      , [-15,-15,-40]  , [15, 15, 40]   ,  vrf[0]],
            "Open_Area" : [ [144, 208, 80] , [45, 156, 208] , [30, 141, 168] , [60, 171, 248] ,  vrf[1]],
            "River"     : [ [0, 127, 255]  , [105, 255, 255], [90, 240, 215] , [120, 270, 295],  vrf[2]],
            "Stream"    : [ [117, 255, 223], [83, 137, 255] , [68, 122, 215] , [98, 152, 295] ,  vrf[3]],
            "Swamp"     : [ [237, 124, 49] , [12, 202, 237] , [-3, 187, 197] , [27, 217, 277] ,  vrf[4]],
            "Forest"    : [ [20, 129, 20]  , [60, 215, 129] , [45, 200, 89]  , [75, 230, 169] ,  vrf[5]],
            "Orchard"   : [ [213, 125, 174], [163, 105, 213], [148, 90, 173] , [178, 120, 253],  vrf[6]],
            "Snow"      : [ [230, 230, 230], [0, 0, 255]    , [-15, -15, 215], [15, 15, 295]  ,  vrf[7]],
            "Slope"     : [ [             ], [            ] , [            ] , [            ] ,  vrf[8]],
        }
        # return object with properties for passing to tile class later
        return self

    def get_RGB_WeBots( self, maxvel = 30, iop_order = 1 ):
        """Returns the Terrain type, mid colour in RBG and the VRF as dictionary"""
        v = lambda tile : round(0.5 * maxvel * ( 1 + tile[1][-1]**iop_order), 4)       # tile v rounded to 4d.p.
        return { item[0] : [item[1][0],                                                 # RGB list
                            v(item) if v(item) <= maxvel else maxvel ]                              # vehicle velocity
                            for item in self.classes.items() if item[0] != "Slope" }    # exclude slopes

    def get_type_keys( self, drop = None ):
        """Return all land type keys as a list of strings."""
        keys = list(self.classes.keys())
        if drop: keys.remove( drop )
        return keys

    def get_mid_mtlb( self, key : str ):
        """Return the mid HSV colour of key as tuple of floats in range 0 to 1."""
        r = self.classes[key][0][0] / 255.
        g = self.classes[key][0][1] / 255.
        b = self.classes[key][0][2] / 255.
        return (r, g, b)

    def get_type_info( self, key : str ):
        """Return colours a list of structure [mid, lower, upper, VRF]."""
        return self.classes[ key ]
    
    def get_colour_range( self, key : str ):
        """Return colours a numpy array of structure [lower, upper]."""
        return np.array( self.classes[ key ][2:4] )

    def __str__( self ):
        """Return the terrain classes as list of keys."""
        return f"{list(self.classes.keys())}"


class Tile:
    """Each section of the terrain has a set of attributes/ traits. Use this class to check if passable."""
    def __init__( self, landObj : LandTypes = None ):
        # make LandTypes object accessible in this class
        self.landObj = landObj

        # initialise the traits contained in the tile
        self.elevation = float
        self.slope = float
        self.image = np.ndarray
        self.iop = 0            # will add to iteratively
        self.velocity = float
        self.obstacle = None    # after calculating if obstacle, update its internal value to save on next call (either T/F)

    def get_type_info( self, key : str ):
        """Return colours a list of structure [mid, lower, upper, VRF]."""
        return self.landObj.classes[ key ]

    def traitCoverage( self, land_class : str ):
        """Check coverage area of land class, return value between 0 and 1."""
        image_hsv = cv2.cvtColor( self.image, cv2.COLOR_BGR2HSV )   # get hsv of image
        lower, upper = self.landObj.get_colour_range( land_class )          # get colour range
        image_mask = cv2.inRange( image_hsv, lower, upper )         # create mask
        return cv2.countNonZero(image_mask) / (self.image.size/self.image.shape[-1])    # ratio of colour to whole image

    def isobstacle( self, max_slope = 0.5, vehicle_type = "land", passable = True ):
        """Check if vehicle can pass this tile with given attributes."""
        if self.obstacle == None:
            if vehicle_type == "land":  # cannot travel over water
                passable = True if self.traitCoverage( "River" ) < 0.5 else False   # if River covers less than 50% of terrain...
            elif vehicle_type == "water":   # cannot travel over land
                passable = True if self.traitCoverage( "River" ) >= 0.5 else False  # if River covers more than 50% of terrain...
            elif vehicle_type == "amphibious":   # can travel anywhere
                passable = True
            return False if max_slope >= self.slope and self.velocity > 0.0 and passable else True # False if NOT an obstacle!
        # if here, return whether tile is an obstacle or not as bool
        return self.obstacle

    def __str__( self ):
        """Returns all the trait variable names and their values."""
        return f"{ vars( self ) }"


if __name__ == "__main__":
    tile = Tile()
    land_types = LandTypes()
    print(land_types)