"""
Colour values shown below were taken from the choosecolours.py script. If you wish to change them,
simply run the script with the path to your chosen image. Click the colour on HSV window and observe
the results shown in the terminal.

By Oliver Heilmann
"""
import numpy as np

class LandTypes:
    """Class to hold land classification and corresponding colour information."""
    def __init__( self, ):
        # outline colours and their ranges
        self.classes = {"Firebrake" : [ [0, 0, 56], [-15, -15, 16], [15, 15, 96] ],
                        "Open Area" : [ [56, 106, 255], [41, 91, 215], [71, 121, 295] ],
                        "River"     : [ [84, 165, 255], [69, 150, 215], [99, 180, 295] ],    
                        "Swamp"     : [ [10, 198, 240], [-5, 183, 200], [25, 213, 280] ],
                        "Forest"    : [ [66, 255, 118], [51, 240, 78], [81, 270, 158] ],
                        "Orchard"   : [ [164, 81, 212], [149, 66, 172], [179, 96, 252] ],
                        }
    def get_type_keys( self ):
        """Return all land type keys as a list of strings."""
        return list(self.classes.keys())

    def get_type_info( self, key : str ):
        """Return colours a list of structure [mid, lower, upper]."""
        return self.classes[ key ]
    
    def get_colour_range( self, key : str ):
        """Return colours a numpy array of structure [lower, upper]."""
        return np.array( self.classes[ key ][1:] )

    def __str__( self ):
        """Return the terrain classes as list of keys."""
        return f"{list(self.classes.keys())}"


if __name__ == "__main__":
    land_types = LandTypes()
    print( land_types.get_colour( "Firebrake" ) )
    print(land_types)