"""
Colour values shown below were taken from the choosecolours.py script. If you wish to change them,
simply run the script with the path to your chosen image. Click the colour on HSV window and observe
the results shown in the terminal.

By Oliver Heilmann
"""

class LandTypes:
    """Class to hold land classification and corresponding colour information."""
    def __init__( self, ):
        # outline colours and their ranges
        self.classes = {"Firebrake" : [ [0, 0, 56], [-10, -10, 16], [10, 10, 96] ],
                        "Open Area" : [ [56, 106, 255], [46, 96, 215], [66, 116, 295] ],
                        "River"     : [ [84, 165, 255], [74, 155, 215], [94, 175, 295] ],    
                        "Swamp"     : [ [10, 198, 240], [0, 188, 200], [20, 208, 280] ],
                        "Forest"    : [ [66, 255, 118], [56, 245, 78], [76, 265, 158] ],
                        "Orchard"   : [ [164, 81, 212], [154, 71, 172], [174, 91, 252] ],
                        }

    def get_type_info( self, key : str ):
        """Return colours a list of structure [mid, lower, upper]."""
        return self.classes[ key ]
    
    def get_colour_range( self, key : str ):
        """Return colours a list of structure [lower, upper]."""
        return self.classes[ key ][1:]

    def __str__( self ):
        """Return the terrain classes as list of keys."""
        return f"{list(self.classes.keys())}"


if __name__ == "__main__":
    land_types = LandTypes()
    print( land_types.get_colour( "Firebrake" ) )
    print(land_types)