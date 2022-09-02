"""
This script contains a class which holds various vehicle properties. If you wish to add another vehicle
type to this class, include it in the dictionary and select appropriate values for it/ them.
"""

class Vehicles:
    """Class to hold vehicle information including vegetation roughness values and other standard properties."""
    def __init__( self ):
        pass

    def __new__( cls, vehicle = "MOOSE" ):
        """Return vehicle data in a dictionary format."""
        # Below information is contained in the below format, units are also defined
        # "VEHICLE TYPE" : { "max_slope_angle" : A [rad], 
        #                    "length"          : B [m], 
        #                    "height"          : C [m], 
        #                    "max_velocity"    : D [km/h],
        #                    "vrf"             : ["Firebreak","Open_Area","River","Stream","Swamp","Forest","Orchard","Snow","Slope"]
        #                   },
        modeOfTransport = {
            "MOOSE"          : {    "max_slope_angle" : 0.698,   # 20: 0.349 
                                                                    # 30: 0.524
                                                                    # 40: 0.698
                                                                    # 50: 0.873
                                    "length" : 2.964, 
                                    "height" : 1.145, 
                                    "max_velocity" : 30.0,
                                    "vrf" : [0.8,    0.5,    -0.73,   -0.8,   -0.8,   -0.8,  -0.5, -0.2,    -0.4],    # -1
                                    "terrain_type" : 'amphibious',
                                },
            "HUMAN"          : {    "max_slope_angle" : 0.785398,
                                    "length" : 0.5,
                                    "height" : 1.83,
                                    "max_velocity" : 9.5,
                                    "vrf" :  [0.3,    0.1,    -1.0,   -0.8,   -0.8,   -0.1,  -0.1, -0.4,    -0.7],
                                    "terrain_type" : 'land',
                                },
            "MOTOCROSS BIKE" : {    "max_slope_angle" : 0.558505,
                                    "length" : 1.87,
                                    "height" : 1.90,
                                    "max_velocity" : 50.0,
                                    "vrf" : [0.8,    0.65,   -1.0,   -0.9,   -1.0,   -0.4,  -0.1, -0.6,    -0.3],
                                    "terrain_type" : 'land',
                                },
        }
        return modeOfTransport[vehicle]

    def __str__( self ):
        """Returns a string representation of the vehicle type"""
        return self.vehicle