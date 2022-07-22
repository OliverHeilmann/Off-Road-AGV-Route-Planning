from PIL import Image
import numpy as np
import cv2

############################## SETUP ###################################

tiff_path = '/Users/Oliver/Downloads/Netherlands_0p5mRES_1300mSQR.tiff' # path to tiff file

#######################################################################

class DEM:
    """Manipulate input TIFF image into a Numpy friendly version which is compatible rest of the library."""
    def __init__( self, impath : str, ):
        self.imgTiff = Image.open( impath )
        
        # convert tiff to numpy array for handling later
        img_arr = np.array(self.imgTiff)
        
        # drops cols where all values are nan
        im_short_np = img_arr[:, ~np.isnan(img_arr).all(axis=0)]

        # Adjust image to be in range 0 to Max, replacing Nans with new min (which equals zero always)
        self.imgNpy = np.nan_to_num(im_short_np - np.nanmin(im_short_np),   # subtract min value from all to set range from 0 --> N
                                    nan = 0 )                               # equals zero after adjustment

    def resize( self, width : int ):
        """Resize image into square to align with WeBots formatting, keep as class variable"""
        self.imgNpy = cv2.resize(   self.imgNpy,                               # image to resize
                                    (width, width),                             # make into square
                                    interpolation = cv2.INTER_LINEAR_EXACT )    # interpolation method!

    def showTiff( self, tiff = True, npy = True ):
        """Show original TIFF image and corrected versions by default."""
        if tiff:
            self.imgTiff.show()
        if npy:
            Image.fromarray( self.imgTiff ).show()

    def __str__( self ):
        """Show sizes of each TIFF image """
        return f'TIFF  (r,c): ({self.imgTiff.size[0]}, { self.imgTiff.size[1]})\nNUMPY (r,c): ({self.imgNpy.shape[1]}, { self.imgNpy.shape[0]})'


if __name__ == '__main__':
    demcls = DEM( tiff_path )
    print( demcls )