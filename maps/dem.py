from PIL import Image
from matplotlib import pyplot as plt
import numpy as np
import cv2

############################## SETUP ###################################

tiff_path = 'maps/bathDEM.tiff' # path to tiff file

# im = cv2.imread('maps/Sentinel2_RGB20200506.tiff', -1)
# cv2.imshow("img", im)
# plt.imshow( cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) )
# plt.show()

#######################################################################

class DEM:
    """Manipulate input TIFF image into a Numpy friendly version which is compatible rest of the library."""
    def __init__( self, impath : str, shape : tuple = None ):
        self.imgTiff = Image.open( impath )
        
        # convert tiff to numpy array for handling later
        img_arr = np.array(self.imgTiff)
        
        # drops cols where all values are nan
        im_short_np = img_arr[:, ~np.isnan(img_arr).all(axis=0)]

        # Adjust image to be in range 0 to Max, replacing Nans with new min (which equals zero always)
        self.imgNpy = np.nan_to_num(im_short_np - np.nanmin(im_short_np),   # subtract min value from all to set range from 0 --> N
                                    nan = 0 )                               # equals zero after adjustment
        
        # resize to the global map size (so all map features are the corresponding scale)
        self.imgNpy = self.resizeDEM( shape ) if shape else self.imgNpy

        # reverse the order of elements along axis 0 (up/down) to align with global coordinate system
        self.imgNpy = np.flipud(self.imgNpy)

    def resizeDEM( self, shape : tuple, interp = cv2. INTER_CUBIC ):
        """Resize image into square to align with WeBots formatting, return new shape."""
        return cv2.resize(  self.imgNpy.astype('float32'),      # image to resize with values as floats
                            shape,                              # make into square
                            interpolation = interp )            # interpolation method!

    def wb_heights( self ):
        """Return the heights of the DEM in WeBots readable string format."""
        return ",".join( [",".join(item) for item in np.round(self.imgNpy,2).astype(str)] ) # np to .wbo format

    def show( self, tiff = True, npy = True ):
        """Show original TIFF image and corrected versions by default."""
        if tiff: 
            # rescaling numpy array from 0 to 255 as TIFF format operates in greyscale within this range
            rescale_np_arr = lambda arr : ((arr - arr.min()) * (1/(arr.max() - arr.min()) * 255)).astype('uint8')

            # do the rescaling now
            img_adj_0to255 = rescale_np_arr( self.imgNpy )

            # make origin bottom left
            img_adj_0to255 = np.flip(img_adj_0to255, 0)

            # turn back to tiff and show again to check borders have been dropped correctly
            im_short_tiff = Image.fromarray(img_adj_0to255)
            im_short_tiff.show()
        if npy:
            # plotting with matplotlib
            plt.imshow(self.imgNpy[:, :], cmap=plt.cm.coolwarm, origin='lower')
            plt.show()

    def __str__( self ):
        """Show sizes of each TIFF image """
        return f'TIFF  (r,c): ({self.imgTiff.size[0]}, { self.imgTiff.size[1]})\nNUMPY (r,c): ({self.imgNpy.shape[1]}, { self.imgNpy.shape[0]})'


if __name__ == '__main__':
    demcls = DEM( tiff_path )
    print( demcls )
    demcls.show()
    wb_dem = demcls.wb_heights()