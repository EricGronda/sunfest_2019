# File: mount_dim.py
# Author: Eric Gronda
# Date: 7/1/19
# Email: eric.gronda@umbc.edu
# Description:
#    This program will calculate the parameters for 
#    the mount given the following:
#
#    Focal Length
#    Camera Type (Davis 240C)
#    Distance from camera (not the lens) to center mirror
#
#    This is heavily based on symmetry, so the camera must be pointed
#    at the center of the center mirror
import math

# Constants
#IMAGE_WIDTH  = 4.42 # dimension in mm; 240C only
#IMAGE_HEIGHT = 2.49 #

IMAGE_WIDTH  = 6.4  # (mm) 346 only
IMAGE_HEIGHT = 4.81 # 

TILT = math.pi / 4 # Tilt is 45 degrees

#---------------------------------------------------------------
# mountParams() calculates the parameters of the mount
# Input:        f; focal length of the lens (mm)
#               d; distance from the camera body to center pt of
#                  center mirror (cm)
# Output:       a list of information in the format
#                 [ angle between side mirrors and wall,
#                   height of center mirror,
#                   width of center mirror,
#                   maximum width field of view,  
#                   maximum height field of view       ]
def mountParams( f , d ):
    q = IMAGE_WIDTH / (3 * f) # this constant appeared a lot,
                              # not sure why

    # these equations were determined through geometry
    # so for the explanation, look at my notebook
    angle     = math.atan(q)
    widthFOV  = math.atan(3 * q)
    heightFOV = 2 * math.atan( IMAGE_HEIGHT / (2 * f) )

    #lenCent = q * d  # convert to mm
    #lenSide = lenCent / math.cos( angle )
    
    # center mirror dimensions
    centerWidth  = q * d
    centerHeight = 2 * (d + f) * math.sin( heightFOV ) * math.tan(TILT)

    # side mirror dimensions
    sideWidth = centerWidth / math.cos( angle )
    outerSideHeight = centerHeight + (sideWidth * math.cos(TILT))

    return [ angle * 180 / math.pi , 
             centerHeight , centerWidth ,  
             widthFOV * 180 / math.pi , heightFOV * 180 / math.pi,
             sideWidth , outerSideHeight                          ]

def main():
    focalLength = float( input("Enter the focal length (mm): "))
    distance    = float( input("Enter the distance from the camera body" 
        + " to the midpoint of the mirror (cm): "))

    data = mountParams( focalLength , distance )
    
    print("\n--- Important angles (degrees) ---")
    print("Angle from wall:   " , data[0] )
    print("Width FOV:         " , data[3] )
    print("Height FOV:        " , data[4] )
    
    print("\n--- Mirror Dimensions ---")
    print("Center Height (cm):" , data[1] )
    print("Center Width (cm): " , data[2] )
    print("Side width:        " , data[5] )
    print("Outer Side Height: " , data[6] )

main()
