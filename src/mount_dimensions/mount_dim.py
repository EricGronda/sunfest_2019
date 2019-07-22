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
IMAGE_WIDTH  = 4.42 # dimension in mm; 240C only
IMAGE_HEIGHT = 2.49 #

#---------------------------------------------------------------
# mountParams() calculates the parameters of the mount
# Input:        f; focal length of the lens (mm)
#               d; distance from the camera to center mirror (cm)
# Output:       a list of information in the format
#                 [ angle between side mirrors and wall,
#                   length of center mirror,
#                   length of side mirrors,
#                   maximum field of view               ]
def mountParams( f , d ):
    q = IMAGE_WIDTH / (3 * f) # this constant appeared a lot,
                              # not sure why

    # these equations were determined through geometry
    # so for the explanation, look at my notebook
    angle   = ( math.atan(q) )
    lenCent = q * d  # convert to mm
    lenSide = lenCent / math.cos( angle )
    maxFOV  = math.atan(3 * q)

    return [ angle * 180 / math.pi , lenCent , lenSide , maxFOV * 180 / math.pi ]

def main():
    focalLength = float( input("Enter the focal length (mm): "))
    distance    = float( input("Enter the distance from the camera (cm): "))

    data = mountParams( focalLength , distance )
    
    print()
    print("Angle from wall:   " , data[0] )
    print("Center length (cm):" , data[1] )
    print("Side length (cm):  " , data[2] )
    print("Max field of view: " , data[3] )

main()
