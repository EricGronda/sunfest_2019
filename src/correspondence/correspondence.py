# File: bag_divider.py
# Author: Eric Gronda
# Date: 7/12/19
# Email: eric.gronda@umbc.edu
# Description:
#    This program takes in a rosbag that has recorded a video for the event 
#    based stereo with mirrors project, and divides up every frame into 3 
#    separate frames, calculates the homographies for each, and then saves to 3
#    rosbags (left, center, right mirrors). This is to be used for calibrating.
import rosbag
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from getXY import * # python ginput equivalent (see getXY.py)

# Constants
YELLOW = (255, 255, 0)
RED = (255 , 0 , 0)
BLUE = (0 , 0 , 255)
GREEN = (0 , 255 , 0)
WHITE = (255, 255, 255)

PX_WIDTH = 346
PX_HEIGHT = 240

LEFT = 0
CENTER = 1
RIGHT = 2

SEARCH = 5

####################################################################
# imshow() displays a cv image to the screen
# input:   img; a cv image
# output:  none (displays to screen)
def imshow( img ):
    cv2.imshow("Image window", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

####################################################################
# pickLine() selects 2 points for a line, then draws it on the image
# input:     img; a cv image to select points from
# output:    imgFrame; blank image matrix with only the lines selected
#            pts; matrix of line points (topL, botL, topR, botR)
def pickLines( img ): 
    print "Select the line points in the following order:"
    print "\tTop Left"
    print "\tBottom Left"
    print "\tTop Right"
    print "\tBottom Right"

    # python ginput equivalent
    #pts = getXY( img )
    pts = [ [147 , 27 ],
            [125 , 257],
            [190 , 27 ],
            [211 , 258]]

    # Generate new image frame
    imgFrame = np.zeros((PX_HEIGHT , PX_WIDTH , 3), np.uint8)

    # convert pts to tuples
    topL = ( int(pts[0][0]) , int(pts[0][1]) )
    botL = ( int(pts[1][0]) , int(pts[1][1]) )

    topR = ( int(pts[2][0]) , int(pts[2][1]) )
    botR = ( int(pts[3][0]) , int(pts[3][1]) )

    # draw lines on image
    cv2.line(imgFrame, topL , botL , YELLOW, 1)
    cv2.line(imgFrame, topR , botR , YELLOW, 1)

    print "\n"
    return imgFrame, pts

#####################################################################
# displayEvents() shows the events of an image in gif format
# input:          gen; /davis/events generator
#                 frame; image frame to display points on
#                 pts; matrix of line points (topL, botL, topR, botR)
# output:         none; displays in window
def displayEvents( gen , frame , pts ):
    
    for topic, msg, t in gen:
    
        # create a new empty matrix
        img = frame.copy()

        # store events by mirror
        left = []
        right = []
        center = []

        # limit correspondences to 1 in every x events
        counter = 0

        # fill with event points
        for event in msg.events:
            # choose color
            if findMirror(event , pts) == LEFT:
                color = RED
                left.append( event )
            elif findMirror(event , pts) == CENTER:
                color = GREEN
                center.append( event )
            elif findMirror(event , pts) == RIGHT:
                color = BLUE
                right.append( event )
            else:
                color = WHITE

            if counter == 100:
                correspond( img , event , left , center , right , pts ) 
                counter = 0
            counter += 1

            #if event.polarity:
            #    color = BLUE

            # draw circles
            cv2.circle(img , (event.x , event.y) , 2 , color, -1)

        # display image
        cv2.imshow("Events Gif", img)
        cv2.waitKey(500)
        
    cv2.destroyAllWindows()

###################################################################
# findMirror() finds which mirror the event point is in
# input:       event; event object (includes coords & time)
#              pts; line pts (topL, botL, topR, botR)
# output:      LEFT (0), CENTER (1), RIGHT (2); integer for mirror 
def findMirror( event , pts ):
    topL = pts[0]
    botL = pts[1]
    topR = pts[2]
    botR = pts[3]

    # find slopes of lines (y values inverted)
    leftSlope  = -( topL[1] - botL[1] ) / ( topL[0] - botL[0] )
    rightSlope =  ( topR[1] - botR[1] ) / ( topR[0] - botR[0] )

    # use point-slope formula to find x value of lines at event y
    leftX  = (((-event.y) - (-botL[1])) / leftSlope ) + botL[0]
    rightX = (((-event.y) - (-botR[1])) / leftSlope ) + botR[0]

    # choose mirror based on x valueif left of left line, then on left mirror
    #if event.x > leftX and event.x < rightX:
    #    return CENTER
    if event.x < rightX:
        return RIGHT
    elif event.x > leftX:
        return LEFT
    else:
        return CENTER

####################################################################
# correspond() draws a line from an event to another 
#                      corresponding event
# input:       img; image to draw line on
#              event; event to find correspondence of
#              left; list of events from left
#              center; list of events from center
#              right; list of events from right
# output:      img; updated image with line drawn
def correspond( img, event, left, center, right , pts):
    # immediately exclude mirror event is in
    location = findMirror( event , pts )
    mirrors = []

    if location == CENTER:
        mirrors = [ left , right ]
    elif location == LEFT:
        mirrors = [ center , right ]
    else:
        mirrors = [ left , center ]

    # Search +/- 5 rows for corresonding event (approx.)
    for compare in mirrors[0]:
        # match polarity and rows
        if (event.y in range( compare.y - SEARCH, compare.y + SEARCH )
            and event.polarity == compare.polarity):
            
            # draw a line for a match
            cv2.line(img, (event.x , event.y) , (compare.x , compare.y), 
                    WHITE, 1)
            break
    
def main():
    # NOTE: interesting stuff moreso at frame 200
    bag = rosbag.Bag('../bagfiles/mount_1/calibrate.bag') 
    print(bag)
    print('------------------------------------------------------------\n')

    # get image from generator
    gen = bag.read_messages(topics=['/davis/image_raw']) 
    topic, msg, t = next(gen)

    # convert to cv image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # divide image into left, right, center
    frame, pts = pickLines( cv_image )
    imshow( frame )    

    # select random event points in center
    gen = bag.read_messages(topics=['/davis/events'])
    displayEvents(gen , frame, pts)

    '''
    topic, msg, t = next(next(gen))

    # plot all points test
    for event in msg.events:

        # choose color
        color = RED
        if event.polarity:
            color = BLUE

        # draw circles
        cv2.circle(cv_image , (event.x , event.y) , 2 , YELLOW, -1)
    imshow(cv_image)
    '''

    # select possible matches in left and right based on events

    # close the bag
    bag.close()

main()
