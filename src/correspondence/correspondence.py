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
PX_HEIGHT = 260

LEFT = 0
CENTER = 1
RIGHT = 2

SEARCH = 5

UPSCALE_PERCENT = 250
FRAME_DELAY = 10000 # millisecs

EVENT_WINDOW = 10000 # nanosecs

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
# input:          image_raw; /davis/image_raw generator
#                 events; /davis/events generator
#                 frame; image frame to display points on
#                 pts; matrix of line points (topL, botL, topR, botR)
# output:         none; displays in window
def displayEvents( image_raw, events , frame , pts ):

    for eventTopic, msg, eventT in events:

        # create a new empty matrix
        img = frame.copy()

        # store events by mirror
        left = []
        right = []
        center = []

        # fill with event points
        for event in msg.events:
            loc = findMirror(event , pts)

            # choose color
            if loc == LEFT:
                color = RED
                left.append( event )

            elif loc == CENTER:
                color = GREEN
                center.append( event )

                # only correspond from center - side rows too far apart
                correspond( img , event , left , center , right , pts ) 

            elif loc == RIGHT:
                color = BLUE
                right.append( event )

            else:
                color = WHITE # unknown pts
            
            # draw circles
            cv2.circle(img , (event.x , event.y) , 1 , color, -1)

        # upscale raw, event image
        eventImg = upscale( img )
        rawImg = getRawImage( image_raw )
        rawImg = upscale( rawImg )

        # stack with raw image
        imgStack = np.hstack((rawImg, eventImg))

        # display image
        cv2.imshow("Events Gif", imgStack)
        cv2.waitKey(FRAME_DELAY)
        
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
    rightSlope = -( topR[1] - botR[1] ) / ( topR[0] - botR[0] )

    # use point-slope formula to find x value of lines at event y
    leftX  = (((-event.y) - (-botL[1])) / leftSlope ) + botL[0]
    rightX = (((-event.y) - (-botR[1])) / leftSlope ) + botR[0]

    # choose mirror based on x valueif left of left line, then on left mirror
    #if event.x > leftX and event.x < rightX:
    #    return CENTER
    if event.x > rightX:
        return RIGHT
    elif event.x < leftX:
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
    
    '''
    mirrors = []

    # each mirror only needs to look at 1 other, 
    # so order is L->C->R->L->...
    if location == CENTER:
        mirror = right
    elif location == LEFT:
        mirror = center
    elif location == RIGHT:
        mirror = left
    else:
        return
    '''

    # Search +/- 5 rows for corresonding event (approx.)
    for mirror in [left , right]:
        for compare in mirror:
            if compareEvents( event , compare ):
                # draw a line for a match
                cv2.line(img, (event.x , event.y) , (compare.x , compare.y), 
                         WHITE, 1)

######################################################################
# compareEvents() determines if 2 events are correspondant
# input:          a; first event to compare
#                 b; second event to compare
# output:         a bool; true if equal, false if not
def compareEvents( a , b ):
    if ( a.y in range( b.y - SEARCH, b.y + SEARCH )
         and a.polarity == b.polarity
         and abs(a.ts.to_nsec() - b.ts.to_nsec()) <= EVENT_WINDOW):
        return True

    return False

######################################################################
# upscale() scales up an image to a constant specified percent
# input:    img; image to scale up
# output:   resized; scaled up image
def upscale( img ):
    width = int(img.shape[1] * UPSCALE_PERCENT / 100)
    height = int(img.shape[0] * UPSCALE_PERCENT / 100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

#####################################################################
# getRawImage() uses CVBridge to get the raw image from the rosbag
#               (also converts to RGB)
# input:        image_raw; image_raw topic generator
# output:       cv_image; next raw image
def getRawImage( image_raw ):
    topic, msg, t = next(image_raw)
    cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
    return cv_image

def main():
    # NOTE: interesting stuff moreso at frame 200
    bag = rosbag.Bag('../bagfiles/mount_1/calibrate.bag') 
    print(bag)
    print('------------------------------------------------------------\n')

    # get image from generator
    image_raw = bag.read_messages(topics=['/davis/image_raw']) 
    cv_image = getRawImage( image_raw ) 

    # divide image into left, right, center
    frame, pts = pickLines( cv_image )

    # create new generators
    events = bag.read_messages(topics=['/davis/events']) 
    image_raw = bag.read_messages(topics=['/davis/image_raw'])

    # show events, draw correspondences
    displayEvents(image_raw , events , frame, pts)

    # close the bag
    bag.close()

main()
