#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0
j=0

f = open('samples', 'wr')
f1 = open('error_samples', 'wr')

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format

    ts=tim.sim[0]
    ref.ref[0] = -0.5
    ref.ref[1] = 0.5
    start=time.clock()

    # Sets reference to robot
    r.put(ref);

    # Sleeps
    #time.sleep(0.1)   

    # Convert BGR to HSV, define range of colors
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([110, 50, 50], dtype=np.uint8)
    upper_blue = np.array([130,255,255], dtype=np.uint8)
    # define range of green
    lower_green = np.array([40, 50, 50], dtype=np.uint8)
    upper_green = np.array([70,255,255], dtype=np.uint8)
    # define range of red
    lower_red = np.array([0, 50, 50], dtype=np.uint8)
    upper_red = np.array([10,255,255], dtype=np.uint8)
    # Threshold the HSV image to get only 1 color
    maskb = cv2.inRange(hsv, lower_blue, upper_blue)
    maskg = cv2.inRange(hsv, lower_green, upper_green)
    maskr = cv2.inRange(hsv, lower_red, upper_red)
   # convert back to original color - not working
    #maskb_bgr = cv2.cvtColor(maskb, cv2.COLOR_HSV2BGR)
    #cv2.imshow('Blue mask',maskb_bgr)

    # set flag to 0 first time in while loop
    if i==0:
        flag=0
        i=i+1
        print "i equals", i

    #gets moment
    moments1 = cv2.moments(maskb)
    moments2 = cv2.moments(maskg)
    moments3 = cv2.moments(maskr)
    if moments1['m00'] > 500000:
        moments=moments1
        frame1 = cv2.rectangle(maskb,(300,100),(400,500),(0,255,0))
        frame2 = maskb[100:500,300:400]
        cv2.imshow('Mask',maskb)
    elif moments2['m00'] > 500000:
        moments=moments2
        frame1 = cv2.rectangle(maskg,(300,100),(400,500),(0,255,0))
        frame2 = maskg[100:500,300:400]
        cv2.imshow('Mask',maskg)
    else:
        moments = moments3
        frame1 = cv2.rectangle(maskr,(300,100),(400,500),(0,255,0))
        frame2 = maskr[100:500,300:400]
        cv2.imshow('Mask',maskr)
    #draws rectangle frame


    #check if color is in center of frame using moments, and display moment. 
    #If in center stop, then look for next color   
    #use moment to check if entered frame, timer to stop when in center
    #red
    if(moments3['m00'] > 0 and flag==0):
        print 'case 1 red found'        
        flag = 1
        print "move it to center"
        time.sleep(1.1)
        ref.ref[0] = 0
        ref.ref[1] = 0  
        r.put(ref);
        print "keeping it at center for 10 seconds, then searching for green"  
        time.sleep(10) #keep it at center for 10 seconds
        ref.ref[0] = -0.5
        ref.ref[1] = 0.5
        r.put(ref);
        time.sleep(.1)
    #green
    elif(moments2['m00'] > 0 and flag==1):
        print 'case 2 green found'        
        flag = 2 
        print "move it to center"
        time.sleep(1.1)
        ref.ref[0] = 0
        ref.ref[1] = 0  
        r.put(ref);
        print "keeping it at center for 10 seconds, then searching for blue"  
        time.sleep(10) #keep it at center for 10 seconds
        ref.ref[0] = -0.5
        ref.ref[1] = 0.5
        r.put(ref);
        time.sleep(.1)
   #blue
    elif(moments1['m00'] > 0 and flag==2):
        print 'case 3 blue found'

        flag = 0
        print "move it to center"
        time.sleep(1.1)
        ref.ref[0] = 0
        ref.ref[1] = 0  
        r.put(ref);
        print "keeping it at center for 10 seconds, then searching for red"  
        time.sleep(10) #keep it at center for 10 seconds
        ref.ref[0] = -0.5
        ref.ref[1] = 0.5
        r.put(ref);
    else:   #next color not found
        print 'looking for next color'       
        print "m", moments['m00'] 

    #get sim times and calculate difference+error
    ts2=tim.sim[0]
    difference=abs(ts-ts2) 
    while (0.1 > difference):   
        [status, framesize] = t.get(tim, wait=False, last=True)
        ts2=tim.sim[0]
        difference=abs(ts-ts2)

    error=abs(difference-0.1)
    f.write(str(difference))
    f.write('\n')
    f1.write(str(error))
    f1.write('\n')
    print "writing to files"

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------

