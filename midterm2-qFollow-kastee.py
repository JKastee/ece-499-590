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
import math

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0
n=0
kp = 2
ki = 0.1
kd = 10
best_cnt =0
best_cnt1 =0
pe =0

i=0
j=0

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    imgL = np.zeros((newx,newy,3), np.uint8)
    imgR = np.zeros((newx,newy,3), np.uint8)
    c_image = imgL.copy()
    c_image = imgR.copy()
    vidL = cv2.resize(c_image,(newx,newy))
    vidR = cv2.resize(c_image,(newx,newy))
    [status, framesize] = vl.get(vidL, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidL,(nx,ny))
        imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_L", imgL)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )
    [status, framesize] = vr.get(vidR, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidR,(nx,ny))
        imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_R", imgR)
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
#-----------------------------------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # imgL       = cv image in BGR format (Left Camera)
    # imgR       = cv image in BGR format (Right Camera)
    

    ts=tim.sim[0]

    start=time.clock()


    # Commands Robot
    r.put(ref)
    
    hsvR=cv2.cvtColor(imgR,cv2.COLOR_BGR2HSV)  
    hsvL=cv2.cvtColor(imgL,cv2.COLOR_BGR2HSV)


    lower = np.array([50,100,100], dtype=np.uint8)

    upper = np.array([70,255,255], dtype=np.uint8)



    maskL = cv2.inRange(hsvL, lower, upper)
    maskR = cv2.inRange(hsvR, lower, upper)

    cv2.imshow('maskL',maskL)
    cv2.imshow('maskR',maskR)



    contours,hierarchy = cv2.findContours (maskL,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    contours1,hierarchy1 = cv2.findContours (maskR,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    max_areaL = 0

    max_areaR = 0



    # filtering the effect of noise by using only the biggest blob

    # which is the ball    

    for blob in contours:

	areaL = cv2.contourArea(blob)

	if areaL > max_areaL:

	    max_areaL = areaL
            best_cnt = blob
            n=1

    
    for blob1 in contours1:
        areaR = cv2.contourArea(blob1)
        if areaR > max_areaR:
            max_areaR = areaR
            best_cnt1 = blob1
            n=n+1

    if(n==2):  
      M = cv2.moments(best_cnt)
      M1 = cv2.moments(best_cnt1)
      cx = int(M['m10']/M['m00'])
      cx1 = int(M1['m10']/M1['m00']) 
      dist=(0.4*320)/(2*math.tan(0.523)*(cx-cx1))
      print 'dist',dist
 
      #PID Controller for tracking the object
      e = 160 - cx
      p = e*kp
      i = (e + i)*ki  
      d = (e - pe)*kd
    
      pid = p+i+d


#      if (dist>4):
 #        ref.ref[0]= 0.7
  #       ref.ref[1]= 0.7    
#	 r.put(ref);
      if(cx!=160):
         ref.ref[0] = pid/200 
         ref.ref[1] = - pid/200
	 r.put(ref);
	 if (dist<=4):
	   ref.ref[0] = 0
           ref.ref[1] = 0
           r.put(ref);
      pe = e	
           
      ref.ref[0] = 0.5
      ref.ref[1] = 0.5
      r.put(ref);

      #PID Controller for foowing the object
      
       #get sim times and calculate difference+error
    ts2=tim.sim[0]
    difference=abs(ts-ts2) 
    while (0.1 > difference):   
        [status, framesize] = t.get(tim, wait=False, last=True)
        ts2=tim.sim[0]
        difference=abs(ts-ts2)
             

    # Sleeps
    time.sleep(0.1)   

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
