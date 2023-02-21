import cv2
import mediapipe
import time
import numpy as np
import handtrackingmodule as htm
import math
import pycaw
from ctypes import cast, POINTER, pointer
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume

#####################################
wCam = 640
hCam = 480
#####################################

cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)
Ptime = 0

detector = htm.handDetector(maxHands=1)


devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
# volume.GetMute()
# volume.GetMasterVolumeLevel()
volRangge = volume.GetVolumeRange()
# volume.SetMasterVolumeLevel(-20,0,None)
minVol = volRangge[0]
maxVol = volRangge[1]
volBar = 400
vol = 0
volPer = 0
area = 0
colorvolume = (255,0,0)

while True:
    # find hand
    success, img=cap.read()
    img = detector.findHands(img)
    lmList, bbox = detector.findPosition(img, draw=True)
    if len(lmList) !=0:   
        
        # filter based on size
        area  = (bbox[2]-bbox[0]) * (bbox[3]-bbox[1])//100
        # print(area)
        if 200 < area < 1200:

            # find distance between index and thumb
            length, img, lineinfo = detector.findDistance(4,8,img)
            # print(length)

            # convert volume
            
            volBar = np.interp(length,(50,300),[400,150])
            volPer = np.interp(length,(50,300),[0,100])


            # reduce resolution to make it smoother
            smoothness = 10
            volPer = smoothness * round(volPer/smoothness)

            # check finger up
            fingers = detector.fingersUp()
            # print(fingers)

            # if pinky is down set volume
            if not fingers[4]:
                volume.SetMasterVolumeLevelScalar(volPer/100,None)
                cv2.circle(img, (lineinfo[4],lineinfo[5]), 15, (0,255,0), cv2.FILLED)
                colorvolume = (0,255,0)
            else:
                colorvolume = (255,0,0)
                

    # drawings
    cv2.rectangle(img, (50,150), (85,400), (0,255,0), 3)
    cv2.rectangle(img, (50,int(volBar)), (85,400), (0,255,0), cv2.FILLED)
    cv2.putText(img,str(int(volPer)),(40,450),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),3)
    cVol = int(volume.GetMasterVolumeLevelScalar()*100)
    cv2.putText(img, f'Vol Set : {int(cVol)}',(400,450),cv2.FONT_HERSHEY_COMPLEX,1,colorvolume,3)
    # frame rate
    Ctime = time.time()
    fps = 1/(Ctime-Ptime)
    Ptime = Ctime

    cv2.putText(img,str(int(fps)),(10,70),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),3)

    cv2.imshow("Img",img)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
            break