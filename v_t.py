from ctypes import sizeof
import cv2 
import mediapipe as mp
import gestdet as gd
import serial 
import time

capture=cv2.VideoCapture(0)

mpHands=mp.solutions.hands
hands=mpHands.Hands()
mpDraw=mp.solutions.drawing_utils
det=gd.gestdet(False)

#ard=serial.Serial('COM3',9600,timeout=1)

def utf8len(s):
    return len(s.encode('utf-8'))

while True:
    isTrue,frame=capture.read()
 
    det.findhand(frame)
    det.assng_fingers()
    
    det.detect_on(frame)
    det.detect_off(frame)
    det.go_signal(frame)
    det.stop_signal(frame)
    det.left_or_right(frame)

    det.draw_speed(frame)
    det.draw_status(frame)

    det.send_data()

    #if results.multi_hand_landmarks:
        #for handLms in  results.multi_hand_landmarks:
           # for id, lm in enumerate(handLms.landmark):
               # h,w,c=frame.shape
                #cx,cy=int(lm.x*w), int(lm.y*h)
               # print(id,cx,cy)
                #if id==1:
                    #cv2.circle(frame,(cx,cy),15,(255,255,0),cv2.FILLED)
                
            #mpDraw.draw_landmarks(frame,handLms, mpHands.HAND_CONNECTIONS)

    cv2.imshow('Vid',frame)
    
    if cv2.waitKey(20) & 0xFF==ord('d'):
        break

capture.release()
cv2.destroyAllWindows
