### HAND CORDINATES: https://google.github.io/mediapipe/solutions/hands.html

import cv2 
import mediapipe as mp
import math
import serial 
import time

#capture=cv2.VideoCapture(0)

class gestdet():
    def __init__ (self, on_off_status):
       
        self.mpHands=mp.solutions.hands
        self.hands=self.mpHands.Hands()
        self.mpDraw=mp.solutions.drawing_utils

        self.on_or_off=on_off_status
        self.v1=85
        self.v2=85
        self.com='S'
        self.ard=serial.Serial('COM3',9600,timeout=1)
        

    def send_data(self):
        fwd='R'

        self.ard.flush()

        self.ard.write(self.com.encode())

    def grad(self,list1,list2, abso_=False):
        #each list has the x and y cordiante
        #this function rturns the gradient
        if len(list1) >1 and  len(list2) >1:
            x1=list1[0]
            y1=list1[1]

            x2=list2[0]
            y2=list2[1]

            dy=y2-y1
            dx=x2-x1

            if(abso_==True):
                dy=abs(y2-y1)
                dx=abs(x2-x1)
            
            m=dy/(dx+1e-7)

        return m

    def y_intercept(self,list1,list2, abso_=False):
        #takes two points in the form of lists, returns
        #y intercept
        if len(list1) >1 and  len(list2) >1:

            m=self.grad(list1,list2,abso_)

            x1=list1[0]
            y1=list1[1]

            x2=list2[0]
            y2=list2[1]

            c=y2-(m*x2)

        return c

    def y_intercep_2(self,x2,y2,m, abso_=False):
        #takes gradient and two point, returns
        #y intercept

        if x2 > 0 and y2 > 0:
  
            c=y2-(m*x2)
        else:
            c=0

        return c

    def intersection_point(self,list1,list2, abso_=False):
        #THE LISTS HERE CONTAIN TWO POINTS IE
        #LIST OF LIST, then returns a list of the 
        #intercetion point
         intercept_pt=[]

         if len(list1) >1 and  len(list2) >1:

             line_1_p1=list1[0]
             line_1_p2=list1[1]

             line_2_p1=list2[0]
             line_2_p2=list2[1]

             l1_p1_x=line_1_p1[0]
             l1_p1_y=line_1_p1[1]

             l2_p1_x=line_2_p1[0]
             l2_p1_y=line_2_p1[1]

             m1=self.grad(line_1_p1,line_1_p2, abso_=False)
             m2=self.grad(line_2_p1,line_2_p2, abso_=False)

             c1=self.y_intercept(line_1_p1,line_1_p2, abso_=False)
             c2=self.y_intercept(line_2_p1,line_2_p2, abso_=False)

             x=((c2-c1)/m1-m2)
             y=(m1*x)+c1

             intercept_pt[0]=x
             intercept_pt[1]=y

         return intercept_pt

    def intersection_point_2(self,m1,m2,c1,c2, abso_=False):
        #Different version using gradient and int_point,
        #then returns a list of the 
        #intersection point
     intercept_pt=[]

     x=((c2-c1)/(m1-m2+1e-7))
     y=(m1*x)+c1

     if x>640:
         x=635
     if y>480:
         y=460

     #print("int_pt",x,y)

     
     intercept_pt.append(x)
     intercept_pt.append(y)

     return intercept_pt
           
    def findhand(self,img):
        imgRGB=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        results=self.hands.process(imgRGB)

        #print(results.multi_hand_landmarks)
        #2D lists for corrdinates of each fingers
        self.hand_cords=[] 
        self.thumb=[]
        self.index=[]
        self.middle=[]
        self.ring=[]
        self.pinky=[]

        if results.multi_hand_landmarks:
            for handLms in  results.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    h,w,c=img.shape
                    cx,cy=int(lm.x*w), int(lm.y*h)
                    #print(id,cx,cy)
                    #cv2.putText(img,str(id),(cx,cy-10),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2)
                    cord=[cx,cy]
                    #print(cx,cy)
                    self.hand_cords.append([cx,cy])
                

                    #if id==4 or id==8 or id==12 or id==16 or id==20:
                        #cv2.circle(img,(cx,cy),15,(255,255,0),cv2.FILLED)
                    #cv2.putText(img,str(cx),(cx,cy),cv2.FONT_HERSHEY_PLAIN,1,(0,55,255),2)
                    #cv2.putText(img,str(cy),(cx,cy-10),cv2.FONT_HERSHEY_PLAIN,1,(255,55,0),2)
                 
            #self.mpDraw.draw_landmarks(img,handLms, self.mpHands.HAND_CONNECTIONS)
            
    def finger_ext(self,list, angl, frame):
        #determines if the figer (list provided) is extended 
        is_ext=False

        if len(list)>3:
            fing_base=list[0]
            fing_2=list[1]
            fing_3=list[2]
            fing_tip=list[3]

            wrist=self.hand_cords[0]

            #cv2.line(frame,wrist,fing_tip,(0,255,255),2,cv2.LINE_AA)

            dist_to_base=math.dist(fing_base,wrist)
            dist_to_lm=math.dist(fing_2,wrist) #dist from wrist to lower mid finger joint
            dist_to_um=math.dist(fing_3,wrist) #dist from wrist to upper mid finger joint
            dist_to_tip=math.dist(fing_tip,wrist)

            ml_l=self.grad(fing_base,fing_2)
            ml_m=self.grad(fing_2,fing_3)
            ml_h=self.grad(fing_3,fing_tip)

            thet_2=abs(math.degrees(math.atan((ml_l-ml_m)/(1+(ml_l*ml_m)+1e-7))))
            thet_3=abs(math.degrees(math.atan((ml_m-ml_h)/(1+(ml_m*ml_h)+1e-7))))

            thet2_i=int(thet_2)
            thet3_i=int(thet_3)

            #cv2.putText(frame,str(thet2_i),fing_2,cv2.FONT_HERSHEY_PLAIN,1,(0,255,255),2)
            #cv2.putText(frame,str(thet3_i),fing_3,cv2.FONT_HERSHEY_PLAIN,1,(255,255,0),2)

            ang=False

            if thet_2 <= angl or thet_3 <= angl:
                ang=True

            if dist_to_tip > dist_to_base and dist_to_um > dist_to_lm and dist_to_tip > dist_to_um and ang==True:
                is_ext=True
        
        return is_ext

    def distance_tip_to_wrist(self,list_1):
        #dist from tip of fing to wrist
        if len(list_1)>3:
            wrist=self.hand_cords[0]
            f_tip=list_1[3]

            dist=math.dist(wrist,f_tip)

        return dist

    def angle_w_wrist(self,list_1,frame,ang):
        #returns true if the angle made between the specified fing tip
        #to wrist and fing base to tip

        if len(list_1)>3:
            fing_base=list_1[0]
            fing_2=list_1[1]
            fing_3=list_1[2]
            fing_tip=list_1[3]
            wrist=self.hand_cords[0]

            fg_tipx=fing_tip[0]
            fg_tipy=fing_tip[1]

            #cv2.line(frame,wrist,fing_base,(255,255,0),2,cv2.LINE_AA)

            fg_basex=fing_base[0]
            fg_basey=fing_base[1]

            wristx=wrist[0]
            wristy=wrist[1]

            t2x=abs(wristx-fg_tipx)
            t2y=abs(wristy-fg_tipy)

            #cv2.line(frame,wrist,fing_tip,(0,255,255),2,cv2.LINE_AA)

            #dy=abs(fg_tipy-fg_basey)
            #dx=abs(fg_tipx-fg_basex)

            dy1=abs(fg_tipy-wristy)
            dx1=abs(fg_tipx-wristx)

            #angle=math.atan2(dy,dx)
            angle1=math.atan2(t2y,t2x)

            #ang_deg=math.degrees(angle)
            ang_deg1=math.degrees(angle1)

            #print("Fing_base",ang_deg)
            #print("wrist",ang_deg1)

            #cv2.putText(frame,str(ang_deg),(330,120),cv2.FONT_HERSHEY_PLAIN,3,(0,255,255),2)
            #cv2.putText(frame,str(ang_deg1),(330,220),cv2.FONT_HERSHEY_PLAIN,3,(255,255,0),2)

            ##See notes for l1,l2,and l3

            m1=self.grad(wrist,fing_base)

            m2=self.grad(wrist,fing_tip)
            c2=self.y_intercept(wrist,fing_tip)

            thet=abs(math.degrees(math.atan((m1-m2)/(1+(m1*m2)+1e-7))))

            #cv2.putText(frame,str(thet),(wristx,wristy-50),cv2.FONT_HERSHEY_PLAIN,3,(0,255,255),2)

            m3=-1*(1/(m1+1e-7))

            c3=self.y_intercep_2(fg_basex,fg_basey,m3)
            #print("wrist",c2)

            ##intersection of l3 and l2
            p_i=self.intersection_point_2(m2,c2,m3,c3)
            #cv2.circle(frame,(600,460),15,(255,255,0),cv2.FILLED)

            ml_l=self.grad(fing_base,fing_2)
            ml_m=self.grad(fing_2,fing_3)
            ml_h=self.grad(fing_3,fing_tip)

            thet_2=abs(math.degrees(math.atan((ml_l-ml_m)/(1+(ml_l*ml_m)+1e-7))))
            thet_3=abs(math.degrees(math.atan((ml_m-ml_h)/(1+(ml_m*ml_h)+1e-7))))

            ang_c=False

            if thet <= ang:
                ang_c=True

            #cv2.putText(frame,str(thet_2),fing_2,cv2.FONT_HERSHEY_PLAIN,1,(0,75,255),2)
            #cv2.putText(frame,str(thet_3),fing_3,cv2.FONT_HERSHEY_PLAIN,1,(85,200,10),2)
            return ang_c
           
    def palm_wrt_wrist(self,frame, ang):

        if len(self.hand_cords) >0:
            wrist=self.hand_cords[0] #cords of wrist
            middle=self.middle[0]  #cords of base of middle fing

            dx=abs(wrist[0]-middle[0])
            dy=abs(wrist[1]-middle[1])

            thet=math.degrees(math.atan(dy/(dx+1e-7)))

            #cv2.putText(frame,str(thet),middle,cv2.FONT_HERSHEY_PLAIN,3,(0,255,55),2)

            ang_c=False
            
            if thet >= ang:
                ang_c=True

            return ang_c
    
    def palm_to_wrist_ang(self,list):

        if len(self.hand_cords) >0:
            wrist=self.hand_cords[0] #cords of wrist
            fing=list[0]  #cords of base of middle fing

            dx=abs(wrist[0]-fing[0])
            dy=abs(wrist[1]-fing[1])

            thet=math.degrees(math.atan(dy/(dx+1e-7)))

            #cv2.putText(frame,str(thet),middle,cv2.FONT_HERSHEY_PLAIN,3,(0,255,55),2)

            return thet
            
    def two_fingers_dist(self,list_1,list_2):
        #Finds dist between two finger tips
        if len(list_1)>2 and len(list_2)>2:
            t_1=list_1[3]
            t_2=list_2[3]

            lr=math.dist(t_1,t_2)

        return lr
   
    def assng_fingers(self):
        #the cordinates at the eand of each
        # self.hand_cords[0] is the coordinates of wrist
        #tip of figer is the last element of list
       if len(self.hand_cords) >0:
            
            for i in  range(1,5):
               self.thumb.append(self.hand_cords[i])
               #for x in self.thumb:
                   #print(i,x)
            #print(self.thumb[0])
            for j in  range(5,9):
                self.index.append(self.hand_cords[j])
            #print(self.index[3])
            for k in  range(9,13):
                self.middle.append(self.hand_cords[k])
            #print(self.middle[3])
            for l in  range(13,17):
                self.ring.append(self.hand_cords[l])
            for m in  range(17,21):
                self.pinky.append(self.hand_cords[m])

    def detect_on(self, frame):

        if len(self.hand_cords) >0:
            middle_ext=self.finger_ext(self.middle,15,frame)
            ring_ext=self.finger_ext(self.ring,15,frame)
            pinky_ext=self.finger_ext(self.pinky,20,frame)
            
            ang_mid=self.angle_w_wrist(self.middle,frame,20) #10 deg
            ang_ring=self.angle_w_wrist(self.ring,frame,20) #10 deg
            ang_pinky=self.angle_w_wrist(self.pinky,frame,25) #15 deg

            index_ext=self.finger_ext(self.index,15,frame)

            #print("INDEX ",index_ext)

            #ring_ext=self.finger_ext(self.ring)
            #pinky_ext=self.finger_ext(self.pinky)
    
            f_t_m=self.middle[3]
            f_t_r=self.ring[3]
            f_t_p=self.pinky[3]

            mid_fing_base=self.middle[0]

            palm_thet=self.palm_wrt_wrist(frame,60)

            ext=False
            ang_ext=False
            palm_t=False
           

            if middle_ext==True and ring_ext==True and pinky_ext ==True and index_ext==False:
                ext=True

            if ang_mid==True and ang_ring==True and ang_pinky ==True:
                ang_ext=True

            if palm_thet==True:
                palm_t=True

            dist=self.two_fingers_dist(self.index,self.thumb)

            dist_i=int(dist)

            #print(dist)

            cv2.circle(frame,mid_fing_base,3,(255,255,0),cv2.FILLED)

            if ext==True and  palm_t==True and dist_i <=50:
                self.on_or_off=True
                #cv2.putText(frame,"ON",(320,100),cv2.FONT_HERSHEY_PLAIN,3,(55,255,0),2)
            #else:
               # cv2.putText(frame,"OFF",(320,100),cv2.FONT_HERSHEY_PLAIN,3,(55,0,255),2)


            """
            ang_ext==True and

            if middle_ext==True:
                cv2.putText(frame,"Extended",f_t_m,cv2.FONT_HERSHEY_PLAIN,1,(0,255,55),2)

            if ring_ext==True:
                cv2.putText(frame,"Extended",f_t_r,cv2.FONT_HERSHEY_PLAIN,1,(255,6,55),2)

            if pinky_ext==True:
                cv2.putText(frame,"Extended",f_t_p,cv2.FONT_HERSHEY_PLAIN,1,(6,6,255),2)
            """
            

            #print(middle_ext,ring_ext,pinky_ext) and dist_i <=50


            #dist=self.two_fingers_dist(self.index,self.thumb)

            #if middle_ext==True and ring_ext==True and  pinky_ext==True and dist <=50:

                #cv2.putText(frame,"OK",(330,120),cv2.FONT_HERSHEY_PLAIN,3,(0,255,55),2)
  
    def detect_off(self,frame):
        #index and pinky extended,
        #middle and ring not extended
        #tip of thumb dist to wrist (dw) >
        #tip of middle & ring  dw
        if len(self.hand_cords) >0:

            index_ext=self.finger_ext(self.index,15,frame)
            middle_ext=self.finger_ext(self.middle,15,frame)
            ring_ext=self.finger_ext(self.ring,15,frame)
            pinky_ext=self.finger_ext(self.pinky,20,frame)

            dist_mid=self.distance_tip_to_wrist(self.middle)
            dist_rin=self.distance_tip_to_wrist(self.ring)
            dist_t=self.distance_tip_to_wrist(self.thumb)

            fing_ext= False
            fing_dist =False

            if index_ext==True and middle_ext==False and ring_ext==False and pinky_ext==True:
                fing_ext= True
            if dist_mid <= dist_t and dist_rin <= dist_t:
                fing_dist =True

            #print("fing_ext",fing_ext,"fing_dist",fing_dist)

            if fing_ext ==True and fing_dist ==True:
                self.on_or_off=False
    
    def go_signal(self,frame):
        if len(self.hand_cords) >0 and self.on_or_off==True:

            index_ext=self.finger_ext(self.index,15,frame)
            middle_ext=self.finger_ext(self.middle,15,frame)
            ring_ext=self.finger_ext(self.ring,15,frame)
            pinky_ext=self.finger_ext(self.pinky,20,frame)

            dist_pinky=self.distance_tip_to_wrist(self.pinky)
            dist_t=self.distance_tip_to_wrist(self.thumb)

            fing_ext= False
            fing_dist =False

            if index_ext==True and middle_ext==True and ring_ext==True and pinky_ext==False:
                fing_ext= True
            if dist_pinky <= dist_t:
                fing_dist =True

            #print("fing_ext",fing_ext,"fing_dist",fing_dist)

            if fing_ext ==True and fing_dist ==True:
                #Check this not sure
                self.com='F'
                self.v1=180
                self.v2=0

    def stop_signal(self, frame):
        if len(self.hand_cords) >0 and self.on_or_off==True:

            index_ext=self.finger_ext(self.index,15,frame)
            middle_ext=self.finger_ext(self.middle,15,frame)
            ring_ext=self.finger_ext(self.ring,15,frame)
            pinky_ext=self.finger_ext(self.pinky,20,frame)

            dist_pinky=self.distance_tip_to_wrist(self.pinky)
            dist_t=self.distance_tip_to_wrist(self.thumb)
            dist_mid=self.distance_tip_to_wrist(self.middle)

            fing_ext= False
            fing_dist =False

            if index_ext==False and middle_ext==False and ring_ext==False and pinky_ext==True:
                fing_ext= True
            if dist_pinky >= dist_mid and dist_t >= dist_mid:
                fing_dist =True

            print("fing_ext",fing_ext,"fing_dist",fing_dist)

            if fing_ext ==True and fing_dist ==True:
                #Check this not sure
                self.com='S'
                self.v1=85
                self.v2=85
    
    def left_or_right(self,frame):
        if len(self.hand_cords) >0 and self.on_or_off==True:

            index_ext=self.finger_ext(self.index,15,frame)
            middle_ext=self.finger_ext(self.middle,15,frame)
            ring_ext=self.finger_ext(self.ring,15,frame)
            pinky_ext=self.finger_ext(self.pinky,20,frame)

            dist_ring=self.distance_tip_to_wrist(self.ring)
            dist_pinky=self.distance_tip_to_wrist(self.pinky)
            dist_thumb=self.distance_tip_to_wrist(self.thumb)

            mid_fing=self.middle[0]
            wrist=self.hand_cords[0]

            mid_fing_x=mid_fing[0]
            wrist_x=wrist[0]

            x_dif=mid_fing_x-wrist_x

            wr_thet=self.palm_to_wrist_ang(self.middle)

            fing_ext= False
            fing_dist =False

            if index_ext==True and middle_ext==True and ring_ext==False and pinky_ext==False:
                fing_ext= True

            if dist_thumb >= dist_ring and dist_thumb >= dist_ring:
                fing_dist =True

            if fing_ext == True and fing_dist == True and wr_thet >=0 and wr_thet <=25 and x_dif >15:
                #check this, turn left
                self.com='L'
                self.v1=0
                self.v2=180

            if fing_ext == True and fing_dist == True and wr_thet >=0 and wr_thet <=25 and x_dif <-15:
                #check this, turn right
                self.com='R'
                self.v1=0
                self.v2=0

            #print("fing_ext",fing_ext,"fing_dist",fing_dist)
            #print("wrist ang",wr_thet,"x_diff",x_dif)
            
    def draw_speed(self, frame):
        cv2.putText(frame,"V1",(15,75),cv2.FONT_HERSHEY_PLAIN,2,(55,255,100),2)
        cv2.putText(frame,"V2",(130,75),cv2.FONT_HERSHEY_PLAIN,2,(55,255,100),2)

        if self.v1==180 and self.v2==0:
            cv2.putText(frame,"GO",(73,100),cv2.FONT_HERSHEY_PLAIN,1,(55,255,100),2)
        
        if self.v1==85 and self.v2==85:
            cv2.putText(frame,"STOP",(73,100),cv2.FONT_HERSHEY_PLAIN,1,(5,0,255),2)

        if self.v1==0 and self.v2==180:
            cv2.putText(frame,"LEFT",(73,100),cv2.FONT_HERSHEY_PLAIN,1,(175,0,200),2)
        
        if self.v1==0 and self.v2==0:
            cv2.putText(frame,"RIGHT",(73,100),cv2.FONT_HERSHEY_PLAIN,1,(175,0,200),2)

        cv2.putText(frame,str(self.v1),(15,115),cv2.FONT_HERSHEY_PLAIN,2,(55,255,100),2)
        cv2.putText(frame,str(self.v2),(130,115),cv2.FONT_HERSHEY_PLAIN,2,(55,255,100),2)

    def draw_status(self,frame):

        if self.on_or_off ==True:
            cv2.putText(frame,"ON",(520,100),cv2.FONT_HERSHEY_PLAIN,3,(55,255,0),2)
        else:
            cv2.putText(frame,"OFF",(520,100),cv2.FONT_HERSHEY_PLAIN,3,(55,0,255),2)

        print("Status",self.com)




        



    
    

           



        






