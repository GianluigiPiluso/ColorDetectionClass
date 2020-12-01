#https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

# Python program for Detection of a  
# specific color(blue here) using OpenCV with Python 
import cv2 
import numpy as np
import math
import time
from Motor import *
from servo import *
#-------------------------------------------------------------------------------
def AzionaMotori(self,Cx, Cy, Area):
    #print(" ---> CentroX: ", Cx, "   CentroY: ", Cy, "   Area: ", Area, "   Area Max Red: ", AREA_MAX_RED)
    #definiamo i campi di azione
    #1° quadrante
    #if(Cx > 0 and Cx < (X_DIVISION+ISTERESI) and Cy > 0 and Cy < HEIGHT and Area < AREA_MAX_COLORE):
    if(Cx > 0 and Cx < (X_DIVISION+ISTERESI) and Cy > 0 and Cy < HEIGHT):
        #Motori a SX
        print("Motori a Sx")
        PWM.setMotorModel(-MOTOR_DUTY,-MOTOR_DUTY,MOTOR_DUTY,MOTOR_DUTY) #Left
    
    #2° quadrante
    if(Cx > X_DIVISION and Cx < int(X_DIVISION*2) and Cy > 0 and Cy < HEIGHT and Area > AREA_MAX_COLORE):
        #Motori Stop
        print("Motori STOP")
        #cv2.circle(frame, (int(WIDTH/2), int(HEIGHT/2)), int((WIDTH/3-ISTERESI)/2), (255, 0, 255), 2) #STOP - Zone
        PWM.setMotorModel(0,0,0,0) #Stop
    
    #3° quadrante    
    #if(Cx > (213-ISTERESI) and Cx < WIDTH and Cy > 0 and Cy < HEIGHT and Area < AREA_MAX_COLORE):
    if(Cx > (213-ISTERESI) and Cx < WIDTH and Cy > 0 and Cy < HEIGHT):
        #Motori a DX
        print("Motori a Dx")
        PWM.setMotorModel(MOTOR_DUTY,MOTOR_DUTY,-MOTOR_DUTY,-MOTOR_DUTY) #Right         
  
    """
    #4° quadrante    
    #if(Cx >= 0 and Cx <= WIDTH and Cy >=160 and Cy <= HEIGHT):
    if(Cx >= 106 and Cx < 213 and Cy >= 0 and Cy <= HEIGHT and Area >= AREA_MAX_COLORE):
        #Motori Indietro
        print("Motori Indietro")
        PWM.setMotorModel(-1000,-1000,-1000,-1000) #Back
        time.sleep(20/1000)
        PWM.setMotorModel(0,0,0,0) #Stop
        time.sleep(10/1000)
    """

    if(Cx > (X_DIVISION+ISTERESI) and Cx < (int(X_DIVISION*2)-ISTERESI) and Cy > 0 and Cy < HEIGHT and Area < AREA_MAX_COLORE):
        #Motori Avanti
        print("Motori Avanti")
        PWM.setMotorModel(MOTOR_DUTY,MOTOR_DUTY,MOTOR_DUTY,MOTOR_DUTY) #Forward
    return
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
class ColorDetectionClass:   
    def __init__(self, WIDTH=320, HEIGHT=240, TYPE_COLOR='G', AREA_MAX=5000, MOTOR_DUTY=900, frameNumber=1):
        #variabili globali
        self.WIDTH=int(WIDTH)
        self.HEIGHT=int(HEIGHT)
        self.TYPE_COLOR=TYPE_COLOR
        self.AREA_MAX=int(AREA_MAX)        
        self.MOTOR_DUTY=int(MOTOR_DUTY)
        self.frameNumber=frameNumber
        
        self.X_DIVISION=int(self.WIDTH/3)
        self.Y_DIVISION=int(self.HEIGHT/3)
        self.ISTERESI=int(self.X_DIVISION/5)
        self.SOGLIA_AZIONE=int(self.ISTERESI*2.7)        
            
        if(TYPE_COLOR == 'R' or TYPE_COLOR == 'r'):
            self.setRangeRed()
            self.titoloFrame='Red Frame'
            self.titoloMask='Red Mask'
            self.titoloResolution='Red Resolution'
           
        if(TYPE_COLOR == 'G' or TYPE_COLOR == 'g'):
            self.setRangeGreen()
            self.titoloFrame='Green Frame'
            self.titoloMask='Green Mask'
            self.titoloResolution='Green Resolution'
        
        if(TYPE_COLOR == 'B' or TYPE_COLOR == 'b'):
            self.setRangeBlue()
            self.titoloFrame='Blue Frame'
            self.titoloMask='Blue Mask'
            self.titoloResolution='Blue Resolution'
            
    def __str__(self):
        s=''
        s='Risoluzione schermo: '+str(self.WIDTH)+'x'+str(self.HEIGHT)+'\n'
        s=s+'Colore: '+self.TYPE_COLOR+' ---> Area massima: '+str(self.AREA_MAX)+'\n'
        s=s+'Duty del motore: '+str(MOTOR_DUTY)+'\n'
        #return 'Risoluzione schermo: '+str(self.WIDTH)+'x'+str(self.HEIGHT)+'\n'
        return s
        
    def setMotorDuty(self, MOTOR_DUTY):
        self.MOTOR_DUTY=int(MOTOR_DUTY)        
        
    def setAreaMax(self, AREA_MAX_RED):
        self.AREA_MAX=int(AREA_MAX)        
             
    def setColorType(self, TYPE_COLOR):
        self.TYPE_COLOR=TYPE_COLOR
        if(TYPE_COLOR == 'R' or TYPE_COLOR == 'r'):
            self.setRangeRed()
        if(TYPE_COLOR == 'G' or TYPE_COLOR == 'g'):
            self.setRangeGreen()            
        if(TYPE_COLOR == 'B' or TYPE_COLOR == 'b'):
            self.setRangeBlue()
            
    def setRangeRed(self):
        lower_red = np.array([165,196,52]) 
        upper_red = np.array([180,255,255])           
        self.lowerColor=lower_red
        self.upperColor=upper_red
        
    def setRangeGreen(self):
        lower_green = np.array([50,127,71]) 
        upper_green = np.array([100,255,255])             
        self.lowerColor=lower_green
        self.upperColor=upper_green     

    def setRangeBlue(self):
        lower_blue = np.array([85,97,45]) 
        upper_blue = np.array([141,255,255])          
        self.lowerColor=lower_blue
        self.upperColor=upper_blue

    def setScreenWidht(self, WIDTH):
        self.WIDTH=WIDTH

    def setScreenHeight(self, HEIGHT):
        self.HEIGHT=HEIGHT
        
    def AzionaMotori(self, Cx, Cy, Area):
        #print(" ---> CentroX: ", Cx, "   CentroY: ", Cy, "   Area: ", Area, "   Area Max Red: ", self.AREA_MAX)
        #definiamo i campi di azione
        #1° quadrante
        #if(Cx > 0 and Cx < (self.X_DIVISION+self.ISTERESI) and Cy > 0 and Cy < self.HEIGHT and Area < self.AREA_MAX):
        if(Cx > 0 and Cx < (self.X_DIVISION+self.ISTERESI) and Cy > 0 and Cy < self.HEIGHT):
            #Motori a SX
            print("Motori a Sx")
            PWM.setMotorModel(-self.MOTOR_DUTY,-self.MOTOR_DUTY,self.MOTOR_DUTY,self.MOTOR_DUTY) #Left
    
        #2° quadrante
        if(Cx > self.X_DIVISION and Cx < int(self.X_DIVISION*2) and Cy > 0 and Cy < self.HEIGHT and Area > self.AREA_MAX):
            #Motori Stop
            print("Motori STOP")
            #cv2.circle(frame, (int(WIDTH/2), int(HEIGHT/2)), int((WIDTH/3-ISTERESI)/2), (255, 0, 255), 2) #STOP - Zone
            PWM.setMotorModel(0,0,0,0) #Stop
    
        #3° quadrante    
        #if(Cx > (213-self.ISTERESI) and Cx < self.WIDTH and Cy > 0 and Cy < self.HEIGHT and Area < self.AREA_MAX):
        if(Cx > (213-self.ISTERESI) and Cx < self.WIDTH and Cy > 0 and Cy < self.HEIGHT):
            #Motori a DX
            print("Motori a Dx")
            PWM.setMotorModel(self.MOTOR_DUTY,self.MOTOR_DUTY,-self.MOTOR_DUTY,-self.MOTOR_DUTY) #Right         

        if(Cx > (self.X_DIVISION+self.ISTERESI) and Cx < (int(self.X_DIVISION*2)-self.ISTERESI) and Cy > 0 and Cy < self.HEIGHT and Area < self.AREA_MAX):
            #Motori Avanti
            print("Motori Avanti")
            PWM.setMotorModel(self.MOTOR_DUTY,self.MOTOR_DUTY,self.MOTOR_DUTY,self.MOTOR_DUTY) #Forward
            
    def run(self):
        # Webcamera no 0 is used to capture the frames 
        cap = cv2.VideoCapture(0)
        cap.set(3, self.WIDTH)
        cap.set(4, self.HEIGHT)
        while True:
            # Captures the live stream frame-by-frame
            ret, frame = cap.read()
            # Converts images from BGR to HSV 
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Here we are defining range of bluecolor in HSV 
            # This creates a mask of blue coloured  
            # objects found in the frame.
            mask = cv2.inRange(hsv, self.lowerColor, self.upperColor)
   
            # find contours in the binary image
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
            AreaMax=1
            cX=0
            cY=0
            for c in contours:
                M = cv2.moments(c)
                if(M["m00"] > AreaMax and M["m00"] >= 1):
                    AreaMax=int(M["m00"])
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

            print("Cx: ", cX, "   Cy: ", cY, "   Area Max: ", AreaMax)
            radius=int(math.sqrt(AreaMax/3.14))
            
            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv2.circle(frame, (cX, cY), radius, (0, 255, 0), 2)
            cv2.circle(frame, (int(self.WIDTH/2), int(self.HEIGHT/2)), int(self.X_DIVISION/2), (0, 0, 255), 2) #STOP - Zone
            cv2.circle(frame, (int(self.WIDTH/2), int(self.HEIGHT/2)), self.SOGLIA_AZIONE, (255, 0, 0), 2) #maneuvering zone
            cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) #Area detect
            # The bitwise and of the frame and mask is done so  
            # that only the blue coloured objects are highlighted  
            # and stored in res 
            #res = cv2.bitwise_and(frame,frame, mask=mask)
            #cv2.imshow('Frame', frame)
            
            if(self.frameNumber == 0):
                cv2.imshow('',0)
            if(self.frameNumber >= 1):
                cv2.imshow(self.titoloFrame, frame)
            if(self.frameNumber >= 2):
                cv2.imshow(self.titoloMask,mask) 
            if(self.frameNumber == 3):
                res = cv2.bitwise_and(frame,frame, mask=mask)
                cv2.imshow(self.titoloResolution,res)
    
            self.AzionaMotori(cX, cY, AreaMax)
            
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

        #Ferma i motori
        print("Motori STOP")
        PWM.setMotorModel(0,0,0,0) #Stop
        destroy()
        time.sleep(1)

        # Destroys all of the HighGUI windows. 
        cv2.destroyAllWindows() 
        # release the captured frame 
        cap.release()
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
TrovaRosso=ColorDetectionClass(TYPE_COLOR='R', frameNumber=0)
TrovaVerde=ColorDetectionClass(TYPE_COLOR='G',  frameNumber=2)
TrovaBlue=ColorDetectionClass(TYPE_COLOR='B', frameNumber=3)
#print(TrovaRosso)

#muovi servo per prova
"""
pwm=Servo()
pwm.setServoPwm('0',90, error=20)
pwm.setServoPwm('1',90, error=10)
"""

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        TrovaRosso.run()
        destroy()
        TrovaVerde.run()
        destroy()
        TrovaBlue.run()
        destroy()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        exit()
#-------------------------------------------------------------------------------
