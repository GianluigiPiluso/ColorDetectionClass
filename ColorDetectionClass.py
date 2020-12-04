#https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

#Version 2.0

# Python program for Detection of a  
# specific color(blue here) using OpenCV with Python 
import cv2 
import numpy as np
import math
import time
from Motor import *
from servo import *
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
class ColorDetectionClass:  
    def __init__(self, WIDTH=320, HEIGHT=240, TYPE_COLOR='ALL', AREA_MAX=5000, MOTOR_DUTY=900):
        #Attributi
        self.WIDTH=int(WIDTH)
        self.HEIGHT=int(HEIGHT)
        self.TYPE_COLOR=TYPE_COLOR
        self.AREA_MAX=int(AREA_MAX)
        self.MOTOR_DUTY=int(MOTOR_DUTY)
        
        self.X_DIVISION=int(self.WIDTH/3)
        self.Y_DIVISION=int(self.HEIGHT/3)
        self.ISTERESI=int(self.X_DIVISION/5)
        self.SOGLIA_AZIONE=int(self.ISTERESI*2.7)

        #Black and White
        if(TYPE_COLOR == 'ALL' or TYPE_COLOR == 'all' or TYPE_COLOR == 'N' or TYPE_COLOR == 'n'):
            self.setRangeBlack()
            self.titoloFrame='Black Line Frame'
            self.titoloMask='Black Line Mask'
            self.titoloBlackLine='Black Line'        
        
        #Red
        if(TYPE_COLOR == 'ALL' or TYPE_COLOR == 'all' or TYPE_COLOR == 'R' or TYPE_COLOR == 'r'):
            self.setRangeRed()
            self.titoloFrameRed='Red Frame'
            self.titoloMaskRed='Red Mask'
            self.titoloResolutionRed='Red Resolution'
        
        #Green
        if(TYPE_COLOR == 'ALL' or TYPE_COLOR == 'all' or TYPE_COLOR == 'G' or TYPE_COLOR == 'g'):
            self.setRangeGreen()
            self.titoloFrameGreen='Green Frame'
            self.titoloMaskGreen='Green Mask'
            self.titoloResolutionGreen='Green Resolution'
        
        #Blue
        if(TYPE_COLOR == 'ALL' or TYPE_COLOR == 'all' or TYPE_COLOR == 'B' or TYPE_COLOR == 'b'):
            self.setRangeBlue()
            self.titoloFrameBlue='Blue Frame'
            self.titoloMaskBlue='Blue Mask'
            self.titoloResolutionBlue='Blue Resolution'
        
        # Webcamera no 0 is used to capture the frames
        cap = cv2.VideoCapture(0)
        self.cap=cap
        self.cap.set(3, self.WIDTH)
        self.cap.set(4, self.HEIGHT)
        print("Cam inizializzata e pronta per l'uso")
        time.sleep(1)
#-------------------------------------------------------------------------------
    def __str__(self):
        s=''
        s='Risoluzione schermo: '+str(self.WIDTH)+'x'+str(self.HEIGHT)+'\n'
        s=s+'Colore: '+self.TYPE_COLOR+' ---> Area massima: '+str(self.AREA_MAX)+'\n'
        s=s+'Duty del motore: '+str(self.MOTOR_DUTY)+'\n'
        #return 'Risoluzione schermo: '+str(self.WIDTH)+'x'+str(self.HEIGHT)+'\n'
        return s
#-------------------------------------------------------------------------------
    def setMotorDuty(self, MOTOR_DUTY):
        self.MOTOR_DUTY=int(MOTOR_DUTY)        
#-------------------------------------------------------------------------------
    def setAreaMax(self, AREA_MAX_RED):
        self.AREA_MAX=int(AREA_MAX)        
#-------------------------------------------------------------------------------
    def setColorType(self, TYPE_COLOR):
        self.TYPE_COLOR=TYPE_COLOR
        if(TYPE_COLOR == 'ALL' or TYPE_COLOR == 'all' or TYPE_COLOR == 'R' or TYPE_COLOR == 'r'):
            self.setRangeRed()
        if(TYPE_COLOR == 'ALL' or TYPE_COLOR == 'all' or TYPE_COLOR == 'G' or TYPE_COLOR == 'g'):
            self.setRangeGreen()            
        if(TYPE_COLOR == 'ALL' or TYPE_COLOR == 'all' or TYPE_COLOR == 'B' or TYPE_COLOR == 'b'):
            self.setRangeBlue()
#-------------------------------------------------------------------------------
    def setRangeBlack(self):
        #lower_black = np.array([165,196,52]) 
        #upper_black = np.array([180,255,255])           
        #self.lowerColor=lower_lower_black
        #self.upperColor=upper_lower_black
        pass
#-------------------------------------------------------------------------------
    def setRangeRed(self):
        self.lowerRed = np.array([165,196,52]) 
        self.upperRed = np.array([180,255,255])           
#-------------------------------------------------------------------------------
    def setRangeGreen(self):
        self.lowerGreen = np.array([50,127,71]) 
        self.upperGreen = np.array([100,255,255])
        #self.lowerGreen = np.array([39,66,100])
        #self.upperGreen = np.array([161,255,255]) 
#-------------------------------------------------------------------------------
    def setRangeBlue(self):
        self.lowerBlue = np.array([85,97,45]) 
        self.upperBlue = np.array([141,255,255])          
#-------------------------------------------------------------------------------
    def setScreenWidht(self, WIDTH):
        self.WIDTH=WIDTH
#-------------------------------------------------------------------------------
    def setScreenHeight(self, HEIGHT):
        self.HEIGHT=HEIGHT
#-------------------------------------------------------------------------------
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
        return
#-------------------------------------------------------------------------------
    def TrovaRosso(self, frameNumber):
        mask = cv2.inRange(self.hsv, self.lowerRed, self.upperRed)
        FrameRed=np.copy(self.frame)
           
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

        print("Cx: ", cX, "   Cy: ", cY, "   Area Max: ", AreaMax, "Rosso")
        radius=int(math.sqrt(AreaMax/3.14))
        
        cv2.circle(FrameRed, (cX, cY), 5, (255, 255, 255), -1)
        cv2.circle(FrameRed, (cX, cY), radius, (255, 255, 255), 2)
        cv2.circle(FrameRed, (int(self.WIDTH/2), int(self.HEIGHT/2)), int(self.X_DIVISION/2), (0, 0, 255), 2) #STOP - Zone
        cv2.circle(FrameRed, (int(self.WIDTH/2), int(self.HEIGHT/2)), self.SOGLIA_AZIONE, (255, 0, 0), 2) #maneuvering zone
        cv2.putText(FrameRed, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) #Area detect
        
        if(frameNumber == 0):
            cv2.imshow('',0)
        if(frameNumber >= 1):
            cv2.imshow(self.titoloFrameRed, FrameRed)
        if(frameNumber >= 2):
            cv2.imshow(self.titoloMaskRed,mask) 
        if(frameNumber == 3):
            res = cv2.bitwise_and(FrameRed,FrameRed, mask=mask)
            cv2.imshow(self.titoloResolutionRed,res)
        self.AzionaMotori(cX, cY, AreaMax)
#-------------------------------------------------------------------------------
    def TrovaVerde(self, frameNumber):
        mask = cv2.inRange(self.hsv, self.lowerGreen, self.upperGreen)
        FrameGreen=np.copy(self.frame)

        # find contours in the binary image
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        AreaMax=1
        cX=0
        cY=0
        for c in contours:
            M = cv2.moments(c)
            #if(M["m00"] > AreaMax and M["m00"] >= self.AREA_MIN_GREEN and M["m00"] < self.AREA_MAX_GREEN):
            if(M["m00"] > AreaMax and M["m00"] >= 1):
                AreaMax=int(M["m00"])
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

        print("Cx: ", cX, "   Cy: ", cY, "   Area Max: ", AreaMax, "Verde")
        radius=int(math.sqrt(AreaMax/3.14))
            
        cv2.circle(FrameGreen, (cX, cY), 5, (255, 255, 255), -1)
        cv2.circle(FrameGreen, (cX, cY), radius, (255, 255, 255), 2)
        cv2.circle(FrameGreen, (int(self.WIDTH/2), int(self.HEIGHT/2)), int(self.X_DIVISION/2), (0, 0, 255), 2) #STOP - Zone
        cv2.circle(FrameGreen, (int(self.WIDTH/2), int(self.HEIGHT/2)), self.SOGLIA_AZIONE, (255, 0, 0), 2) #maneuvering zone
        cv2.putText(FrameGreen, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) #Area detect
        
        if(frameNumber == 0):
            cv2.imshow('',0)
        if(frameNumber >= 1):
            cv2.imshow(self.titoloFrameGreen, FrameGreen)
        if(frameNumber >= 2):
            cv2.imshow(self.titoloMaskGreen,mask) 
        if(frameNumber == 3):
            res = cv2.bitwise_and(FrameGreen, FrameGreen, mask=mask)
            cv2.imshow(self.titoloResolutionGreen,res)
        self.AzionaMotori(cX, cY, AreaMax)
#-------------------------------------------------------------------------------
    def TrovaBlue(self, frameNumber):
        mask = cv2.inRange(self.hsv, self.lowerBlue, self.upperBlue)
        FrameBlue=np.copy(self.frame)
           
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

        print("Cx: ", cX, "   Cy: ", cY, "   Area Max: ", AreaMax, "Blue")
        radius=int(math.sqrt(AreaMax/3.14))
            
        cv2.circle(FrameBlue, (cX, cY), 5, (255, 255, 255), -1)
        cv2.circle(FrameBlue, (cX, cY), radius, (255, 255, 255), 2)
        cv2.circle(FrameBlue, (int(self.WIDTH/2), int(self.HEIGHT/2)), int(self.X_DIVISION/2), (0, 0, 255), 2) #STOP - Zone
        cv2.circle(FrameBlue, (int(self.WIDTH/2), int(self.HEIGHT/2)), self.SOGLIA_AZIONE, (255, 0, 0), 2) #maneuvering zone
        cv2.putText(FrameBlue, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) #Area detect
        
        if(frameNumber == 0):
            cv2.imshow('',0)
        if(frameNumber >= 1):
            cv2.imshow(self.titoloFrameBlue, FrameBlue)
        if(frameNumber >= 2):
            cv2.imshow(self.titoloMaskBlue,mask) 
        if(frameNumber == 3):
            res = cv2.bitwise_and(FrameBlue,FrameBlue, mask=mask)
            cv2.imshow(self.titoloResolutionBlue,res)
        self.AzionaMotori(cX, cY, AreaMax)
#-------------------------------------------------------------------------------
    def TrovaNero(self):
        gray=cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
        #cv2.imshow('Black Line Frame',self.frame)
        cv2.imshow('Black Line Gray',gray)
        cv2.imshow('Black Line Threshold',thresh)
#-------------------------------------------------------------------------------
    def run(self, cap):
        # Captures the live stream frame-by-frame
        ret, self.frame = cap.read()
        # Converts images from BGR to HSV 
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        
        #decide quali colori processare
        if(self.TYPE_COLOR == 'N' or self.TYPE_COLOR == 'ALL'):
            self.TrovaNero()
        if(self.TYPE_COLOR == 'R' or self.TYPE_COLOR == 'ALL'):
            self.TrovaRosso(1)
        if(self.TYPE_COLOR == 'G' or self.TYPE_COLOR == 'ALL'):
            self.TrovaVerde(1)
        if(self.TYPE_COLOR == 'B' or self.TYPE_COLOR == 'ALL'):
            self.TrovaBlue(1)
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

#CercaColore=ColorDetectionClass(WIDTH=640, HEIGHT=480)
CercaColore=ColorDetectionClass() #default 320x240
#CercaColore=ColorDetectionClass(WIDTH=160, HEIGHT=120)
print(CercaColore)
time.sleep(1)
#exit()

#muovi servo per prova
pwm=Servo()
pwm.setServoPwm('0',90, error=10)
pwm.setServoPwm('1',60, error=10)

"""
angleList=[90, 120, 90, 60, 90]
for angolo in angleList:
    pwm.setServoPwm('0',angolo, error=10)
    time.sleep(0.5)
pwm.setServoPwm('1',60, error=10)
"""

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    #try:
    CercaColore.setColorType('ALL')
    while True:
        cap=CercaColore.cap
        CercaColore.run(cap)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    #Ferma i motori
    print("Motori STOP")
    PWM.setMotorModel(0,0,0,0) #Stop
    destroy()
    time.sleep(0.5)
    # Destroys all of the HighGUI windows. 
    cv2.destroyAllWindows() 
    # release the captured frame
    CercaColore.cap.release()
    exit()
    
    """
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
        cv2.destroyAllWindows()
        cap.release()
        exit()
    """
#-------------------------------------------------------------------------------
