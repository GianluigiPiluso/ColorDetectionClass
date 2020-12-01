#https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

# Python program for Detection of a  
# specific color(blue here) using OpenCV with Python 
import cv2 
import numpy as np
import math
import time
from Motor import *

#variabili globali
MOTOR_DUTY=900
AREA_MAX_RED=5000
AREA_MAX_GREEN=5000
AREA_MAX_BLUE=5000



def AzionaMotori(Cx, Cy, Area):
    #print(" ---> CentroX: ", Cx, "   CentroY: ", Cy, "   Area: ", Area, "   Area Max Red: ", AREA_MAX_RED)
    #definiamo i campi di azione
    #1° quadrante
    if(Cx > 0 and Cx < (X_DIVISION+ISTERESI) and Cy > 0 and Cy < HEIGHT and Area < AREA_MAX_COLORE):
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
    if(Cx > (213-ISTERESI) and Cx < WIDTH and Cy > 0 and Cy < HEIGHT and Area < AREA_MAX_COLORE):
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



while(1):
    risoluzioneSchermo = input('Inserisci la risoluzione dello schermo\n 1 -> 320x240\n 2 -> 640x480\n: ')
    if(risoluzioneSchermo == '1'):
        WIDTH=320
        HEIGHT=240
        break
    if(risoluzioneSchermo == '2'):
        WIDTH=640
        HEIGHT=480
        break

X_DIVISION=int(WIDTH/3)
Y_DIVISION=int(HEIGHT/3)

ISTERESI=int(X_DIVISION/5)
SOGLIA_AZIONE=int(ISTERESI*2.7)

# Webcamera no 0 is used to capture the frames 
cap = cv2.VideoCapture(0)
cap.set(3, WIDTH)
cap.set(4, HEIGHT)

while(1):
    tipoColore = input('Inserisci il colore (Red, Green, Blue): ')
    tipoColore=tipoColore.upper()
    if(tipoColore == 'R'):
        AREA_MAX_COLORE=AREA_MAX_RED
        break
    if(tipoColore == 'G'):
        AREA_MAX_COLORE=AREA_MAX_GREEN
        break
    if(tipoColore == 'B'):
        AREA_MAX_COLORE=AREA_MAX_BLUE
        break


if(tipoColore == 'R'):
    lower_red = np.array([165,196,52]) 
    upper_red = np.array([180,255,255])
    lowerColor=lower_red
    upperColor=upper_red
    
if(tipoColore == 'G'):
    #lower_green = np.array([45,136,71]) 
    #upper_green = np.array([99,255,255])
    lower_green = np.array([50,127,71]) 
    upper_green = np.array([100,255,255])
    lowerColor=lower_green
    upperColor=upper_green
    
if(tipoColore == 'B'):
    #lower_blue = np.array([90,128,27]) 
    #upper_blue = np.array([105,255,255])
    lower_blue = np.array([85,97,45]) 
    upper_blue = np.array([141,255,255])
    lowerColor=lower_blue
    upperColor=upper_blue

#kernel = np.ones((5,5),np.uint8)

# This drives the program into an infinite loop.
while(1):
    # Captures the live stream frame-by-frame
    ret, frame = cap.read()
    
    #frame = cv2.blur(frame,(5,5))
    #frame=cv2.medianBlur(frame,3)
    #frame = cv2.GaussianBlur(frame,(5,5),0)
    
    # Edges 
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    #gray = cv2.medianBlur(gray, 5) 
    #edges = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 9) 
   
    # Cartoonization 
    #color = cv2.bilateralFilter(frame, 9, 250, 250) 
    #cartoon = cv2.bitwise_and(color, color, mask=edges)
    
    #frame=cartoon
    
    
    #imposta canali BG a 0 e R a 255
    #frame[:,:,0]=0
    #frame[:,:,1]=0
    #frame[:,:,2]=255

    """
    b=frame[:,:,0]
    g=frame[:,:,1]
    r=frame[:,:,2]    
    b=b-5
    g=g-2
    b=b+5    
    frame=cv2.merge((b,g,r))
    """
    
    """
    for i in range(HEIGHT):
        for j in range (WIDTH):
            b=frame.item(i,j,0)
            g=frame.item(i,j,1)
            r=frame.item(i,j,2)
            if(b>(r+30) and b>(g+30)):
                b=255
                frame.itemset((i,j,0),b)
            if(g>(b+30) and g>(r+30)):
                r=255
                frame.itemset((i,j,1),r)
            if(r>(b+30) and r>(g+30)):
                r=255
                frame.itemset((i,j,2),r)
    """            
    
    # Converts images from BGR to HSV 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      
    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame.
    mask = cv2.inRange(hsv, lowerColor, upperColor)
        
    #erosion = cv2.erode(mask,kernel,iterations = 1)
    #dilation = cv2.dilate(mask,kernel,iterations = 1)
    
    #--- Definisco le coordinate e l'area dell'oggetto ---
    # convert image to grayscale image
    #gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # convert the grayscale image to binary image
    #ret,thresh = cv2.threshold(gray_image,127,255,0)
    # calculate moments of binary image
    #M = cv2.moments(thresh)
        
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
    cv2.circle(frame, (int(WIDTH/2), int(HEIGHT/2)), int(X_DIVISION/2), (0, 0, 255), 2) #STOP - Zone
    cv2.circle(frame, (int(WIDTH/2), int(HEIGHT/2)), SOGLIA_AZIONE, (255, 0, 0), 2) #maneuvering zone
    cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) #Area detect

    
    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
    #res = cv2.bitwise_and(frame,frame, mask=mask)
    cv2.imshow('Frame',frame)
    #cv2.imshow('mask',mask) 
    #cv2.imshow('res',res)
    
    #cv2.imshow("edges", edges) 
    #cv2.imshow("Cartoon", cartoon)
    
    AzionaMotori(cX, cY, AreaMax)    

    """
    for i in range(HEIGHT):
        for j in range (WIDTH):
            b=frame.item(i,j,0)
            g=frame.item(i,j,1)
            r=frame.item(i,j,2)
            if(r>(b+40) and r>(g+40)):
                r=255
                frame.itemset((i,j,2),r)
                
    cv2.imshow('Frame Modificato',frame)
    """            

    # This displays the frame, mask  
    # and res which we created in 3 separate windows.
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