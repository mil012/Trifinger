import RPi.GPIO as GPIO  
import math
import time
GPIO.setmode(GPIO.BOARD)
import pandas as pd
import numpy as np

class Coordinates:

    def __init__(self,x=0,y=0):
        self.fromCartesian(x,y)
        self.x = x
        self.y = y
        self.R = 0
        self.phi = 0

    def fromCartesian(self,x,y): # Converts from Cartesian to Polar
        if x == 0 and y == 0:
            self.r = 0
            self.phi = 0
            return
        self.r = math.sqrt(sq(x) + sq(y))
        if x == 0 and 0 < y:
            self.phi = math.pi/2
        elif x == 0 and y < 0:
            self.phi = math.pi*3/2
        elif x < 0:
            self.phi = math.atan( y / x ) + math.pi
        elif y < 0:
            self.phi = math.atan( y / x ) + 2 * math.pi
        else:
            self.phi = math.atan( y / x )
        
    def fromPolar(self,r,phi): # Converts from Polar to Cartesian
        self.r = r
        self.phi = phi
        self.x = r * math.cos(phi)
        self.y = r * math.sin(phi)

    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getR(self):
        return self.R
    def getAngle(self):
        return self.phi

fillerA = Coordinates(0,0)
fillerB = Coordinates(0,0)
fillerC = Coordinates(0,0)

point = [fillerA,fillerB,fillerC] #Replaces these with Coordinates

#Some quality of life things so I don't have to change function names
def sq(num):
    return num**2

def delayMicroseconds(microseconds):
    time.sleep(microseconds/1500000) #Changed from 1000000 to 1500000

M_PI = math.pi #This is if M_PI turns out to be different than just pi, its easier to switch
DEG_TO_RAD = 0.0174532925 #Serves the same function as in arduino, degrees*this=rads
RAD_TO_DEG = 57.29577951 #Same function as arduino

HIGH = 1 #Might have to use GPIO.HIGH
LOW = 0

def analogRead(Pin):
    print("AnalogRead this pin")

def digitalWrite(Pin, State):
    GPIO.output(Pin, State) 

def digitalRead(Pin):
    return GPIO.input(Pin) 
 
#---------------------------------------------------------------
# Vertical Actuator
dirVerPin = [3, 5, 7]
stepVerPin = [11,13 , 15]

# Horizontal Actuator
dirHorPin = [19, 21, 23]
stepHorPin = [40, 29, 31] # 27 is the problem, cahnged to 40

# Finger Tip
servoPin = [33,35 ,37 ]

# Limit Switch
limitVerPin = [8, 10, 12] #These have been changed to be correct
limitHorPin = [24, 26, 32] #28 is the problem, changed to 32

GPIO.setup(dirVerPin, GPIO.OUT, initial=GPIO.LOW)      
GPIO.setup(stepVerPin, GPIO.OUT, initial=GPIO.LOW)      
GPIO.setup(dirHorPin, GPIO.OUT, initial=GPIO.LOW)      
GPIO.setup(stepHorPin, GPIO.OUT, initial=GPIO.LOW)      
GPIO.setup(servoPin, GPIO.OUT, initial=GPIO.LOW)      
GPIO.setup(limitHorPin, GPIO.IN)      
GPIO.setup(limitVerPin, GPIO.IN)   

pwm33=GPIO.PWM(33, 50)
pwm35=GPIO.PWM(35, 50)
pwm37=GPIO.PWM(37, 50)
pwm33.start(0)
pwm35.start(0)
pwm37.start(0)

pwms = [pwm33,pwm35,pwm37]

# Potentiometer Pins, Replace this with what Allison has for potentiometer
A0 = 0
A1 = 0
A2 = 0
Pins = [A0, A1, A2]

servoTipAnglePrev = [45,45,45] # use this one to keep track of the current servo angle

# Global Variables

thetasVer = [0,0,0]
thetasHor = [25*DEG_TO_RAD, 25*DEG_TO_RAD, 25*DEG_TO_RAD]
thetasTip = [0,0,0]
stepsVer = [0,0,0]
stepsHor = [0,0,0]
stepsTip = [0,0,0]

dirVer = [LOW,LOW,LOW]
dirHor = [LOW,LOW,LOW]
dirTip = [LOW,LOW,LOW]

thetaVerPrev = [0,0,0]
thetaHorPrev = [25*DEG_TO_RAD, 25*DEG_TO_RAD, 25*DEG_TO_RAD]
thetaTipPrev = [0,0,0]

angle = [0,0,0]

print("Servo Code line 62") #Unsure how to integrate servo

invKin = []

# Defaults 
num = 0 
potVal = 0
potReading = 0
ang = 0

servoToPot = [57,24,32]

# Adjustables
diagLength = [12.992,12.962,12.885]
tipLength = [12.6, 12.505, 12.659]

microDelays = [500,500,500]

originR = [10.931,11.025,10.882]
originZ = 25
originAlpha = [0*DEG_TO_RAD,120*DEG_TO_RAD,240*DEG_TO_RAD]

coordX = [0,0,0]
coordY = [0,0,0]
coordZ = [0,0,0]

servoMapping = [1.538,1.295,1.353]

def SetServoAngle(angle,armNum):
    if angle == thetasTip[armNum]:
        return
    duty = angle / 18 + 2
    GPIO.output(33 + armNum*2, True)
    pwms[armNum].ChangeDutyCycle(duty)
    time.sleep(0.25)
    GPIO.output(33 + armNum*2, False)
    pwms[armNum].ChangeDutyCycle(0)
    


def potMapping(armNum, ang):
    #potReading = analogRead(Pins[armNum]);
    #potVal = map(potReading, 0, 1023, 0, 180);
    #int x = servo[armNum].read(); 
    x = 1 # <----- This is a placeholder
    potVal = 1 #<----- This is a placeholder
    while x != potVal:
        if x < potVal:
            potVal = potVal - 1
        else:
            potVal = potVal + 1

    ang = servoToPot[armNum] - 1
    while ang != potVal:
        if ang < potVal:
            potVal = potVal - 1
        else:
            potVal = potVal + 1
        print(potVal)
        #servo[armNum.write(potVal)]
        time.sleep(0.1)


# Global Conversion
def globalAxisConversion( r, alpha, z, armNum):

    alpha = alpha*DEG_TO_RAD

    coordZ[armNum] = z

    polarR = math.sqrt(sq(r) + sq(originR[armNum]) - 2*r*originR[armNum]*math.cos(originAlpha[armNum]-alpha))  
    polarAlpha = -math.asin(r*math.sin(originAlpha[armNum]-alpha)/polarR)

    point[armNum].fromPolar(polarR,polarAlpha)
    
    coordX[armNum] = point[armNum].getX()
    coordY[armNum] = point[armNum].getY()

    return

# Inverse Kinematics Formula
def inverseKinematics(xf, ySetServoAnglef, zf, armNum):
    xf = -xf
    yf = -yf
    zf = -zf
    df = math.sqrt( sq(xf) + sq(yf) + sq(zf) )
    y2D = -math.sqrt( sq(yf) + sq(zf) )
    x2D = xf
    thetaVer = math.atan( yf / zf )
    thetaTip = math.acos( sq(diagLength[armNum]) + sq(tipLength[armNum]) - sq(x2D) - sq(y2D) / (2*diagLength[armNum] * tipLength[armNum]))
    thetaHor = M_PI - (math.atan(y2D / x2D) + math.atan( (tipLength[armNum] * math.sin(M_PI - thetaTip))/(diagLength[armNum] + tipLength[armNum] * math.cos(M_PI - thetaTip))));
    # Error Handling
    if zf > 0:
        return 1
    if df < (tipLength[armNum] - diagLength[armNum]):
        return 2
    if df > tipLength[armNum] + diagLength[armNum]: 
        return 3
    if thetaVer < -90* DEG_TO_RAD or thetaVer >  90* DEG_TO_RAD: 
        return 4
    if thetaHor <  20* DEG_TO_RAD or thetaHor > 120* DEG_TO_RAD: 
        return 5
    if thetaTip <   0* DEG_TO_RAD or thetaTip > 120* DEG_TO_RAD: 
        return 6

    thetasVer[armNum] = thetaVer
    thetasHor[armNum] = thetaHor-20*DEG_TO_RAD
    thetasTip[armNum] = thetaTip

    return 0

# Angle Conversion Formula
def angleConversion(thetaVer, thetaHor, armNum):
    stepVer = abs(round((thetaVer - thetaVerPrev[armNum]) * 100 * 9 / (M_PI)))
    stepHor = abs(round((thetaHor - thetaHorPrev[armNum]) * 100 * 9 / (M_PI)))
    stepsVer[armNum] = stepVer
    stepsHor[armNum] = stepHor
    return

#Run Motors
def motorRun( stepVer, stepHor, armNum):
    #print("Motor Running")
    maxStep = max(stepVer, stepHor)
    
    if thetasVer[armNum] >= thetaVerPrev[armNum]:
        dirVer[armNum] = HIGH
    else: 
        dirVer[armNum] = LOW

    if thetasHor[armNum] >= thetaHorPrev[armNum]:
        dirHor[armNum] = LOW 
    else: 
        dirHor[armNum] = HIGH
    i = maxStep
    while i > 0:
        digitalWrite(dirVerPin[armNum], dirVer[armNum])
        if i < stepVer:
            for x in range(0,8):
                if digitalRead(limitVerPin[armNum]) == LOW:
                    print("HIT THE LIMIT")
                    continue;
                digitalWrite(stepVerPin[armNum], HIGH)
                delayMicroseconds(microDelays[0])
                digitalWrite(stepVerPin[armNum], LOW)
                delayMicroseconds(microDelays[0])
        digitalWrite(dirHorPin[armNum], dirHor[armNum])
        if i < stepHor:
            for x in range(0,8):
                if digitalRead(limitHorPin[armNum]) == LOW:
                    print("HIT THE LIMIT")
                    continue;
                digitalWrite(stepHorPin[armNum], HIGH)
                delayMicroseconds(microDelays[1])
                digitalWrite(stepHorPin[armNum], LOW)
                delayMicroseconds(microDelays[1])
        i = i - 1 

    # Setting up the previous variables
    thetaVerPrev[armNum] = thetasVer[armNum]
    thetaHorPrev[armNum] = thetasHor[armNum]
    

#Run Motors with servo
def motorRunwservo( stepVer, stepHor, servo_ang, armNum):

    maxStep = max(stepVer, stepHor)
    
    if thetasVer[armNum] >= thetaVerPrev[armNum]:
        dirVer[armNum] = HIGH
    else: 
        dirVer[armNum] = LOW

    if thetasHor[armNum] >= thetaHorPrev[armNum]:
        dirHor[armNum] = LOW 
    else: 
        dirHor[armNum] = HIGH
    i = maxStep
    while i > 0:
        digitalWrite(dirVerPin[armNum], dirVer[armNum])
        if i < stepVer:
            for x in range(0,8):
                if digitalRead(limitVerPin[armNum]) == LOW:
                    print("HIT THE LIMIT")
                    continue;
                digitalWrite(stepVerPin[armNum], HIGH)
                delayMicroseconds(microDelays[0])
                digitalWrite(stepVerPin[armNum], LOW)
                delayMicroseconds(microDelays[0])
        digitalWrite(dirHorPin[armNum], dirHor[armNum])
        if i < stepHor:
            for x in range(0,8):
                if digitalRead(limitHorPin[armNum]) == LOW:
                    print("HIT THE LIMIT")
                    continue;
                digitalWrite(stepHorPin[armNum], HIGH)
                delayMicroseconds(microDelays[1])
                digitalWrite(stepHorPin[armNum], LOW)
                delayMicroseconds(microDelays[1])
        i = i - 1 
    #SetServoAngle(new_servo_commands[1],armNum)
    #SetServoAngle(new_servo_commands[2],armNum)
    SetServoAngle(servo_ang,armNum)
    # Setting up the previous variables
    thetaVerPrev[armNum] = thetasVer[armNum]
    thetaHorPrev[armNum] = thetasHor[armNum]
    #servoTipAnglePrev[armNum] = servo_ang
    
    #(servo_ang,armNum)

def motorRun3(stepsVer, stepsHor, servoAngs):
    maxStepVer = max(stepsVer[0], stepsVer[1])
    maxStepVer = max(maxStepVer, stepsVer[2])
    maxStepHor = max(stepsHor[0], stepsHor[1])
    maxStepHor = max(maxStepHor, stepsHor[2])

    maxStep = max(maxStepVer, maxStepHor)

    for i in range(0,3):
    
        if thetasVer[i] >= thetaVerPrev[i]:
            dirVer[i] = HIGH
        else:
            dirVer[i] = LOW
        if thetasHor[i] >= thetaHorPrev[i]:
            dirHor[i] = LOW    #This might have to be switched
        else:
            dirHor[i] = HIGH

    for i in range(0,3):
        digitalWrite(dirVerPin[i], dirVer[i])
        digitalWrite(dirHorPin[i], dirHor[i])

    for i in range(0,maxStep):
        for x in range(0,8):
            for j in range(0,3):
                if i < stepsVer[j]:
                    digitalWrite(stepVerPin[j], HIGH)
            delayMicroseconds(microDelays[0])
            for j in range(0,3):
                if i < stepsVer[j]:
                    digitalWrite(stepVerPin[j], LOW)
            delayMicroseconds(microDelays[0])

        for x in range(0,8):
            for j in range(0,3):
                if i < stepsHor[j]:
                    digitalWrite(stepHorPin[j], HIGH)
            delayMicroseconds(microDelays[1])
            for j in range(0,3):
                if i < stepsHor[j]:
                    digitalWrite(stepHorPin[j], LOW)
            delayMicroseconds(microDelays[1])

    for i in range(0,3):
        thetaVerPrev[i] = thetasVer[i]
        thetaHorPrev[i] = thetasHor[i]
        SetServoAngle(servoAngs[i],i)
    

def reset():
    for i in range(0,3):
        dirVer[i] = LOW
        dirHor[i] = HIGH
        digitalWrite(dirVerPin[i], dirVer[i])
        digitalWrite(dirHorPin[i], dirHor[i]) 

    limitHorPressed = [0,0,0]
    #print("If limits are working WORKING should print below")
    while limitHorPressed[0] == 0 or limitHorPressed[1] == 0 or limitHorPressed[2] == 0:
        #print("Worked")
        for x in range(0,8):
            for i in range(0,3):
                if digitalRead(limitHorPin[i]) == HIGH and limitHorPressed[i] == 0:   
                    digitalWrite(stepHorPin[i], HIGH)
                else: 
                    limitHorPressed[i] = 1
            delayMicroseconds(microDelays[0])
            for i in range(0,3):
                if digitalRead(limitHorPin[i]) == HIGH and limitHorPressed[i] == 0:
                    digitalWrite(stepHorPin[i], LOW)
                else: 
                    limitHorPressed[i] = 1
            delayMicroseconds(microDelays[0])
    
    limitVerPressed = [0,0,0]
    while limitVerPressed[0] == 0 or limitVerPressed[1] == 0 or limitVerPressed[2] == 0:
        for x in range(0,8):
            for i in range(0,3):
                if digitalRead(limitVerPin[i]) == HIGH and limitVerPressed[i] == 0:   
                    digitalWrite(stepVerPin[i], HIGH)
                else: 
                    limitVerPressed[i] = 1
            delayMicroseconds(microDelays[0]) 
            for i in range(0,3):
                if digitalRead(limitVerPin[i]) == HIGH and limitVerPressed[i] == 0:
                    digitalWrite(stepVerPin[i], LOW)
                else: 
                    limitHorPressed[i] = 1
            delayMicroseconds(microDelays[0])
    
    for i in range(0,3):
        thetaVerPrev[i] = -90*DEG_TO_RAD
        thetaHorPrev[i] = 15*DEG_TO_RAD
        thetasVer[i] = 0*DEG_TO_RAD
        thetasHor[i] = 20*DEG_TO_RAD
        angleConversion(thetasVer[i], thetasHor[i], i)
    #for armnum in range(0,3):
        #SetServoAngle(90,armnum)
    motorRun3(stepsVer, stepsHor,[90,90,90])

def parse_command(command):
    commandlist = command.split(" ")
    commandlist.append("EMP")
    commandlist.append("EMP")
    commandlist.append("EMP")
    return commandlist

def loop(): #Very much work in progress

    #String str = Serial.readString();
    #char data[100];
    #str.toCharArray(data, 100);
    #char a1[20], a2[20], a3[20];
    #char a0[20];
    #int result = sscanf(data, "%s %s %s %s", &a0, &a1, &a2 , &a3);
    
    #double xf = String(a1).toDouble();
    #double yf = String(a2).toDouble();
    #double zf = String(a3).toDouble();
    command = []
    command.append(input("Input:"))
    
    #xf = command[1]
    #yf = command[2]
    #zf = command[3]
    #Check string
    #Switch Statement below

    if command[0] == 'h': # Home Button
        reset()

    if command[0] == 'c': # Global Reference Frame Button
        for i in range(0,3):
            globalAxisConversion(xf,yf,zf,i)
            invKin[i] = inverseKinematics(coordX[i], coordY[i], coordZ[i],i)
        if invKin[0] == 0 and invKin[1] == 0 and invKin[2] == 0:
            for i in range(0,3):
                angleConversion(thetasVer[i], thetasHor[i],i)
                invKin[i] = 0
        motorRun3(stepsVer, stepsHor)

    if command[0] == 'a': # Finger All Angle Button
        for i in range(0,3):
            thetasVer[i] = xf*DEG_TO_RAD
            thetasHor[i] = yf*DEG_TO_RAD

            angleConversion(thetasVer[i],thetasHor[i],i)
        
        motorRun3(stepsVer, stepsHor)
        
        for i in range(0,3):
            potMapping(i,zf)
            time.sleep(0.1)

    FingerLetter = ['A','B','C']
    num = 2
    if command[0] in FingerLetter:# Finger A,B, and C Angle Button #This is just compressed code a bit
        print("Tried Command")
        for i in range(0,3):
            if FingerLetter[i] == command[0]:
                num = i

                thetasVer[num] = xf*DEG_TO_RAD
                thetasHor[num] = yf*DEG_TO_RAD
            
            angleConversion(thetasVer[num], thetasHor[num], num)
            motorRun(stepsVer[num], stepsHor[num], num)
            #potMapping(num, zf)
            break

    if command[0] == 'g': # Grab Button
        for i in range(0,3):
            thetasVer[i] = thetaVerPrev[i]
            thetasHor[i] = thetaHorPrev[i] + xf*DEG_TO_RAD
            angleConversion(thetasVer[i], thetasHor[i], i)
        motorRun3(stepsVer,stepsHor)
    
    if command[0] == 'r': # Release Button
        for i in range(0,3):
            thetasVer[i] = thetaVerPrev[i]
            thetasHor[i] = thetaHorPrev[i] - xf*DEG_TO_RAD
            angleConversion(thetasVer[i], thetasHor[i], i)
        motorRun3(stepsVer, stepsHor)

    #################################
    if command[0] == 'p': # Push an object at origin towards arm A
        
        #angles = pd.read_csv('angs.csv')
        #print(angles)
        #[0,25,90,0,25,90,0,25,90], is the neutral
        angles = np.array([[0,25,90,0,25,90,0,25,90],
                           [0,30,90,0,30,90,0,30,90],
                           [0,35,90,0,35,90,0,35,90],
                           [0,40,90,0,40,90,0,40,90],
                           [0,45,100,0,45,100,0,45,100],
                           [0,50,110,0,50,110,0,50,110],
                           [0,55,110,0,55,110,0,55,110],
                           [0,60,120,0,60,120,0,60,120],
                           [0,65,120,0,65,120,0,65,120],
                           [0,70,120,0,70,120,0,70,120],
                           [0,75,120,0,75,120,0,75,120],
                           [0,80,120,0,80,120,0,80,120]
                           ])
        for i in range(angles.shape[0]):
            
            A_angles = angles[i,0:2] *DEG_TO_RAD
            A_servo_ang = angles[i,2]
            B_angles = angles[i,3:5] *DEG_TO_RAD
            B_servo_ang = angles[i,5]
            C_angles = angles[i,6:8] *DEG_TO_RAD
            C_servo_ang = angles[i,8]
            
            #print(A_angles[0])
            #print(B_angles[1])
            #print(C_angles[2])
            
            #for arm A
            
            thetasVer[0] = A_angles[0]
            thetasHor[0] = A_angles[1]
            
            
            thetasVer[1] = B_angles[0]
            thetasHor[1] = B_angles[1]
            
            
            thetasVer[2] = C_angles[0]
            thetasHor[2] = C_angles[1]
            
            
            angleConversion(A_angles[0],A_angles[1],0)
            angleConversion(B_angles[0],B_angles[1],1)
            angleConversion(C_angles[0],C_angles[1],2)
            
            #motorRunwservo(stepsVer[0],stepsHor[0],A_servo_ang,0)
            #motorRunwservo(stepsVer[1],stepsHor[1],B_servo_ang,1)
            #motorRunwservo(stepsVer[2],stepsHor[2],C_servo_ang,2)
            servoAngs = [A_servo_ang,B_servo_ang,C_servo_ang]
            motorRun3(stepsVer,stepsHor,servoAngs)
            
            thetasTip[0] = A_servo_ang
            thetasTip[1] = B_servo_ang
            thetasTip[2] = C_servo_ang
    ###################################


def setup():
    num = 2
    Vf =  0
    Hf = 40 #make this 25 to not move the smaller arm
    thetasVer[num] = Vf*DEG_TO_RAD 
    thetasHor[num] = Hf*DEG_TO_RAD #For some reason the thetas Horizontal and vertival have accidentally switchef
            
    angleConversion(thetasVer[num], thetasHor[num], num)
    motorRun(stepsVer[num], stepsHor[num], num)
    print("DIGITAL FIN READ")
    print(GPIO.input(limitVerPin[2]))
    print(digitalRead(limitVerPin[2]))
    print("END DIGITAL READ")
    
#setup() # This is where stuff actually runs
#reset()
for i in range(50):
    loop()