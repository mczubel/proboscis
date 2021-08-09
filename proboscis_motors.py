#!/usr/bin/env python

from __future__ import division
import RPi.GPIO as io
io.setmode(io.BCM)
import math
import time
import datetime
import Adafruit_PCA9685
import Adafruit_ADS1x15
from adafruit_motorkit import MotorKit
import board
from adafruit_motor import stepper
from adafruit_servokit import ServoKit
import cfg
from RpiMotorLib import RpiMotorLib

#define GPIO pins
GPIO_pins = (-1, -1, -1) # Microstep Resolution MS1-MS3 -> GPIO Pin
GPIO_pins2 = (-1, -1, -1) 
direction= 4       # Direction -> GPIO Pin
step = 18      # Step -> GPIO Pin
direction2= 10       # Direction -> GPIO Pin
step2 = 9 
# Declare an named instance of class pass GPIO pins numbers
mymotortest = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")
mymotortest2 = RpiMotorLib.A4988Nema(direction2, step2, GPIO_pins2, "A4988")
# Initialise the first hat on the default address
kit1 = MotorKit()
# Initialise the second hat on a different address
#kit2 = MotorKit(address=0x61)
kit1.stepper2.release()
# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
values=0



def sensorCallback(channel):
    global tramo
    global sumador
    tramo = (40*math.pi)/20 # 40 centimetros diametro rueda*pi/ magnets number 
    timestamp=time.time()
    stamp=datetime.datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')
    if io.input(channel):
        print("sensor HIGH"+stamp)
        sumador = sumador
        w_distance = lambda x, y : x * y  # w_distance = wheel distance
        setw_distance(w_distance(sumador,tramo))
    else:
        #print("sensor LOW"+stamp)
        sumador +=1
        w_distance = lambda x, y : x * y
        print("sumador hall:",w_distance(sumador,tramo))
        setw_distance(w_distance(sumador,tramo))
        

def sensorCallback2(channel):
    cfg.tramo1 = (40*math.pi)/20 # 40 centimetros diametro rueda*pi/ magnets number 
    timestamp=time.time()
    stamp=datetime.datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')
    if io.input(channel):
        cfg.counter1 +=1
        w_distance = lambda x, y : x * y
        cfg.wdist=w_distance(cfg.counter1,cfg.tramo1)
        setw_distance(w_distance(cfg.counter1,cfg.tramo1)) 
   
io.setup(17, io.IN, pull_up_down=io.PUD_UP)

io.add_event_detect(17, io.RISING,callback=sensorCallback2, bouncetime=200)

def setw_distance(wd):
    global wdist
    wdist=wd
 
setw_distance(0)
def getw_distance():
    return wdist

def getdireccion():
    values=adc.read_adc(0, gain=GAIN)
    return values

def frontback(step,sr):
     setStepper4(step,sr)
     
def frontfront(step,sr):
    setStepper3(step,sr)

def sidefront(step):
    setStepper1(step)

def sideback(step):
    setStepper2(step)

def heightup(step):
    setStepper5(step)
    
def heightdown(step):
    setStepper6(step)

def stop():
    speedleft = 0
    speedright = 0
    setMotorLeft(0)
    setMotorRight(0)
    exit()


def setStepper1(step):
    cfg.sumk2stepper2+=step
    print("sumk2stepper2(+):",cfg.sumk2stepper2)
    mymotortest.motor_go(True, "Full" , step, .001, False, .05) 

def setStepper2(step):
    cfg.sumk2stepper2-=step
    print("sumk2stepper2(-):",cfg.sumk2stepper2)
    mymotortest.motor_go(False, "Full" , step, .001, False, .05)

def setStepper3(step,sr):
    stepp=step
    cfg.counter3+=1
    print("sumk1st2(1):",cfg.sumk1stepper2) 
    if (cfg.counter3<stepp)and (cfg.k1s2f==0):
        for i in range(sr):
            kit1.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
            cfg.sumk1stepper2+=1
    if (cfg.counter3>=step):
        cfg.counter3=0
        cfg.k1s2f=1
        return     

def setStepper4(step,sr): 
    stepp=step
    cfg.counter3+=1
    if (cfg.counter3<stepp)and (cfg.k1s2b==0):
        for i in range(sr):
            kit1.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
            cfg.sumk1stepper2-=1
    if (cfg.counter3>=step):
        cfg.counter3=0
        cfg.k1s2b=1
        return   

def setStepper5(step):
    cfg.sumk2stepper1+=step
    mymotortest2.motor_go(True, "Full" , step, .001, False, .05)

def setStepper6(step):
    cfg.sumk2stepper1-=step 
    mymotortest2.motor_go(False, "Full" , step, .001, False, .05) 

def stop2():
    kit1.stepper2.release()
    kit2.stepper1.release()
    kit2.stepper2.release()


# Initialise the PCA9685 using the default address (0x40).
PCA9685_pwm = Adafruit_PCA9685.PCA9685(address=0x41)
PCA9685_pwm.set_pwm_freq(50)
duty_cycle = 4095
io.setwarnings(False)

leftmotor_in1_pin = 22
leftmotor_in2_pin = 23
io.setup(leftmotor_in1_pin, io.OUT)
io.setup(leftmotor_in2_pin, io.OUT)

leftmotor2_in1_pin = 24 #5
leftmotor2_in2_pin = 12 #6
io.setup(leftmotor2_in1_pin, io.OUT)
io.setup(leftmotor2_in2_pin, io.OUT)


rightmotor_in1_pin = 13
rightmotor_in2_pin = 19
io.setup(rightmotor_in1_pin, io.OUT)
io.setup(rightmotor_in2_pin, io.OUT)

rightmotor2_in1_pin = 5 #25
rightmotor2_in2_pin = 6 #12
io.setup(rightmotor2_in1_pin, io.OUT)
io.setup(rightmotor2_in2_pin, io.OUT)


direccion_in1_pin = 16
direccion_in2_pin = 20
io.setup(direccion_in1_pin, io.OUT)
io.setup(direccion_in2_pin, io.OUT)

aplicar_in1_pin = 21
aplicar_in2_pin = 26
io.setup(aplicar_in1_pin, io.OUT)
io.setup(aplicar_in2_pin, io.OUT)




io.output(leftmotor_in1_pin, True)
io.output(leftmotor_in2_pin, True)
io.output(rightmotor_in1_pin, True)
io.output(rightmotor_in2_pin, True)
io.output(leftmotor2_in1_pin, True)
io.output(leftmotor2_in2_pin, True)
io.output(rightmotor2_in1_pin, True)
io.output(rightmotor2_in2_pin, True)
io.output(direccion_in1_pin, True)
io.output(direccion_in2_pin, True)
io.output(aplicar_in1_pin, True)
io.output(aplicar_in2_pin, True)

def setServoPulse(channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    PCA9685_pwm.set_pwm(channel, 0, int(pulse))
    
def setRotationAngle(channel, Angle): 
    if(Angle >= 0 and Angle <= 180):
        temp = Angle * (2000 / 180) + 501
        setServoPulse(channel, temp)
    else:
        print("Angle out of range")

def servoangle(angleg):
    global angleg2,angleg3 
    angle1=angleg
    in_min=40
    in_max=140
    out_min=400
    out_max=570
    angleg2=int((angle1-in_min)* (out_max-out_min)/(in_max-in_min)+out_min)
    angleg3=970-angleg2
    PCA9685_pwm.set_pwm(10, 0, angleg3)
    
def getangle():
    return angleg2,angleg3  

def setMotorMode(motor, mode):
   if motor == "leftmotor":
      if mode == "reverse":
         io.output(leftmotor_in1_pin, True)
         io.output(leftmotor_in2_pin, False)
      elif  mode == "forward":
         io.output(leftmotor_in1_pin, False)
         io.output(leftmotor_in2_pin, True)
      else:
         io.output(leftmotor_in1_pin, False)
         io.output(leftmotor_in2_pin, False)
   elif motor == "rightmotor":
      if mode == "reverse":
         io.output(rightmotor_in1_pin, False)
         io.output(rightmotor_in2_pin, True)      
      elif  mode == "forward":
         io.output(rightmotor_in1_pin, True)
         io.output(rightmotor_in2_pin, False)
      else:
         io.output(rightmotor_in1_pin, False)
         io.output(rightmotor_in2_pin, False)
   else:
      io.output(leftmotor_in1_pin, False)
      io.output(leftmotor_in2_pin, False)
      io.output(rightmotor_in1_pin, False)
      io.output(rightmotor_in2_pin, False)
      


def setMotorLeft(power):
   int(power)
   if power < 0:
      
      pwm = -int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(2, 0, pwm)
      PCA9685_pwm.set_pwm(3, 0, 0)   
   elif power > 0:
      
      pwm = int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(2, 0, 0)
      PCA9685_pwm.set_pwm(3, 0, pwm)      
   else:
      
      PCA9685_pwm.set_pwm(2, 0, 0)
      PCA9685_pwm.set_pwm(3, 0, 0)  
      PCA9685_pwm.set_pwm(4, 0, 0)
      PCA9685_pwm.set_pwm(5, 0, 0)
      PCA9685_pwm.set_pwm(11, 0, 0)
      PCA9685_pwm.set_pwm(12, 0, 0)  
      PCA9685_pwm.set_pwm(13, 0, 0)
      PCA9685_pwm.set_pwm(14, 0, 0) 

def setMotorLeft2(power):
   int(power)
   if power < 0:
      pwm = -int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(13, 0, pwm)
      PCA9685_pwm.set_pwm(14, 0, 0)   
   elif power > 0:
      pwm = int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(13, 0, 0)
      PCA9685_pwm.set_pwm(14, 0, pwm)      
   else:
      
      PCA9685_pwm.set_pwm(2, 0, 0)
      PCA9685_pwm.set_pwm(3, 0, 0)  
      PCA9685_pwm.set_pwm(4, 0, 0)
      PCA9685_pwm.set_pwm(5, 0, 0)
      PCA9685_pwm.set_pwm(11, 0, 0)
      PCA9685_pwm.set_pwm(12, 0, 0)  
      PCA9685_pwm.set_pwm(13, 0, 0)
      PCA9685_pwm.set_pwm(14, 0, 0) 
   
def setMotorRight(power):
   int(power)
   if power < 0:
      pwm = -int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(4, 0, pwm)
      PCA9685_pwm.set_pwm(5, 0, 0)        
   elif power > 0:
      pwm = int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(4, 0, 0)
      PCA9685_pwm.set_pwm(5, 0, pwm)      
   else:
      
      PCA9685_pwm.set_pwm(2, 0, 0)
      PCA9685_pwm.set_pwm(3, 0, 0)  
      PCA9685_pwm.set_pwm(4, 0, 0)
      PCA9685_pwm.set_pwm(5, 0, 0)
      PCA9685_pwm.set_pwm(11, 0, 0)
      PCA9685_pwm.set_pwm(12, 0, 0)  
      PCA9685_pwm.set_pwm(13, 0, 0)
      PCA9685_pwm.set_pwm(14, 0, 0)  

def setMotorRight2(power):
   int(power)
   if power < 0:
      pwm = -int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(12, 0, pwm)
      PCA9685_pwm.set_pwm(11, 0, 0)        
   elif power > 0:
      pwm = int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(12, 0, 0)
      PCA9685_pwm.set_pwm(11, 0, pwm)      
   else:
      PCA9685_pwm.set_pwm(2, 0, 0)
      PCA9685_pwm.set_pwm(3, 0, 0)  
      PCA9685_pwm.set_pwm(4, 0, 0)
      PCA9685_pwm.set_pwm(5, 0, 0)
      PCA9685_pwm.set_pwm(11, 0, 0)
      PCA9685_pwm.set_pwm(12, 0, 0)  
      PCA9685_pwm.set_pwm(13, 0, 0)
      PCA9685_pwm.set_pwm(14, 0, 0)  


def setDireccion(power):
    if power is not None:
        int(power)
        if power < 0:
            pwm = -int(duty_cycle * power)
            if pwm > duty_cycle:
                pwm = duty_cycle
            PCA9685_pwm.set_pwm(6, 0, pwm)
            PCA9685_pwm.set_pwm(7, 0, 0)        
        elif power > 0:
            pwm = int(duty_cycle * power)
            if pwm > duty_cycle:
                pwm = duty_cycle
            PCA9685_pwm.set_pwm(6, 0, 0)
            PCA9685_pwm.set_pwm(7, 0, pwm)      
        else:
          PCA9685_pwm.set_pwm(6, 0, 0)
          PCA9685_pwm.set_pwm(7, 0, 0)  
      
      
def setAplicacion(power):
   int(power)
   if power < 0:
      pwm = -int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(8, 0, pwm)
      PCA9685_pwm.set_pwm(9, 0, 0)        
   elif power > 0:
      pwm = int(duty_cycle * power)
      if pwm > duty_cycle:
         pwm = duty_cycle
      PCA9685_pwm.set_pwm(8, 0, 0)
      PCA9685_pwm.set_pwm(9, 0, pwm)      
   else:
      PCA9685_pwm.set_pwm(8, 0, 0)
      PCA9685_pwm.set_pwm(9, 0, 0)  


def spray():
    if (cfg.startapl==0):
        starttimea = time.monotonic()
        cfg.endtime2=(starttimea + 1)
        cfg.startapl=1
    timeactuala = time.monotonic()
    if (cfg.timeantes < cfg.endtime2):
        aplicacion = -0.6
        cfg.endapl=0
    if (cfg.timeantes > cfg.endtime2):
        aplicacion = 0
        cfg.endapl=1
    if aplicacion < -1:
        aplicacion = -1
    setAplicacion(aplicacion)
    endaplicacion=cfg.endapl
    endtc=cfg.endtc
    if (endaplicacion==1) and (endtc==0):
        starttimec = time.monotonic()
        cfg.endtime21=(starttimec + 1.7)
        cfg.endapl=0
        cfg.endtc=1
    timeactualc=time.monotonic()
    if (timeactualc < cfg.endtime21)and (cfg.endtc==1):
        aplicacion = 0.4
    if (timeactualc > cfg.endtime21)and (cfg.endtc==1):
        aplicacion=0
        cfg.wflag=1
        cfg.height_cal=1
    if aplicacion > 1:
        aplicacion = 1
    setAplicacion(aplicacion)
    print("wflag final;",cfg.wflag)
                         

def forward1(speedleftt,speedrightt):
    speedleft = speedleftt#speedleft + 0.1
    speedright = speedrightt#speedright + 0.1
    if speedleft > 1:
        speedleft = 1
    if speedright > 1:
        speedright = 1
    setMotorLeft(speedleft)
    setMotorRight(speedright)
    setMotorLeft2(speedleft)
    setMotorRight2(speedright) 
    print("forward")

def reverse(speedleftt,speedrightt):
    speedleft = speedleftt#speedleft + 0.1
    speedright = speedrightt#speedright + 0.1
    if speedleft < -1:
        speedleft = -1
    if speedright < -1:
        speedright = -1
    setMotorLeft(speedleft)
    setMotorRight(speedright)
    setMotorLeft2(speedleft)
    setMotorRight2(speedright)
    #print("reverse")
def set_distdif(distdif):
    global dist_dif
    dist_dif=distdif

set_distdif(0)

def get_distdif():
    return dist_dif

def move(distl,go,t1,t2,t3,):
    distance_from = cfg.wdist
    if (cfg.avance==0):
        cfg.distance_to = distl
        cfg.avance=1
        print("distance_to:",cfg.distance_to)
    cfg.dist_dif= cfg.distance_to-distance_from
    print("dist_dif",cfg.dist_dif)
    set_distdif(cfg.dist_dif)
    if (distance_from < (cfg.distance_to-10)) or (distl>10): 
        if (cfg.forst==0):
            inicialtime=time.monotonic()
            cfg.break0time=inicialtime + 3
            cfg.break1time=inicialtime + 5
            cfg.break2time=inicialtime + 8
            cfg.forst=1
        timeactual=time.monotonic()
        if (timeactual< cfg.break0time):
            distancewheel1 = cfg.distancewheel
            cfg.distancewheel = cfg.wdist
            print("distancewheel:", cfg.distancewheel)
            if distancewheel1==cfg.distancewheel:
                cfg.torque += 0.1
                print("torque:",cfg.torque)
            else:
                cfg.torque=0.2
                print("torque2:",cfg.torque)
        if (timeactual < cfg.break1time):
            if go==0:
                forward1(cfg.torque,cfg.torque)
            else:
                reverse(-cfg.torque,-cfg.torque)
            cfg.distancewheel = cfg.wdist
            #print("moviendo")#moving
        if (timeactual > cfg.break1time) and (timeactual < cfg.break2time):
            if go==0:
                forward1(0,0)
            else:
                reverse(0,0)   
            #print("parado")#stopped
        if (timeactual > cfg.break2time):
            cfg.forst=0
            inicialtime=0
            cfg.break0time=0
            cfg.break1time=0
            cfg.break2time=0
        cfg.avance=0
    
        
        
#################### TURN ##########################
def left_to_center ():
      values = getdireccion()
      if values > 22350:
          direccion=-0.4
          setDireccion(direccion)
          if direccion > 1:
              direccion = 1
          values=getdireccion()     
      
      if values < 22350 :
             direccion = 0
             setDireccion(direccion)
      
      
def center_to_right (turn):
      values=getdireccion()
      #if values < 22350:
      if values < 22350:
          direccion=-0.4
          setDireccion(direccion) 
          if direccion <- 1:
              direccion = -1
          values = getdireccion()     
      
      if values < turn :
             direccion = 0
             setDireccion(direccion) 
     
      
      
   
def right_to_center ():
      values=getdireccion()
      if values< 22100:
          direccion=0.2
          setDireccion(direccion) 
          if direccion > 1:
              direccion = 1
      
          values=getdireccion()
      if values > 22150 :
             direccion = 0
             setDireccion(direccion)
      
      
      
def center_to_left (turn ):
      values=getdireccion()
      if values > 21500:
          direccion= 0.4
          setDireccion(direccion) 
          if direccion > 1:
              direccion = 1
      
          values=getdireccion()
      if values > turn :
             direccion = 0
             setDireccion(direccion)
      
      

    

def exit():
   io.output(leftmotor_in1_pin, False)
   io.output(leftmotor_in2_pin, False)
   io.output(rightmotor_in1_pin, False)
   io.output(rightmotor_in2_pin, False)
   io.output(direccion_in1_pin, False)
   io.output(direccion_in2_pin, False)
   io.cleanup()
   
   PCA9685_pwm.set_pwm(2, 0, 0)
   PCA9685_pwm.set_pwm(3, 0, 0)
   PCA9685_pwm.set_pwm(4, 0, 0)
   PCA9685_pwm.set_pwm(5, 0, 0)
   PCA9685_pwm.set_pwm(6, 0, 0)
   PCA9685_pwm.set_pwm(7, 0, 0)
   kit1.stepper1.release
   kit1.stepper2.release

