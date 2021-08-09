#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time
import argparse
#from PCA9685 import PCA9685
import proboscis_motors as pm
import imutils
import pickle
import math
import serial
import cfg
ser= serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()
svm=cv2.ml.SVM_create()
svm=cv2.ml.SVM_load("/home/pi/Desktop/proboscis/models/svm_obst3.json")
model=pickle.load(open("/home/pi/Desktop/proboscis/models/anomaly_detector.model",'rb'))
#pwm = PCA9685()
#pwm.setPWMFreq(50)

 # Motion params
PAN_INCREMENT = 5
TILT_INCREMENT = 10
HYSTERESIS = 0.2

forward = True
#forward2=True
# Roam params
ROAM_START = 6
PAN_RANGE = (70,110) 
TILT_RANGE = (90,140)
frames_since_cardos = 10
# Initial pan-tilt position
cam_pan=90
cam_tilt = TILT_RANGE[1]
pm.setRotationAngle(1, 90)
pm.setRotationAngle(0, 90)
time.sleep(3)
# cam_tilt = 90
DEPTH_THRESH_HIGH = 3000
DEPTH_THRESH_LOW = 100
#DEPTH_THRESH_HIGH2 = 3000
#DEPTH_THRESH_LOW2 = 10

WARNING_DIST = 300

def headingar() :
    ser.write(b"n")
   
def voltar():
    ser.write(b"v")
    
def serialdata():
    if ser.in_waiting>0:
        line=ser.readline().decode('utf-8').rstrip()
        print(line)
    

    
def dist_lineal(angle,distz):
    angle1=angle
    distlin=0
    in_min=90
    in_max=140
    out_min=62                                                                                                                                                                      
    out_max=76
    distz=distz/10
    heightangle=int((angle1-in_min)* (out_max-out_min)/(in_max-in_min)+out_min)
    heightangle=(out_max+out_min)-heightangle
    heightangle=heightangle-cfg.weed_height 
    delta=(distz**2 - heightangle**2)
    if (delta>0):
        distlin = math.sqrt(delta)
    if (delta<0):
        cfg.weed_height+=1
    return distlin


def roidepth(frame,ta):
    coloredDisp = frame
    ta=ta
    centro1=coloredDisp.shape[0]/2
    centro2=coloredDisp.shape[1]/2
    if ta==0:
     #######COORDENADAS DE DETECCION DE OBSTACULOS####
         x1=int(centro2-200)
         x2=int(centro2+160)
         y1=int(centro1-100)
         y2=int(centro1+200)
    ########COORDENADAS DE DETECCCION DE CIRCULO#################################
    if ta==1:
        x1=int(centro2-150)
        x2=int(centro2+150)
        y1=int(centro1-145)
        y2=int(centro1+155)
    ########COORDENADAS XY CALIBRACION#######################################
    if ta==2:
        x1=int(centro2-170)
        x2=int(centro2+130)
        y1=int(centro1-145)
        y2=int(centro1+155)
   
    #cv2.rectangle(coloredDisp, (x1, y2), (x2, y1), color, cv2.FONT_HERSHEY_SIMPLEX)
    #cv2.circle(coloredDisp,(int(x1),int(y1)),5,(0,0,255),3) 
    #cv2.circle(coloredDisp,(int(x2),int(y1)),5,(255,255,255),3) 
   # cv2.circle(coloredDisp,(int(x1),int(y2)),5,(0,255,255),3)
   # cv2.circle(coloredDisp,(int(x2),int(y2)),5,(255,0,255),3) 
    roi_values=coloredDisp[y1:y2,x1-20:x2-20]
    cv2.imshow("depth colored disp", coloredDisp)
    centerroi1=roi_values.shape[0]/2
    centerroi2=roi_values.shape[1]/2
    
    #cv2.circle(roi_values,(int(centerroi1),int(centerroi2)),5,(255,0,255),3)    
    #cv2.circle(roi_values,(int(centerroi1-40),int(centerroi2)),5,(0,255,0),3)
    #cv2.circle(roi_values,(int(centerroi1-20),int(centerroi2)),5,(255,255,255),3)  
    return roi_values

def calc_spatials(bbox, depth):
        # Decrese the ROI to 1/3 of the original ROI
        deltaX = int((bbox[2] - bbox[0]) * 0.33)
        deltaY = int((bbox[3] - bbox[1]) * 0.33)
        bbox[0] = bbox[0] + deltaX
        bbox[1] = bbox[1] + deltaY
        bbox[2] = bbox[2] - deltaX
        bbox[3] = bbox[3] - deltaY
        

        # Calculate the average depth in the ROI. TODO: median, median /w bins, mode
        cnt = 0.0
        sum = 0.0
        for x in range(bbox[2] - bbox[0]):
            for y in range(bbox[3] - bbox[1]):
                depthPixel = depth[bbox[1] + y][bbox[0] + x]
                if depthPixel is not 0:
                    if np.any(depthPixel>DEPTH_THRESH_LOW) and np.any(depthPixel<DEPTH_THRESH_HIGH):
                        cnt+=1.0
                        sum+=depthPixel                       
        averageDepth = sum / cnt if 0 < cnt else 0        
        centroidX = int((bbox[2] - bbox[0]) / 2) + bbox[0]
        centroidY = int((bbox[3] - bbox[1]) / 2) + bbox[1]
        mid = int(depth.shape[0] / 2) # middle of the depth img
        bb_x_pos = centroidX - mid
        bb_y_pos = centroidY - mid
        angle_x = calc_angle(bb_x_pos)
        angle_y = calc_angle(bb_y_pos)
        z = averageDepth;
        x = z * math.tan(angle_x)
        y = -z * math.tan(angle_y)        
        spatial_coords=[x,y,z, centroidX, centroidY]        
        return spatial_coords

monoHFOV = np.deg2rad(73.5)
depthWidth = 1080.0

def calc_angle(offset):
        return math.atan(math.tan(monoHFOV / 2.0) * offset / (depthWidth / 2.0))
    
def distancia(spatial_coords,centro1,centro2,frame):
    sp=spatial_coords
    distance = math.sqrt((cfg.distx-sp[0])**2 + (cfg.disty-sp[1])**2 + (cfg.distz-sp[2])**2)
    cv2.line(frame, (int(sp[3]), int(sp[4])), (int(cfg.centro1), int(cfg.centro2)), (50,220,100), 4)
    return distance

def circle(frame,centrox,centroy):
    frame=frame
    centrox=centrox
    centroy=centroy
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV) 
    mask = cv2.inRange(hsv, cfg.colorLower, cfg.colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cfg.center = None
    cfg.radius=0
    if len(cnts) > 2:
        c = max(cnts, key=cv2.contourArea)
        ((cfg.x3, cfg.y3), cfg.radius) = cv2.minEnclosingCircle(c)
        x4,y4,w2,h2=cv2.boundingRect(c)
        M = cv2.moments(c)
        cfg.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cfg.center1 = (int(M["m10"] / M["m00"]))
        cfg.center2 = (int(M["m01"] / M["m00"]))
        cfg.circle_coords=[x4,y4,(x4+w2),(y4+h2)]
        calc_spatials(cfg.circle_coords,roi_values)
        return cfg.x3,cfg.y3,cfg.radius, cfg.circle_coords,cfg.center1,cfg.center2,cfg.center
x4=0
y4=0
w2=0
h2=0

def draw_circulo_rectangulo(x3,y3,radius, circle_coords,center1,center2,center, frame, roi_valuescolor):
    cfg.radius=radius
    cc=circle_coords
    cfg.center1=center1
    cfg.center2=center2
    cfg.center=center
    cfg.x3=x3
    cfg.y3=y3
    if cfg.radius > 20:
        cv2.circle(frame, (int(cfg.x3), int(cfg.y3)), int(cfg.radius),
                  (0, 255, 0), 2)
        cv2.circle(frame, cfg.center, 5, (0, 255, 0), -1)
        cv2.rectangle(frame,(cc[0],cc[1]),(cc[2],cc[3]),(0,255,0),2)
        cv2.rectangle(roi_valuescolor,((cc[0]-40),cc[1]),((cc[2]-40),cc[3]),(0,255,0),2)
        ##-------------------------------xy calibration
        cv2.rectangle(roi_valuescolor,((cc[0]-20),cc[1]),((cc[2]-20),cc[3]),(255,255,255),2)
        cv2.rectangle(roi_valuescolor,((cc[0]-0),cc[1]),((cc[2]-0),cc[3]),(250,0,255),2)
        
 
    else:
        cv2.circle(frame, (int(cfg.x3), int(cfg.y3)), int(cfg.radius),
                 (0, 0, 255), 2)
        cv2.circle(frame, cfg.center, 5, (0, 0, 255), -1) 



def obst_avoid(frame,frame1,ta):   #frame=depthFrame, frame1=depthFrameColor,ta=0)
    roi_values=roidepth(frame,ta)
    rvob1=int(roi_values.shape[0]/2)
    rvob2=int(roi_values.shape[1]/2)
    rvobx1=int(rvob1-30)
    rvobx2=int(rvob1+90)
    rvoby1=int(rvob2-30)
    rvoby2=int(rvob2+90)
    roi_values_obst=[rvobx1,rvoby1,rvobx2,rvoby2]
    roi_valuescolor=roidepth(frame1,ta)
    sample=[]
    image=cv2.resize(roi_valuescolor,(240, 160))
    hog=cv2.HOGDescriptor()
    hist=hog.compute(image)
    sample.append(hist)
    sample=np.float32(sample)
    res=[]
    res=svm.predict(sample)
    label="clear" if int(res[1])==1 else "obstacle"
    color=(0,255,0) if int(res[1])==1 else (0,0,255)
    cv2.putText(roi_valuescolor,label,(20, 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, color, 2)
    image=roi_valuescolor
    hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    features= quantify_image(hsv, bins=(3, 3, 3))
    preds=model.predict([features])[0]
    label="anomaly" if preds==-1 else "normal"
    color=(0,0,255) if preds==-1 else (0,255,0)
    cv2.putText(roi_valuescolor,label,(10, 45), cv2.FONT_HERSHEY_SIMPLEX,
                 0.7, color, 2)
    cv2.rectangle(roi_valuescolor, (rvobx1, rvoby2), (rvobx2, rvoby1), (255,255,255), cv2.FONT_HERSHEY_SIMPLEX)  
    cv2.imshow("frame_roi",roi_valuescolor)
    res=int(res[1])
    print("preds:",preds,"res:",res)
    return preds,res,roi_values_obst
    


#########################ISOLATED FOREST###################
def quantify_image(image, bins=(4, 6, 3)):
    hist=cv2.calcHist([image], [0, 1, 2],None, bins,
                      [0, 180, 0, 256, 0, 256])
    hist=cv2.normalize(hist, hist).flatten()
    return hist
###########################################################

labelMap = ["background", "carB", "centro", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

nnPathDefault = str((Path(__file__).parent / Path('models/mobilenet.blob')).resolve().absolute())
parser = argparse.ArgumentParser()
parser.add_argument('nnPath', nargs='?', help="Path to mobilenet detection network blob", default=nnPathDefault)
parser.add_argument('-ff', '--full_frame', action="store_true", help="Perform tracking on full RGB frame", default=False)

args = parser.parse_args()


fullFrameTracking = args.full_frame
# Start defining a pipeline
pipeline = dai.Pipeline()

pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2020_1)

colorCam = pipeline.createColorCamera()
spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
objectTracker = pipeline.createObjectTracker()

xoutRgb = pipeline.createXLinkOut()
trackerOut = pipeline.createXLinkOut()
xoutDepth = pipeline.createXLinkOut()

xoutRgb.setStreamName("preview")
trackerOut.setStreamName("tracklets")
xoutDepth.setStreamName("depth")

colorCam.setPreviewSize(300, 300)
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
colorCam.setInterleaved(False)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

colorCam.setBoardSocket(dai.CameraBoardSocket.RGB)
##########################################################
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo.setOutputDepth(True)
stereo.setConfidenceThreshold(255)

stereo.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)
stereo.setLeftRightCheck(True)
#stereo.setDepthAlign(dai.CameraBoardSocket.RGB)###ERROR CRITICO!!! Don't work!!
######################################################################
spatialDetectionNetwork.setBlobPath(args.nnPath)
spatialDetectionNetwork.setConfidenceThreshold(0.45)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)
# Create outputs
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
# Link plugins CAM . NN . XLINK
colorCam.preview.link(spatialDetectionNetwork.input)
objectTracker.passthroughTrackerFrame.link(xoutRgb.input)

objectTracker.setDetectionLabelsToTrack([1])  # track only person
# possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS
objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
# take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
objectTracker.setTrackerIdAssigmentPolicy(dai.TrackerIdAssigmentPolicy.SMALLEST_ID)
#objectTracker.setTrackerIdAssigmentPolicy(dai.TrackerIdAssigmentPolicy.UNIQUE_ID)
objectTracker.out.link(trackerOut.input)
if fullFrameTracking:
    colorCam.setPreviewKeepAspectRatio(False)
    colorCam.video.link(objectTracker.inputTrackerFrame)
    objectTracker.inputTrackerFrame.setBlocking(False)
    # do not block the pipeline if it's too slow on full frame
    objectTracker.inputTrackerFrame.setQueueSize(2)
else:
    spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
##########################################
# Pipeline defined, now the device is connected to
with dai.Device(pipeline) as device:

    # Start the pipeline
    device.startPipeline()

    preview = device.getOutputQueue("preview", 4, False)
    tracklets = device.getOutputQueue("tracklets", 4, False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    
    startTime = time.monotonic()
    counter = 0
    fps = 0
    frame = None
    depthFrame = None
    depthFrameColor = None 
    sumador3=0
    sumador=0
    sumador6=0 
    datain=0   
    explorar=1
    avoid_collision=0
    preds=0    
    preds2=0   
    distdif=0    
    values=0
    tidi4=0
    tidi5=0
    tidist4=0
    tidist5=0
    
    while(True):
        
        imgFrame = preview.get()
        track = tracklets.get()
        depth = depthQueue.get()
        
        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        color = (255, 0, 0)
        #color = (255, 255, 255) 
        frame = imgFrame.getCvFrame()
        trackletsData = track.tracklets
        depthFrame = depth.getFrame()
        depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        roi_values=roidepth(depthFrame,1)
        roi_valuescolor=roidepth(depthFrameColor,1)
        
        for t in trackletsData:
            roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
            x1 = int(roi.topLeft().x)
            y1 = int(roi.topLeft().y)
            x2 = int(roi.bottomRight().x)
            y2 = int(roi.bottomRight().y)
            fx1=int(frame.shape[0]-frame.shape[0])
            fy1=int(frame.shape[1]-frame.shape[1])
            fx2=int(frame.shape[0])
            fy2=int(frame.shape[1])
            tidi=int (t.id)
            tist=[t.status.value]
            tidist= int(t.spatialCoordinates.z)
            if (tist==[1]):
                cfg.dicdis={tidi:tidist}
                tidi0,tidi1=next(iter(cfg.dicdis.items()))
                if (tidi1>0):
                    if cfg.tidi2==-1 :
                        cfg.tidi2=tidi0
                        cfg.tidi3=tidi1
                if (cfg.tidi3>tidi1): 
                    if cfg.tidi2!=tidi0:
                        cfg.tidi2=tidi0
                        tidi3=tidi1
                    else:
                        cfg.tidi2=cfg.tidi2
                        cfg.tidi3=tidi1
                if (cfg.tidi3<tidi1):
                   if cfg.tidi2==tidi0:
                       cfg.tidi2=cfg.tidi2
                       cfg.tidi3=tidi1
                   else:
                        tidi4=(tidi1-cfg.tidi3)/10
                        if tidi4>5:
                            cfg.tidi5=tidi4
                            print(cfg.tidi5)
            if tidi==cfg.tidi2 and tist==[1]:
                
                cfg.frames_since_cardos = 0
                cfg.distx=int(t.spatialCoordinates.x)
                cfg.disty=int(t.spatialCoordinates.y)
                cfg.distz=int(t.spatialCoordinates.z)
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)
                cfg.centro1= x1+((x2-x1)/2)
                cfg.centro2= y1+((y2-y1)/2)
                cfg.sumador+=1
                cfg.sumador21+=1
                cfg.sumador22+=1
                cfg.centrox=cfg.centro1+cfg.centrox
                cfg.centroy=cfg.centro2+cfg.centroy                               
                if cfg.sumador== 4:
                    cfg.centroy= cfg.centroy/cfg.sumador
                    cfg.centrox=cfg.centrox/cfg.sumador
                    turn_x = float(cfg.centrox - (300/2))
                    turn_y = float(cfg.centroy - (300/2))
                    turn_x /= float(300/2)
                    turn_y /= float(300/2)
                    turn_x *=2# VFOV
                    turn_y *=2# HFOV
                    if abs(turn_x) > HYSTERESIS:
                        cam_pan  += -turn_x
                    if abs(turn_y) > HYSTERESIS:
                        cam_tilt += turn_y
                    # Clamp Pan/Tilt to 0 to 180 degrees
                    cam_pan = max(0, min(180,cam_pan))
                    cam_tilt = max(0, min(180,cam_tilt))
                    pm.setRotationAngle(0, cam_pan)
                    pm.setRotationAngle(1, cam_tilt)
                    cfg.cam_pan2=cam_pan
                    if cfg.campanh==0:# campanh=cam_pan handle, this variable make 3rd servo independient/ esta variable hace independiente el 3er servo
                        pm.servoangle(cfg.cam_pan2)
                        cfg.cam_pan3=cfg.cam_pan2
                    else:
                        pm.servoangle(cfg.cam_pan3)
                    fpan=cam_pan-90
                    ftilt=cam_tilt
                    #cfg.reswtr=trated_weed(cfg.weedtrat_coords,frame)
                    #reswtra=trated_weed2(weedtrat_coords,frame)
                    #cfg.reswtr2=cfg.reswtr+cfg.reswtr2
                    #reswtra2=reswtra+reswtra2
                    cfg.res=circle(frame,cfg.centrox,cfg.centroy)
                    if cfg.res is not None:
                        cfg.x3,cfg.y3,cfg.radius,cfg.circle_coords,cfg.center1,cfg.center2,cfg.center=circle(frame,cfg.centrox,cfg.centroy)
                        draw_circulo_rectangulo(cfg.x3,cfg.y3,cfg.radius,cfg.circle_coords,cfg.center1,cfg.center2,cfg.center,frame,roi_valuescolor)
                        spatial_coords=calc_spatials(cfg.circle_coords,roi_values)
                        cfg.circulo_z=int(spatial_coords[2])
                        cfg.circulo_z2=cfg.circulo_z2+cfg.circulo_z
                        distapl=distancia(spatial_coords,cfg.centrox,cfg.centroy,frame)
                        cfg.distapl2=cfg.distapl2+distapl
                        cfg.center12=cfg.center12+cfg.center1
                        cfg.center22=cfg.center22+cfg.center2
                        cfg.centrox2=cfg.centrox+cfg.centrox2
                        cfg.centroy2=cfg.centroy2+cfg.centroy 
                    distlin1 = dist_lineal(ftilt,cfg.distz)
                    cfg.distlin2 = cfg.distlin2 + distlin1
                    cfg.centrox=0
                    cfg.centroy=0
                    cfg.center1=0
                    cfg.center2=0
                    cfg.circulo_z=0 
                    distlin1=0
                    cfg.distapl=0 
                    #cfg.reswtr=0
                    ##cfg.reswtra=0
                    cfg.sumador=0                 
                if (cfg.sumador21==40):
                    cfg.distapl2=cfg.distapl2/10
                     cfg.distlin2= cfg.distlin2/10
                    if ftilt>2 and ftilt<134 : 
                        if (cfg.sumk2stepper2 <-1000):
                            cfg.distlin2=cfg.distlin2+10
                        pm.move(cfg.distlin2,0,2,3,6)
                    if (ftilt < 110) and (fpan > 20):
                        pm.center_to_left(22900)
                    if (ftilt < 110) and (fpan < -20):
                        pm.center_to_right(21700) 
                    if (ftilt>=134)and (ftilt<150):
                        pm.forward1(0,0)
                        if (cfg.distapl2>0):
                            cfg.timeantes=time.monotonic()
                            if (cfg.distapl2 < 75)and (cfg.wflag==0)and (cfg.difheight<10)and (dify>-10) and (dify<10) and (difx<10) and (difx>-10):#(sbf==1):#THIS IS/ESTE ES// CLOSE, CENTERED, 1ST TIME/ CERCANO, CENTRADO 1ER VEZ!!!!
                                cfg.sprayap=1
                        cfg.distapl2=0                        
                        cfg.distlin2=0
                        cfg.reswtr=0                        
                    cfg.sumador21=0
                if (cfg.sumador22==cfg.stepper_velocity):
                    sv=cfg.stepper_velocity/4
                    cfg.circulo_z2=cfg.circulo_z2/sv
                    cfg.center22=cfg.center22/sv
                    cfg.center12=cfg.center12/sv
                    cfg.centrox2=cfg.centrox2/sv
                    cfg.centroy2=cfg.centroy2/sv
                    cfg.difheight=(cfg.distz-cfg.circulo_z2)/10                                             
                    difx=(cfg.centrox2)-cfg.center12
                    dify=(cfg.centroy2)-cfg.center22 #143 pixels are 3cms lineals, 143/3= aprox 50pixels/cm
                     print("DIFX:",difx,"DIFY:",dify)
                    if (ftilt>=134)and (ftilt<150):
                        pm.forward1(0,0)
                    ##++++++++++++++++++++++++++++++APPROACH IN X AXIS//APROXIMACION EN EJE X
                    if (ftilt > 110) and (ftilt< 140) and (fpan <-20) and (fpan >-30) and (cfg.sumk1stepper2>-3900) :
                        anglefb=fpan*0.6
                        anglefb=anglefb*(-1)
                        step=anglefb*300
                        sr=100
                        pm.frontback(step,sr)
                        pm.kit1.stepper2.release()
                    if (ftilt > 110) and (ftilt< 140) and (fpan > 20) and (fpan < 30) and (cfg.sumk1stepper2<3900) :
                        angleff=fpan*0.6
                        step=angleff*300
                        sr=100
                        pm.frontfront(step,sr)                         
                        pm.kit1.stepper2.release() 
                    if ftilt>134 :#and ftilt<146:
                        ##++++++++++++++++++++CENTERED IN X-Y AXIS// CENTRADO EN EJES X-Y
                        if dify<-10:
                             if (cfg.sumk2stepper2<-2500):
                                pm.move(17,0,2,2,4)
                             else:
                                pm.sideback(140)##250=1cms lineal 250steps=1cm, 10 pixels=1cm, totalpixels/10=cms, cms*250=steps
                                print("ajust y")
                        if dify>10:                           
                            pm.sidefront(140)##300=1cms lineal 300steps=1cm, 10 pixels=1cm, totalpixels/10=cms, cms*300=steps
                        if difx<-10:
                            cfg.campanh=1
                            angle=difx/5
                            angle=angle*(-1)
                            cfg.cam_pan3=cfg.cam_pan3+angle
                            pm.servoangle(cfg.cam_pan3)
                        if difx>10:
                            cfg.campanh=1
                            angle=difx/5
                            cfg.cam_pan3=cfg.cam_pan3-angle
                           
                            pm.servoangle(cfg.cam_pan3)
                        ##++++++++++++++++++++++++++++++++++CLOSE IN Z AXIS/ CERCANO EN EJE Z
                        
                        print("diheight",cfg.difheight,"distz:",cfg.distz,"circuloz:",cfg.circulo_z2)
                        if (cfg.difheight > 10):
                            pm.heightdown(140)
                        if (cfg.difheight < 3):
                            pm.heightup(140)
                    if (cfg.sprayap==1):
                         pm.spray()
                         print('BURN..!!')
                    if (cfg.wflag==1) and (cfg.height_cal==1):
                        cam_tilt=120
                        pwm.setRotationAngle(1,cam_tilt)
                        pm.move(cfg.tidi5,0,2,2,4) 
                        if cfg.dist_dif<0:
                            cfg.wflag=0
                    cfg.sumador22=0
                    cfg.circulo_z2=0
                    cfg.centrox2=0
                    cfg.centroy2=0
                    cfg.center12=0
                    cfg.center22=0
            try:
                label = labelMap[t.label]
            except:
                label = t.label
            #statusMap = {dai.Tracklet.TrackingStatus.NEW : "NEW", dai.Tracklet.TrackingStatus.TRACKED : "TRACKED", dai.Tracklet.TrackingStatus.LOST : "LOST"}
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            #cv2.putText(frame, statusMap[t.status], (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
            cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        cv2.imshow("tracker", frame)
        cv2.imshow("roiDepth", roi_valuescolor)
        if not(trackletsData):
            cfg.campanh=0
            cfg.wflag=0
            cfg.weed_height=2
            cfg.sumador2+=1
            if (sumador2==5)and (datain==0):
                headingar()
                datain=1
            serialdata()
            if (cfg.xy_calibration==0):
                    cam_tilt=140
                    cam_pan=90
                    cfg.cam_pan2=cam_pan+5
                    #print("cam_tilt2:",cam_tilt)
                    pm.setRotationAngle(0,cam_pan)
                    pm.setRotationAngle(1,cam_tilt)
                    pm.servoangle(cfg.cam_pan2)
                    if (cam_tilt==140):
                        cfg.centro1= frame.shape[0]/2
                        cfg.centro2= frame.shape[1]/2
                        cfg.sumador0+=1
                        cfg.sumador3+=1
                        cfg.centrox=cfg.centro1+cfg.centrox
                        cfg.centroy=cfg.centro2+cfg.centroy                               
                        if cfg.sumador0== 2:
                            cfg.centroy= cfg.centroy/cfg.sumador0
                            cfg.centrox=cfg.centrox/cfg.sumador0
                            cfg.resoff=circle(frame,cfg.centrox,cfg.centroy)
                            if cfg.resoff is None:
                                if forward:
                                    cam_pan = cam_pan + PAN_INCREMENT
                                else:
                                    cam_pan = cam_pan - PAN_INCREMENT
                                if cam_pan < PAN_RANGE[0]:
                                    forward = True                                   
                                    cam_pan = PAN_RANGE[0]           #                     
                                elif cam_pan > PAN_RANGE[1]:
                                    forward = False                                    
                                    cam_pan = PAN_RANGE[1]
                                pm.setRotationAngle(0, cam_pan)
                                cfg.sumador0=0
                            if cfg.resoff is not None:
                                cfg.x3,cfg.y3,cfg.radius,cfg.circle_coords,cfg.center1,cfg.center2,cfg.center=circle(frame,cfg.centrox,cfg.centroy)
                                draw_circulo_rectangulo(cfg.x3,cfg.y3,cfg.radius,cfg.circle_coords,cfg.center1,cfg.center2,cfg.center,frame,roi_valuescolor)
                                spatial_coords=calc_spatials(cfg.circle_coords,roi_values)
                                cfg.circulo_z=int(spatial_coords[2])
                                cfg.circulo_z2=cfg.circulo_z2+cfg.circulo_z
                            cfg.sumador0=0
                            cfg.centrox=0
                            cfg.centroy=0
                        if cfg.sumador3==4: 
                                    cfg.circulo_z2=cfg.circulo_z2/2
                                    if (cfg.circulo_z2>45):
                                        cfg.difheight=(cfg.circulo_z2/10)-45
                                        pm.heightup(140)
                                    cfg.circulo_z2=0
                                    cfg.sumador3=0
                                    if ((cfg.circulo_z2/10)<=45):
                                        print("cz2:",cfg.circulo_z2)
                                        if (cfg.sumk1stepper2<-2900) or (cfg.sumk1stepper2>2900) or(cfg.sumk2stepper2<-3900):  
                                            cfg.calibration2=1
                                    cfg.sumador3=0
                                    cv2.imshow("roiDepth", roi_valuescolor)
                                    cv2.imshow("tracker", frame) 
                    if (cfg.sumk1stepper2<0) and (cfg.calibration2==1) :
                            step=300
                            sr=100
                            pm.frontfront(step,sr)
                            pm.kit1.stepper2.release()
                    if (cfg.sumk1stepper2>0) and (cfg.calibration2==1) :
                            step=300
                            sr=100
                            pm.frontback(step,sr)
                            pm.kit1.stepper2.release()
                    
                    if (cfg.sumk2stepper2<0) and (cfg.calibration2==1)  :  
                            step=140
                            pm.sidefront(step)
                    if((cfg.circulo_z2/10)<=45)and(cfg.sumk1stepper2==0) and (cfg.sumk1stepper2==0) and (cfg.sumk2stepper2==0):
                        cfg.xy_calibration=1
            if (cfg.xy_calibration==1):
                if (cfg.sumador2<10): 
                    if (cam_tilt==90):
                        if cfg.sumador2<5:
                            preds,res,roi_values_obst=obst_avoid(depthFrame,depthFrameColor,0)
                            if preds==-1:
                                preds=0
                            cfg.preds2 +=preds
                            cfg.res2 += res
                        if cfg.sumador2==5:
                            cfg.preds2=cfg.preds2/5
                            cfg.res2=cfg.res2/5
                            if (cfg.preds2<0.7) and (cfg.res2<0.7):
                                avoid_collision=1
                                explorar=0
                                spatial_coords=calc_spatials(roi_values_obst,roi_values)
                                obst_z = int(spatial_coords[2])/10
                                if (obst_z < 100):
                                    distlin2=(100-obst_z)
                                    pm.move(distlin2,1,3,7,2)
                                    cfg.sumador2=0
                            elif(cfg.sumador2==8):
                                 cfg.sumador2=0
                        if avoid_collision==1 and cfg.sumador2>5:
                                spatial_coords=calc_spatials(roi_values_obst,roi_values)
                                obst_z = int(spatial_coords[2])/10
                                if obst_z>100:
                                    cam_pan=130
                                    pm.setRotationAngle(0, cam_pan)
                                    pm.reverse(0,0)
                                    avoid_collision=0                                    
                                cfg.sumador2=0
                        if (cfg.preds2>0.7) and (cfg.res2>0.7)and (explorar==0)and (cfg.sumador2>5):
                                pm.center_to_left(23300)
                                pm.move(100,0,3,5,2)
                                if (cfg.dist_dif<0)and (explorar==0):
                                    pm.forward1(0,0)
                                    cam_pan=92                                    
                                    pm.setRotationAngle(0, cam_pan)                                    
                                cfg.sumador2=0
                        if (cfg.preds2>0.7) and (cfg.res2>0.7)and (explorar==0)and (cfg.sumador2>5) and  (cam_pan==92):
                                pm.left_to_center()
                                cam_pan=80
                                pm.setRotationAngle(0, cam_pan)                                    
                                cfg.sumador2=0
                                cfg.res2=0
                                cfg.preds2=0
                        if(cfg.preds2>0.7) and  (cfg.res2>0.7)and (explorar==0)and (cfg.sumador2>5) and  (cam_pan==80):
                                pm.center_to_right(21100)
                                pm.move(100,0,3,5,2)
                                if (cfg.dist_dif<0)and (explorar==0):
                                    pm.forward1(0,0)
                                    cam_pan=85                                    
                                    pwm.setRotationAngle(0, cam_pan)                                    
                                cfg.sumador2=0
                                cfg.res2=0
                                cfg.preds2=0
                        if (cfg.preds2>0.7) and (cfg.res2>0.7)and (explorar==0)and (cfg.sumador2>5) and (cam_pan==85):                                    
                                pm.right_to_center()
                                cam_pan=110
                                pm.setRotationAngle(0, cam_pan)                                    
                                cfg.sumador2=0
                                cfg.res2=0
                                cfg.preds2=0
                        if (cfg.preds2>0.7) and (cfg.res2>0.7)and (explorar==0)and (cfg.sumador2>5) and (cam_pan==110):
                                pm.center_to_left(22500)
                                pm.move(100,0,3,5,2)
                                print("distdif:",cfg.dist_dif)
                                if (cfg.dist_dif<0)and (explorar==0):
                                    pm.forward1(0,0)
                                    cam_pan=90                                    
                                    pm.setRotationAngle(0, cam_pan)
                                    explorar=1                                    
                                cfg.sumador2=0
                                cfg.res2=0
                                cfg.preds2=0
                        if (preds2>0.7) and (res2>0.7) and (explorar==1):
                                avoid_collision=0
                                explorar=1
                                pm.reverse(0,0)
                                cam_tilt=95
                                pm.setRotationAngle(1, cam_tilt)
                                cfg.sumador2=0
                                cfg.res2=0
                                cfg.preds2=0
               
                    if explorar==1 and avoid_collision==0:
                        if cfg.sumador2==7:
                            frames_since_cardos = frames_since_cardos + 1      
                            if frames_since_cardos > ROAM_START:
                                if forward:
                                    cam_pan = cam_pan + PAN_INCREMENT
                                else:
                                    cam_pan = cam_pan - PAN_INCREMENT
                                if cam_pan < PAN_RANGE[0]:
                                    forward = True                                   
                                    cam_tilt = cam_tilt - TILT_INCREMENT   
                                    cam_pan = PAN_RANGE[0]           #                     
                                elif cam_pan > PAN_RANGE[1]:
                                    forward = False                                    
                                    cam_tilt = cam_tilt - TILT_INCREMENT
                                    cam_pan = PAN_RANGE[1]                           
                                if cam_tilt < (TILT_RANGE[0]):
                                    cam_tilt = TILT_RANGE[1]
                                if cam_pan==90 and cam_tilt==90:
                                    cfg.frames_since_cardos=0
                                if cam_tilt==90:
                                    if (cfg.avance==0):
                                         pm.move(40,0,3,5,2)
                                pm.setRotationAngle(0, cam_pan)
                                pm.setRotationAngle(1, cam_tilt)
                            cfg.sumador2=0
            cv2.imshow("roiDepth", roi_valuescolor)
            cv2.imshow("tracker", frame)      
        
        if cv2.waitKey(1) == ord('q'):
            pm.forward1(0,0)
            stop()
            break
 