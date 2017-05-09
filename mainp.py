import math
import almath
import time
import argparse
import redball
import time
import landmark
from naoqi import ALProxy

def StandToGetstick():
    postureProxy.goToPosture("Stand", 0.4)



    names = ['RShoulderRoll','RElbowRoll']
    angleLists = [-20.0*almath.TO_RAD , 2.0*almath.TO_RAD]
    timeLists = [1.0,1.5]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)



    names = ['RShoulderRoll','RElbowYaw','RShoulderPitch']
    angleLists = [-45.0*almath.TO_RAD , -4.0*almath.TO_RAD , 4.0*almath.TO_RAD]
    timeLists = [1.0,1.5,2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

    motionProxy.openHand('RHand')
    time.sleep(20.0)
    motionProxy.closeHand('RHand')
    motionProxy.setStiffnesses('RHand', 1.0)

    names = ['RShoulderPitch','RElbowYaw','RShoulderRoll']
    angleLists = [84.0*almath.TO_RAD , 68.0*almath.TO_RAD , -25.0*almath.TO_RAD]
    timeLists = [1.0,1.5,2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

def HitBall(power):
    postureProxy.goToPosture("Stand", 0.4)
    names = ['RShoulderRoll','RElbowRoll']
    angleLists = [-25.0*almath.TO_RAD, 2.0*almath.TO_RAD]
    timeLists = [1.0,1.5]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

    names = ['RShoulderRoll','RElbowYaw','RShoulderPitch']
    angleLists = [-45.0*almath.TO_RAD , -4.0*almath.TO_RAD , 4.0*almath.TO_RAD]
    timeLists = [1.0,1.5,2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

    names = ['RWristYaw','RElbowYaw','RShoulderRoll','RElbowRoll','RShoulderPitch']
    angleLists = [-100.0*almath.TO_RAD , 100.0*almath.TO_RAD , 0.0*almath.TO_RAD , 9.0*almath.TO_RAD , 9.0*almath.TO_RAD]
    timeLists = [1.0,1.5,2.0,2.5,3]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(2.0)

    names = 'RWristYaw'
    angleLists = 10.0*almath.TO_RAD
    timeLists = power
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)
    time.sleep(1.0)

    names = ['RShoulderRoll', 'RElbowRoll', 'RShoulderPitch']
    angleLists = [ 0.0 * almath.TO_RAD, -20.0 * almath.TO_RAD, -20.0 * almath.TO_RAD]
    timeLists = [1.0, 1.5, 2.0, 2.5, 3]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(2.0)


    names = 'RWristYaw'
    angleLists = -100.0*almath.TO_RAD
    timeLists = 1.0
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

    names = ['RShoulderPitch','RElbowRoll','RShoulderRoll','RElbowYaw','RWristYaw']
    angleLists = [4*almath.TO_RAD , -45.0*almath.TO_RAD , -45.0*almath.TO_RAD , -4.0*almath.TO_RAD , 4.0*almath.TO_RAD]
    timeLists = [1.0,1.5,2.0,2.5,3]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(2.0)

    names = ['RShoulderPitch','RElbowYaw','RShoulderRoll']
    angleLists = [84.0*almath.TO_RAD , 68.0*almath.TO_RAD , -25.0*almath.TO_RAD]
    timeLists = [1.0,1.5,2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

def Movex(d):
    sleep_time=0.7
    mdistance=0.15
    if(d>0):
        while(1):
            if (d<=mdistance):
                x = d
                y = 0.0
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                break
            else:
                x =mdistance
                y = 0.0
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                d=d-mdistance
    else:
        d=-d
        while(1):
            if (d<=mdistance):
                x = -d
                y = 0.0
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                break
            else:
                x = -mdistance
                y = 0.0
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                d=d-mdistance

def Movey(d):
    sleep_time = 0.5
    mdistance = 0.15
    if(d>0):
        while(1):
            if (d<=mdistance):
                x = 0
                y = d
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                break
            else:
                x = 0.0
                y = mdistance
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                d=d-mdistance
    else:
        d=-d
        while(1):
            if (d<=mdistance):
                x = 0
                y = -d
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                break
            else:
                x = 0.0
                y = -mdistance
                theta = 0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                d=d-mdistance


def Movet(t):
    sleep_time = 0.7
    mdistance = math.pi/10
    if(t>0):
        while(1):
            if (t<=( mdistance)):
                x = 0
                y = 0
                theta = t
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                break
            else:
                x = 0.0
                y = 0.0
                theta = mdistance
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                t=t-( mdistance)
    else:
        t=-t
        while(1):
            if (t<=( mdistance)):
                x = 0
                y = 0
                theta = -t
                motionProxy.moveTo(x, y, theta)
                time.sleep(mdistance)
                break
            else:
                x = 0.0
                y = 0.0
                theta = -math.pi/24
                motionProxy.moveTo(x, y, theta)
                time.sleep(sleep_time)
                t=t-( mdistance)


def turnhead(Yaw,Picth):
    names      = ["HeadPitch","HeadYaw"]
    angleLists = [Picth*almath.TO_RAD, Yaw*almath.TO_RAD]
    timeLists  = [1.0, 2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)


def DetecteLandmark():
    camProxy.setParam(18, 0)
    landmarkTheoreticalSize=0.1
    currentCamera = "CameraTop"
    ip=robotIP
    period = 500
    landMarkProxy.subscribe("Test_LandMark", period, 0.0 )
    for i in range(0, 10):
        time.sleep(0.5)
        markData = memoryProxy.getData("LandmarkDetected")
        if(markData is None or len(markData) == 0):
            continue
        else:
            if(len(markData)==1):
                wzCamera = markData[1][0][0][1]
                wyCamera = markData[1][0][0][2]

                # Retrieve landmark angular size in radians.
                angularSize = markData[1][0][0][3]

                # Compute distance to landmark.
                distanceFromCameraToLandmark = landmarkTheoreticalSize / ( 2 * math.tan( angularSize / 2))

                motionProxy = ALProxy("ALMotion", ip, 9559)

                # Get current camera position in NAO space.
                transform = motionProxy.getTransform(currentCamera, 2, True)
                transformList = almath.vectorFloat(transform)
                robotToCamera = almath.Transform(transformList)

                # Compute the rotation to point towards the landmark.
                cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

                # Compute the translation to reach the landmark.
                cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

                # Combine all transformations to get the landmark position in NAO space.

                robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform *cameraToLandmarkTranslationTransform
                landMarkProxy.unsubscribe("Test_LandMark")
                print  "landmark detect",robotToLandmark.r1_c4,robotToLandmark.r2_c4,robotToLandmark.r3_c4,distanceFromCameraToLandmark
                return 1,robotToLandmark.r1_c4,robotToLandmark.r2_c4,robotToLandmark.r3_c4,distanceFromCameraToLandmark
            else:
                wzCamera = markData[1][0][0][1]
                wyCamera = markData[1][0][0][2]

                # Retrieve landmark angular size in radians.
                angularSize = markData[1][0][0][3]

                # Compute distance to landmark.
                distanceFromCameraToLandmark = landmarkTheoreticalSize / (2 * math.tan(angularSize / 2))

                motionProxy = ALProxy("ALMotion", ip, 9559)

                # Get current camera position in NAO space.
                transform = motionProxy.getTransform(currentCamera, 2, True)
                transformList = almath.vectorFloat(transform)
                robotToCamera = almath.Transform(transformList)

                # Compute the rotation to point towards the landmark.
                cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

                # Compute the translation to reach the landmark.
                cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

                # Combine all transformations to get the landmark position in NAO space.

                robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
                landMarkProxy.unsubscribe("Test_LandMark")
                print  "landmark detect", robotToLandmark.r1_c4, robotToLandmark.r2_c4, robotToLandmark.r3_c4, distanceFromCameraToLandmark
                return 1, robotToLandmark.r1_c4, robotToLandmark.r2_c4, robotToLandmark.r3_c4, distanceFromCameraToLandmark


    landMarkProxy.unsubscribe("Test_LandMark")
    print  "landmark detect fff"
    return 0,0,0,0,0

def DetecteLandmark_my():
    for i in range(0, 3):
        time.sleep(0.5)
        size, left = landmark.landmarkdetect(IP, PORT, 0)
        if((size!=0)&(left!=0)):
            return 1,size, left
        else:
            print  "landmark_my detect fff"
            return 0,0,0







def closeball1(threshold):
    
    LEFT_=0
    redball.lookat(robotIP, PORT)
    count=1
    flag_p=0
    flag_n=0
    while (1):
        size, left, top, cam, isfind = redball.findball(robotIP, PORT)
        if isfind == 1:
            
            if (cam == 1):

                if (left <=threshold) & (left >= -threshold):
                    break
                elif (left > threshold):
                    flag_p=1
                    if((flag_p==1)&(flag_n==1)):
                        count=count+1
                        flag_n=0
                    x = 0.0
                    y = 0.0
                    theta = -1*math.pi / (8*(2*count))
                    motionProxy.moveTo(x, y, theta)
                    time.sleep(1.5)

                elif (left <- threshold):
                    flag_n=1
                    if((flag_p==1)&(flag_n==1)):
                        count=count+1
                        flag_p=0
                    x = 0.0
                    y = 0.0
                    theta = math.pi / (8*(2*count))
                    motionProxy.moveTo(x, y, theta)
                    time.sleep(1.5)
            else:
                if(top<=-10):
                    if (left < 30):
                        x = 0.0
                        y = 0.0
                        theta = math.pi / 15
                        motionProxy.moveTo(x, y, theta)
                        time.sleep(1.5)
                    elif (left > 30):
                        x = 0.0
                        y = 0.0
                        theta = -1*math.pi / 15
                        motionProxy.moveTo(x, y, theta)
                        time.sleep(1.5)
                    Movex(0.45)
                elif(-20<=top<=50):
                    if (left < 25):
                        x = 0.0
                        y = 0.0
                        theta = math.pi / 15
                        motionProxy.moveTo(x, y, theta)
                        time.sleep(1.5)
                    elif (left > 25):
                        x = 0.0
                        y = 0.0
                        theta = -1*math.pi / 15
                        motionProxy.moveTo(x, y, theta)
                        time.sleep(1.5)
                    Movex(0.30)
                else:
                    if (left < 25):
                        x = 0.0
                        y = 0.0
                        theta = math.pi / 15
                        motionProxy.moveTo(x, y, theta)
                        time.sleep(1.5)
                    elif (left > 25):
                        x = 0.0
                        y = 0.0
                        theta = -1*math.pi / 15
                        motionProxy.moveTo(x, y, theta)
                        time.sleep(1.5)
                    Movex(0.15)
                    
                    
        else:
            if(LEFT_==0):
                x = 0.0
                y = 0.0
                theta = math.pi / 8
                motionProxy.moveTo(x, y, theta)
            else:
                x = 0.0
                y = 0.0
                theta = -math.pi / 8
                motionProxy.moveTo(x, y, theta)
#**********************************
def closeball1_right(threshold):
    LEFT_ = 1
    redball.lookat(robotIP, PORT)
    count = 1
    flag_p = 0
    flag_n = 0
    while (1):
        size, left, top, cam, isfind = redball.findball(robotIP, PORT)
        if (isfind == 1):

            if (cam == 1):

                if (left <= threshold) & (left >= -threshold):
                    break
                elif (left > threshold):
                    flag_p = 1
                    if ((flag_p == 1) & (flag_n == 1)):
                        count = count + 1
                        flag_n = 0
                    x = 0.0
                    y = 0.0
                    theta = -1 * math.pi / (8 * (2 * count))
                    motionProxy.moveTo(x, y, theta)
                    time.sleep(1.5)

                elif (left < - threshold):
                    flag_n = 1
                    if ((flag_p == 1) & (flag_n == 1)):
                        count = count + 1
                        flag_p = 0
                    x = 0.0
                    y = 0.0
                    theta = math.pi / (8 * (2 * count))
                    motionProxy.moveTo(x, y, theta)
                    time.sleep(1.5)
            else:
                Movex(0.25)
                if (left < 20):
                    x = 0.0
                    y = 0.0
                    theta = math.pi / 16
                    motionProxy.moveTo(x, y, theta)
                    time.sleep(1.5)
                elif (left > 20):
                    x = 0.0
                    y = 0.0
                    theta = -1 * math.pi / 16
                    motionProxy.moveTo(x, y, theta)
                    time.sleep(1.5)
        else:
            if (LEFT_ == 0):
                x = 0.0
                y = 0.0
                theta = math.pi / 8
                motionProxy.moveTo(x, y, theta)
            else:
                x = 0.0
                y = 0.0
                theta = -math.pi / 8
                motionProxy.moveTo(x, y, theta)

#*********************************************************8
def closeball2(threshold,close1):
    redball.lookat(robotIP, PORT)
    while (1):

        size, left, top, cam, isfind = redball.findball(robotIP, PORT)
        if (isfind == 1):

            if (top >= threshold):
                closeball1(close1)
                break



            elif(-50<=top<30):
                x = 0.05
                y = 0.0
                theta=0
                motionProxy.moveTo(x, y, theta)
                time.sleep(1.8)
                closeball1(close1)
            elif(30<=top<threshold):
                x = 0.02
                y = 0.0
                theta=0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(1.8)
                closeball1(27)
                
            elif (top <-50):
                x = 0.04
                y = 0.0
                theta=0.0
                motionProxy.moveTo(x, y, theta)
                time.sleep(1.8)

        else:
            pass

        

  #  distance in meter 0.011   
def runtohitball90(threshold , topdistance ,ditance):
    #turn head
    #detecteland
    #Movex(-0.08)
    close1_=23

    count=0
    while(1):
        names = ["HeadPitch", "HeadYaw"]
        angleLists = [0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
        timeLists = [1.0, 2.0]
        isAbsolute = True
        motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        time.sleep(2.0)

        isfindLand,x,y,z,d=DetecteLandmark()

        time.sleep(2.0)
        print  "landmark"
        if(isfindLand==0):
            print x, threshold + ditance, ditance - threshold
            count=count+1
            Movey(0.10)
            redball.lookat(robotIP, PORT)
            closeball1(17)
            closeball2(topdistance,close1_)
        elif(((x-ditance)<threshold)&((x-ditance)>-threshold)):
            print x,threshold+ditance,ditance-threshold
            return x,y,z,d
        elif(x-ditance < -threshold):
            print x, threshold + ditance, ditance - threshold
            Movey(-0.02)
            redball.lookat(robotIP, PORT)
            #*************************************
            closeball1(17)
            closeball2(topdistance,close1_)
        else:
            count=count+1
            Movey(0.05)
            redball.lookat(robotIP, PORT)
            closeball1(17)
            closeball2(topdistance,close1_)



            
#*************************************************
def runtohitball90_n(threshold , topdistance ,ditance):
    #turn head
    #detecteland
    #Movex(-0.08)
    close1_=23
    count=0
    while(1):
        names = ["HeadPitch", "HeadYaw"]
        angleLists = [0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
        timeLists = [1.0, 2.0]
        isAbsolute = True
        motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        time.sleep(2.0)

        isfindLand,x,y,z,d=DetecteLandmark()

        time.sleep(2.0)
        print  "landmark"
        if(isfindLand==0):
            count=count+1
            Movey(-0.06)
            redball.lookat(robotIP, PORT)
            closeball1(17)
            closeball2(topdistance,close1_)
        elif(((x-ditance)<threshold)&((x-ditance)>-threshold)):
            return x,y,z,d
        elif ((x-ditance)<threshold):

            Movey(0.02)
            redball.lookat(robotIP, PORT)
            #*************************************
            closeball1(17)
            closeball2(topdistance,close1_)
        else:
            count=count+1
            Movey(-0.03)
            redball.lookat(robotIP, PORT)
            closeball1(17)
            closeball2(topdistance,close1_)
#**************************************



            
def runtohitball(threshold , topdistance ,ditance):
    #***********************
    close1_=23
    names = ["HeadPitch", "HeadYaw"]
    angleLists = [0.0 * almath.TO_RAD, 0.0 * almath.TO_RAD]
    timeLists = [1.0, 2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

    #*****************************
    while(1):
        isfindLand,x,y,z,d=DetecteLandmark()
        #find landmark
        if(isfindLand==1):
            if(x<0):
                #this value need y=0.25+((-x)/y)*(-x),x=0.25
                Movey(0.3)
                Movex(0.3+(((-x)/y)*(-x)))
                print (0.25+(((-x)/y)*(-x)))
                #20cm -> value  = 50?
                closeball1(17)
                closeball2(topdistance,close1_)
                return runtohitball90(threshold , topdistance ,ditance)
            if(x>=0):
                #theoretics value
                mm=(-0.2/(math.tan(((3.14159/4)-math.atan(x/y)))))
                print mm
                Movey(mm)
                #Movey(0.15)
                closeball1(17)
                closeball2(topdistance,close1_)
                return runtohitball90(threshold , topdistance ,ditance)
        #if not find ball
        else:
            #first consider the best position
            
            #***********************
            names = ["HeadPitch", "HeadYaw"]
            angleLists = [0.0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
            timeLists = [1.0, 2.0]
            isAbsolute = True
            motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
            time.sleep(1.0)

            #*****************************

            
            isfindLand,x,y,z,d=DetecteLandmark()
            if(isfindLand==1):
                closeball1(17)
                closeball2(topdistance, close1_)
                return runtohitball90(threshold, topdistance, ditance)
                
            else:
                for i in range (0,5):
                    #***********************
                    names = ["HeadPitch", "HeadYaw"]
                    angleLists = [0.0 * almath.TO_RAD, (30.0+i*30.0) * almath.TO_RAD]
                    timeLists = [1.0, 2.0]
                    isAbsolute = True
                    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
                    time.sleep(1.0)

                    #*****************************
                    
                    isfindLand,x,y,z,d=DetecteLandmark()
                    #find landmark
                    if(isfindLand==1):
                        if(i==0):
                            Movey(0.05)
                            closeball1(17)
                            closeball2(topdistance,close1_)
                            return runtohitball90(threshold , topdistance ,ditance)
                        if(i<=3):
                            return runtohitball90(threshold , topdistance ,ditance)
                        if(i==4):
                            if(x>=0):
                                return runtohitball90(threshold , topdistance ,ditance)
                            else:
                                return runtohitball90_n(threshold , topdistance ,ditance)
                        if(i==5):
                            if(x>=0):
                                return runtohitball90(threshold , topdistance ,ditance)
                            else:
                                return runtohitball90_n(threshold , topdistance ,ditance)
                        break
                
                    else:
                        pass

                    
                #******************************
                    
                names = ["HeadPitch", "HeadYaw"]
                angleLists = [0.0 * almath.TO_RAD, 0.0 * almath.TO_RAD]
                timeLists = [1.0, 2.0]
                isAbsolute = True
                motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
                time.sleep(1.0)
                
                #*********************************************
                for i in range (0,5):
                    
                    #***********************
                    names = ["HeadPitch", "HeadYaw"]
                    angleLists = [0.0 * almath.TO_RAD, -1*(30.0+i*30.0) * almath.TO_RAD]
                    timeLists = [1.0, 2.0]
                    isAbsolute = True
                    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
                    time.sleep(1.0)

                    #*****************************
                    
                    isfindLand,x,y,z,d=DetecteLandmark()
                    #find landmark
                       #find landmark
                    if(isfindLand==1):
                        if(i<=3):
                            Movey(0.9)
                            Movex(0.45)
                            Movet(-math.pi / 1.5)
                            closeball1(17)
                            closeball2(topdistance,close1_)
                            return runtohitball90(threshold , topdistance ,ditance)
                        if(i==4):
                            Movey(1.0)
                            Movex(0.5)
                            Movet(-math.pi / 1.5)
                            closeball1(17)
                            closeball2(topdistance, close1_)
                            return runtohitball90(threshold , topdistance ,ditance)

                        if(i==5):
                            Movey(1.4)
                            Movex(1.2)
                            Movet(-math.pi / 1.5)
                            closeball1(17)
                            closeball2(topdistance, close1_)
                            return runtohitball90(threshold , topdistance ,ditance)


                        break
                
                    else:
                        pass

        Movey(0.4)
        Movex(0.5)
        break
#***************************************************************************************
def far_ball():
    close1_=23
    topdistance= 30
    Ldistance = 30
    threshold = 20
    while (1):
        isfindLand, size, x = DetecteLandmark_my()
        if (isfindLand == 0):
            count = count + 1
            Movey(0.06)
            redball.lookat(robotIP, PORT)
        elif (((x - Ldistance) < threshold) & ((x - Ldistance) > -threshold)):
            return size
        elif (x - Ldistance < -threshold):
            Movey(-0.02)
            closeball1(17)
            closeball2(topdistance,close1_)
#********************************************************************************** -70~-190
#-60,
def runtohitball90_far(threshold , topdistance ,ditance):
    #turn head
    #detecteland
    #Movex(-0.08)
    adj=0.08
    close1_=23
    count=0
    while(1):
        names = ["HeadPitch", "HeadYaw"]
        angleLists = [0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
        timeLists = [1.0, 2.0]
        isAbsolute = True
        motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        time.sleep(2.0)

        isfindLand, size, x = DetecteLandmark_my()
        print x,threshold+ditance,ditance-threshold
        time.sleep(2.0)
        print  "landmark"
        if(isfindLand==0):
            count=count+1
            Movey(0.12)
            redball.lookat(robotIP, PORT)
            closeball1(17)
            closeball2(topdistance,close1_)

        elif(((x-ditance)>threshold)&((x-ditance)<-threshold)):
            return size, x
        elif((x-ditance)<threshold):

            Movey(-0.03)
            redball.lookat(robotIP, PORT)
            #*************************************
            closeball1(17)
            closeball2(topdistance,close1_)
        else:
            count=count+1
            Movey(0.04)
            redball.lookat(robotIP, PORT)
            closeball1(17)
            closeball2(topdistance,close1_)

#**********************************************************************************
def runtohitball_far( topdistance):
    # ***********************
    close1_=23
    closeball1(17)
    # **************
    closeball2(30,close1_)  # 55


    names = ["HeadPitch", "HeadYaw"]
    angleLists = [0.0 * almath.TO_RAD, 0.0 * almath.TO_RAD]
    timeLists = [1.0, 2.0]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1.0)

    # *****************************
    while (1):
        isfindLand,size,x = DetecteLandmark_my()
        # find landmark
        if (isfindLand == 1):
            if (x < 0):
                # this value need y=0.25+((-x)/y)*(-x),x=0.25
                #***********************************
                names = ["HeadPitch", "HeadYaw"]
                angleLists = [0.0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
                timeLists = [1.0, 2.0]
                isAbsolute = True
                motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
                time.sleep(1.0)
                far_ball()
                #**************************************
                return size
            if (x >= 0):


                #***************
                names = ["HeadPitch", "HeadYaw"]
                angleLists = [0.0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
                timeLists = [1.0, 2.0]
                isAbsolute = True
                motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
                time.sleep(1.0)
                far_ball()
                #********************
                return size
        # if not find ball
        else:
            # first consider the best position
            # ***********************


            # *****************************
            names = ["HeadPitch", "HeadYaw"]
            angleLists = [0.0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
            timeLists = [1.0, 2.0]
            isAbsolute = True
            motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
            time.sleep(1.0)
            return far_ball()





        break
    


def main(robotIP, PORT):

    close1_=23
    #*****************************************
    #motionProxy.rest()
    #*************************
    leftArmEnable = False
    rightArmEnable = False
    motionProxy.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
    #
    # names = ["HeadPitch", "HeadYaw"]
    # angleLists = [0.0 * almath.TO_RAD, 90.0 * almath.TO_RAD]
    # timeLists = [1.0, 2.0]
    # isAbsolute = True
    # motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    # while(1):
    #     print DetecteLandmark_my()


    #get stick
    StandToGetstick()


    #******************************************3******************************************
    # HitBall(0.47)##333
    # Movet(math.pi / 1.9)#3333333333
    # Movex(0.9)#cancel
    # Movet(-math.pi / 4)#333333
    # #
    # #
    # closeball1(23)
    # # # # # # # **************
    # closeball2(30,23)#55
    # Movey(0.8)#33333333333
    # Movex(0.4)#333333333
    # Movet(-math.pi / 1.2)
    # # # # run to hit ball position   wtih 0
    # # #
    # closeball1(23)
    # # # # # **************
    # closeball2(30, 23)
    # size,x= runtohitball90_far(-60 , 30 ,-130)
    # # #x0, y0, z0, dis = runtohitball(0.2, 30, 0.2)#30
    # print "SS",x
    # closeball2(49, 23)
    # # # choose power depending on the distant
    # #
    # #
    # HitBall(0.40)
    # Movet(math.pi / 1.8)
    # Movex(1.6)
    #****************************************2*****************************************

   #  hitball  max=0.27

    #HitBall(0.42) #2 0.45

    #Movet(math.pi / 2)  # 2
    #Movex(0.6)  #2
    #Movey(0.75)#non
    #Movet(-math.pi / 3)#2
    #Movex(0.9)
   #  Movex(0.4)
   #  Movet(math.pi /3)#2
   #  # Movex(0.9)#2
   # #  turn

    while(1):
        #Movet(math.pi/2.8)
        #run to ball
        closeball1(17)
        # #**************
        closeball2(30, 17)
        #run to hit ball position   wtih 0


        x0, y0, z0, dis=runtohitball(0.10, 30, 0.22)

        #closeball2(58, 25)
        #choose power depending on the distant
        if(dis<1.0):

            HitBall(0.45)

        elif (dis <=1.2):
            HitBall(0.42)
        elif (dis >=1.2):
            HitBall(0.42)


        
        
if __name__ == '__main__':
    robotIP = "192.168.1.103"
    IP = robotIP
    PORT = 9559
    # ********************motion init
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    # *******************posture init
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    # ***********************Memory init
    memoryProxy = ALProxy("ALMemory", robotIP, PORT)
    camProxy = ALProxy("ALVideoDevice", IP, PORT)

    # *******************landmark init
    try:
        landMarkProxy = ALProxy("ALLandMarkDetection", IP, PORT)
    except Exception, e:
        print "Error when creating landmark detection proxy:"
        print str(e)
        exit(1)
    try:
        memoryProxy = ALProxy("ALMemory", IP, PORT)
    except Exception, e:
        print "Error when creating memory proxy:"
        print str(e)
        exit(1)
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.1.104",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)    
    
    
