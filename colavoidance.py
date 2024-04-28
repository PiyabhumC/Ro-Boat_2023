import cv2
import numpy as np
import os
import time
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1)
import pigpio

servo=17  #Connect the ESC in this GPIO pin 

pi = pigpio.pi();
pi.set_servo_pulsewidth(servo, 0) 

max_steering = 2000
mid_steering = 1500
min_steering = 1000

turning_multiplier = 3.5 # obj_width(w) / half_frame or desired max width(a) * max_steering(500)


#This is to pull the information about what each objct is called
classNames = []
classFile = "/home/uoirobo/Desktop/Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

#This is to pull the information about what each object should look like
#Change the path according to your setup.
configPath = "/home/uoirobo/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/uoirobo/Desktop/Object_Detection_Files/frozen_inference_graph.pb"

#This is some set up values to get good results
net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


def stCmd(obs_x, obs_w ,prev_state):
    if prev_state < 1 and obs_w > 0:
        prev_state += 1
        return 0,0,prev_state
    
    if obs_x < 320:
        st = 'R'
        d = 1
        
    else:
        st = 'L'
        d = -1
        
    p = obs_w*turning_multiplier
    #if p > 60:
    #    p = 60
    
    if p == 0:
        prev_state = 0
        
    pwr = mid_steering + (d*p)
    if pwr > max_steering:
        pwr = max_steering
    if pwr < min_steering:
        pwr = min_steering
    
    print(st,',', pwr,',', prev_state,',',obs_w)
    return st, pwr, prev_state


#This is to set up what the drawn box size/colour is and the font/size/colour of the name tag and confidence label   
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
#Below has been commented out, if you want to print each sighting of an object to the console you can uncomment below     
#print(classIds,bbox)
    #print(classIds)
    max_cls_width = 0
    max_cls_x = 0
    max_cls_y = 0
    #print('Obj1 x pos: ',bbox[0][0])
    #print('Obj1 y pos: ',bbox[0][1])
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects: 
                objectInfo.append([box,className])
                
                if box[2] > max_cls_width:
                    max_cls_width = box[2]
                    max_cls_x = box[0]+(box[2]//2)
                    max_cls_y = box[1]+(box[3]//2)
                    
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.circle(img, (box[0]+(box[2]//2), box[1]+(box[3]//2)), box[2]//2,color=(0,255,255),thickness= 3 ) # Getting the centre cv2.FILLED
                    #print('Centre & Size = ' ,(box[0]+(box[2]//2), box[1]+(box[3]//2)),box[2],box[3] )
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
        #print('Max class pos = ' ,max_cls_x,max_cls_y,max_cls_width)
    return img,objectInfo, max_cls_x, max_cls_width

#Below determines the size of the live feed window that will be displayed on the Raspberry Pi OS
if __name__ == "__main__":

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS,2)
    
    w = 960
    h = 540
    prev_state = 0
    st_cmd = 1500
    print('turn, pwm, status, px')
    
#Below is the never ending loop that determines what will happen when an object is identified.    
    while True:
        success, img = cap.read()
#Below provides a huge amount of controll. the 0.45 number is the threshold number, the 0.2 number is the nms number)
        crop = img[:350 , :]
        result, objectInfo, obs_x, obs_w = getObjects(crop,0.45,0.2, objects=['tv','person','umbrella','boat','airplane'])
        st,p,prev_state = stCmd(obs_x, obs_w, prev_state)
        pi.set_servo_pulsewidth(servo, p)
        cv2.waitKey(1)# & 0xFF == ord('q'):

    cap.release()
    pi.set_servo_pulsewidth(servo, mid_steering)
    time.sleep(1)
    print("Stop the program")
    pi.set_servo_pulsewidth(servo, 0)
    #out.release()
    
    cv2.destroyAllWindows()
