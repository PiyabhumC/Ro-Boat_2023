These are the files that were used in the 1st Agean Ro-Boat Race on a custom made low-cost USV.
They also work on any electric R/C vehicle or any R/C that can be controlled by a PWM receiver.

Requirements:
- Raspberry Pi preferably 4B, also tested on 3B+ and it runs but very slowly.
- Pi Camera or any USB camera
- CV2
- MobileNet pre-trained model: ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt and frozen_inference_graph.pb from https://github.com/zafarRehan/object_detection_COCO .
- pigpio
  
- Arduino Mega
- GPS
- Magnetometer or compass
- SD card module
  
- R/C transmitter with at least 4 channels.
- Channel 3 and 4 must be able to use as toggle switch (hold high or low value).

- ESC for trolling motor and steering servo.
We use Hobbywing Quicrun WP 880 ESC for the trolling motor with AGFRC A280BVSW for steering.
This servo is extremely fast, exteremly powerful and extremely expensive. It can be substituted with ASMC-05A or 05B to lower the cost. However a waterproofing is required.

In case of using with another type of vehicle, as long as it uses an ESC for propulsion and a servo for steering this code will work.
A parameter tuning for the steering endpoints and max throttle will be required.

How to use:
colavoidance.py runs on Raspberry Pi with pin 17 connected to Arduino pin A22.
Boat_Autopilot_Waypoints_Avoidance.ino is for Arduino (obviously). The pinouts are in the sketch.
Please change the magnetOffsetX,Y,Z accordingly.
