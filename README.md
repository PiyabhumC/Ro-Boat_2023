These are the files that were used in the 1st Agean Ro-Boat Race on a custom made low-cost USV.
They also work on any electric R/C vehicle or any R/C that can be controlled by a PWM receiver.

Requirements:
- Raspberry Pi preferably 4B, also tested on 3B+ and it runs but very slowly.
-- CV2
-- MobileNet with ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt https://gist.github.com/dkurt/54a8e8b51beb3bd3f770b79e56927bd7 and pre-trained model.
  
- Arduino Mega
-- GPS
-- Magnetometer or compass
-- SD card module
  
- R/C transmitter with at least 4 channels.
- - Channel 3 and 4 must be able to use as toggle switch (hold high or low value).

- ESC for trolling motor and steering servo.

How to use:
colavoidance.py runs on Raspberry Pi with pin 17 connected to Arduino pin A22.
Boat_Autopilot_Waypoints_Avoidance.ino is for Arduino (obviously). The pinouts are in the sketch.
Please change the magnetOffsetX,Y,Z accordingly.
