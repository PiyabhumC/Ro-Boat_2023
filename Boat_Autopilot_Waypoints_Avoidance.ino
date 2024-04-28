/*
Arduino sketch for the low-cost USV project used in 1st Agean Ro-Boat Race. Team: Row-UoI-Boat, University of Ioannina (hence UoI), Deparement of Computer Science and Engineering.
Trnsmitter input:
CH1: Steering (Teleop)
CH2: Throttle (Teleop)
CH3 (toggle high-low): Autopilot Activation
CH4 (toggle high-low): Collision Avoidance Activation

Changelog
24/6/2023
- Added CH5 for input from Pi, object detection as extra steering input via PWM
- Added reset waypoint with channel 4. To reset waypoint, CH3:low + CH4:high
- Reset waypoint also results in testing the collision avoidance test & driving aid
- Added collision avoidance
- Added logging messages: final steering command [st_fin], collision avoidance input [col]

25/6/2023
- Trimmed down to lightweight version
*/

// GPS
#include <TinyGPS++.h>
// IMU - Compass
#include <MPU9250_asukiaaa.h>
// SD Card
#include <SPI.h>
#include <SD.h>
//#include <arduino.h>
#include <math.h>
#include <EnableInterrupt.h>
#include <Servo.h>

float lat,lon;
TinyGPSPlus gps;
int cnt = 0;
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

// Setting up remote drive and autopilot
#define RC_NUM_CHANNELS  5

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4

#define RC_CH1_INPUT  A8
#define RC_CH2_INPUT  A9
#define RC_CH3_INPUT  A10
#define RC_CH4_INPUT  A11
#define RC_CH5_INPUT  A12

float goal_threshold = 2.2;

Servo servo;  // Steering
Servo esc;  // Propulsion

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

const int chipSelect = 53;

// Calibrate the magnet if the location is changed and put the offset values here
float magnetOffsetX = -40;
float magnetOffsetY = -17;
float magnetOffsetZ = 44;

// Insert Waypoint here *********************

//double wp_lat = 39.617901;
//double wp_lon = 20.838997;

# PWM output command for the trolling motor
#define autopilot_max 135
#define autopilot_spd 120
#define autopilot_coast 110

/* 
 *  // Car Waypoint
#define waypoint 4
double wp_lat[waypoint] = {39.617901,39.618061,39.618088,39.617931};
double wp_lon[waypoint] = {20.838997,20.839022,20.839214,20.839224};
*/
// Ioannina Lake 4 Waypoints: final one = 0, first one = 1
#define waypoint 4
double wp_lat[waypoint] = {39.680093, 39.679687, 39.678999, 39.679404};
double wp_lon[waypoint] = {20.843039, 20.842412, 20.843163, 20.843790};

// Ioannina Lake 3 Waypoint, Triangle
/*
#define waypoint 3
double wp_lat[waypoint] = {39.679844,39.679687, 39.678999};
double wp_lon[waypoint] = {20.843562,20.842412, 20.843163};
*/

int done = 0;

int this_state = 0;
int prev_state = 0;

// Autopilot Vehicle Parameters
float st_min = 30;
float st_max = 150;
float th_min = 81;
float th_max = 99;

float st_com_min = -10;
float st_com_max = 10;

int st = 90;
int th = 90;

// PID Controller
float dt = 0.5; // Delta time in second
float v_err_prev = 0.0;
float v_i = 0.0;
float th_err_prev = 0.0;
float th_i = 0.0;
// ******************************************

void setup() {
  // Open serial communications and wait for port to open:
  
  Serial.begin(115200);
  Serial1.begin(9600);
  // Light indicators
  pinMode(2, OUTPUT); //  Waypoint = 0, Ready to start
  pinMode(3, OUTPUT); // GPS Reading
  pinMode(4, OUTPUT); // Compass
  pinMode(5, OUTPUT); // SD Logging
    
  servo.attach(8);  // attaches the servo on pin 8
  esc.attach(9);  // attaches the ESC on pin 9
  
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  while(!Serial);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif
  Serial.println("Setting Up IMU...");
  mySensor.beginMag();

  Serial.println("Adding magnet offset to compass...");
  setMagOffset(&mySensor, magnetOffsetX, magnetOffsetY, magnetOffsetZ);

  Serial.println("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  String dataInit = "loop,date,time,spd,lat,lon,mx,my,mz,mdir,th_des,th_err,dist,st,st_fin,col,waypoint";
  if (dataFile) {
      dataFile.println(dataInit);
      dataFile.close();
      // print to the serial port too:
      Serial.println("Done writing header to file!");
      digitalWrite(5, HIGH);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
      digitalWrite(5, LOW);
    }

}

void loop() {
  uint8_t sensorId;
  int result;
  double lat = 0.0;
  double lon = 0.0;
  double th_des = 0.0;
  double dist = 0.0; 
  float th_err = 0.0;
  int new_st = 90;
  int new_th = 90;

  if (done == 0)
  {
    digitalWrite(2, HIGH);
  }
  else
  {
    digitalWrite(2, LOW);
  }

  rc_read_values();
  if (rc_values[RC_CH3] < 1200)
  {
    this_state = 1;
    st = map(rc_values[RC_CH1], 980, 2040, st_min, st_max);
    th = map(rc_values[RC_CH2], 980, 2040, 40, 140);
    //Serial.println(th);

    // reset the waypoint to origin
    if (rc_values[RC_CH4] > 1800)
    {
      done = 0;
      st = colAvoid(st);
    }

    drive(st,th);

    if (this_state != prev_state)
    {
      prev_state = 1;
    }

  }
  
  while(Serial1.available()){ // check for gps data
    String dataString = String(cnt);
    
    rc_read_values();
    if (rc_values[RC_CH3] > 1800)
      this_state = 2;
    if (this_state != prev_state)
    {
      done = (done+1)%waypoint;
      prev_state = 2;
    }
    
    
    //digitalWrite(2, HIGH);
    if(gps.encode(Serial1.read()))// encode gps data
    { 
      
      
      if (gps.date.isUpdated())
      {
        dataString += "," + String(gps.date.day()) + "." + String(gps.date.month()) + "." + String(gps.date.year());
      }
      else
      {
        dataString += "," + String(gps.date.day()) + "." + String(gps.date.month()) + "." + String(gps.date.year());
      }

      if (gps.time.isUpdated())
      {
        dataString += ", ";
        if (gps.time.hour() < 10) dataString +="0";
        dataString += String(gps.time.hour()+3) + ":";
        if (gps.time.minute() < 10) dataString +="0";
        dataString += String(gps.time.minute()) + ":";
        if (gps.time.second() < 10) dataString +="0";
        dataString += String(gps.time.second()) + ".";
        if (gps.time.centisecond() < 10) dataString +="0";
        dataString += String(gps.time.centisecond());
      }
      else
      {
        // Print invalid otherwise.
        dataString += ", 00:00:00.00";
        //Serial.print(F("INVALID"));
      }

      if (gps.speed.isUpdated())
      {
        dataString += ", " + String(gps.speed.kmph());
      }
      else
      {
        // Print invalid otherwise.
        dataString += ", 0";
      }
       
      if (gps.location.isUpdated())
      {
        digitalWrite(3, HIGH);
        dataString += ", " + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6);

        

      }
      // prints invalid if no information was recieved in regards to location.
      else
      {
        digitalWrite(3, LOW);
        dataString += ", " + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6);
        //Serial.print(F("INVALID"));
      }
      
      //dataString += " , " + String(gps.location.lat(), 6) + " , " + String(gps.location.lng(), 6);


      result = mySensor.readId(&sensorId);
      if (result == 0) {
        //Serial.println("sensorId: " + String(sensorId));
        digitalWrite(4, HIGH);
      } else {
        Serial.println("Cannot read sensorId " + String(result));
        digitalWrite(4, LOW);
      }

      result = mySensor.magUpdate();
      if (result != 0) {
        Serial.println("cannot read mag so call begin again");
        mySensor.beginMag();
        result = mySensor.magUpdate();
      }
      if (result == 0) {
        mX = mySensor.magX();
        mY = mySensor.magY();
        mZ = mySensor.magZ();
        mDirection = mySensor.magHorizDirection();
        //Serial.println("magX: " + String(mX));
        //Serial.println("maxY: " + String(mY));
        //Serial.println("magZ: " + String(mZ));
        //Serial.println("horizontal direction: " + String(mDirection));
        if(mDirection < 0.0)
          mDirection = mDirection + 360; 
        
        dataString += ", " + String(mX) + ", " + String(mY) + ", " + String(mZ) + ", " + String(mDirection);
      } else {
        //Serial.println("Cannod read accel values " + String(result));
        dataString += ", 0, 0, 0, 0";
      }
    
    //Serial.println("at " + String(millis()) + "ms");
    //Serial.println(""); // Add an empty line
    lat = gps.location.lat();
    lon = gps.location.lng();
    
    th_des = getAngle(lat,lon);
    dist = getDistance(lat,lon);
    st = servoCommand(th_des);
    //st = spdFilter(st,new_st);
    th_err = getTHErr(th_des);
    /*
    Serial.print("Angle to waypoint relative to north = ");
    Serial.print(th_des);
    Serial.print("\t Angular error = ");
    Serial.print(th_err);
    Serial.print("\t Distance = ");
    Serial.println(dist);
    Serial.print("Servo = ");
    Serial.println(st);
    */
    dataString += ", " + String(th_des) + ", " + String(th_err) + ", " + String(dist) + ", " + String(st);

    if(dist > goal_threshold)
    {
      th = autopilot_spd;
      //th = spdFilter(th,new_th);
      //spdFilter2(new_st,new_th);
    }
    else
    {
      th = 90;
      dataString += "REACHED WAYPOINT!!!";
      delay(2000);
      done = (done+1)%waypoint;
    }
    
    rc_read_values();
    //print();
    if (rc_values[RC_CH3] > 1800)
    {
      if (rc_values[RC_CH4] > 1800)
      {
        st = colAvoid(st);
      }
      drive(st,th);
    }
    else
    {
      st = map(rc_values[RC_CH1], 980, 2040, st_min, st_max);
      th = map(rc_values[RC_CH2], 980, 2040, 40, 140);
      drive(st,th);
    }
    dataString += ", " + String(st) + ", " + String(rc_values[RC_CH5]) + ", " + String(done);

  // if the file is available, write to it:
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
        // print to the serial port too:
        //Serial.println(dataString);
        digitalWrite(5, HIGH);
      }
      // if the file isn't open, pop up an error:
      else {
        //Serial.println("error opening datalog.txt");
        digitalWrite(5, LOW);
      }
    cnt +=1;
    delay(50);
   }
  }
}


void setMagOffset(MPU9250_asukiaaa* sensor, float magX, float magY, float magZ) {
  sensor->magXOffset = magX;
  sensor->magYOffset = magY;
  sensor->magZOffset = magZ;
}


double getAngle(double lat, double lon)
{
  double lat1_rad = lat*DEG_TO_RAD;
  double lon1_rad = lon*DEG_TO_RAD;
  double lat2_rad = wp_lat[done]*DEG_TO_RAD;
  double lon2_rad = wp_lon[done]*DEG_TO_RAD;
  double th_des = 0.0;

  double delta_lon = lon2_rad - lon1_rad;
  double y = sin(delta_lon) * cos(lat2_rad);
  double x = cos(lat1_rad) * sin(lat2_rad) - (sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon));
  th_des = atan2(y, x);
  th_des = th_des*RAD_TO_DEG;
  
  if (th_des < 0)
        th_des += 360;

  return th_des;
}

double getDistance(double lat, double lon)
{
  double lat1_rad = lat*DEG_TO_RAD;
  double lon1_rad = lon*DEG_TO_RAD;
  double lat2_rad = wp_lat[done]*DEG_TO_RAD;
  double lon2_rad = wp_lon[done]*DEG_TO_RAD;

  double delta_lat = lat2_rad - lat1_rad;
  double delta_lon = lon2_rad - lon1_rad;

    //Haversine formula
  double a = pow(sin(delta_lat / 2),2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(delta_lon / 2),2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double radius_of_earth = 6371.0;  // Radius of the Earth in kilometers
  double distance = radius_of_earth * c * 1000;

  return distance;
}

float getTHErr(double th_des)
{
  float th_err = float(th_des) - mDirection;
  if (abs(th_err) > 180)
  {
    if (th_err < 0)
      th_err = th_err + 360;
    else
      th_err = th_err-360;
  } 
  return th_err;
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}


void drive(int st, int th)
{
  servo.write(st);
  esc.write(th);
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }

int servoCommand(double th_des)
{
  float kt_p = 0.15;
  float kt_i = 0.001;
  float kt_d = 0.003;
  float st_cmd = 0.0;
  float th_i_new = 0.0;
  float th_d = 0.0;
  float th_err = float(th_des) - mDirection;
  int steering = 90;

  if (abs(th_err) > 180)
  {
    if (th_err < 0)
      th_err = th_err + 360;
    else
      th_err = th_err-360;
  } 

  th_i_new = th_i + (th_err*dt);
  th_d = (th_err - th_err_prev)/ dt;

  st_cmd = kt_p * th_err + kt_i * th_i_new; // + kt_d*th_d;

  // Updating error values
  th_err_prev = th_err;
  th_i = th_i_new;

  if (st_cmd > st_com_max)
    st_cmd = st_com_max;
  if (st_cmd < st_com_min)
    st_cmd = st_com_min;

  steering = map(st_cmd,st_com_min,st_com_max,st_min,st_max);

  return steering;
}

int escCommand(float v_des)
{
  float k_p = 0.2;
  float k_i = 3;
  float k_d = 0.01;
  float th_cmd = 0.0;
  float v_i_new = 0.0;
  float v_d = 0.0;
  float v_err = v_des - th;
  int throttle = 90;

  v_i_new = v_i + (v_err* dt); //intrgral error
  v_d = (v_err - v_err_prev)/ dt;

  th_cmd = k_p*v_err + k_i*v_i_new;

  // Updating error values
  v_err_prev = v_err;
  v_i = v_i_new;

  if (th_cmd > 1.0)
    th_cmd = 1.0;
  if (th_cmd < -1.0)
    th_cmd = -1.0;

  throttle = map(th_cmd,-1.0,1.0,th_min,th_max);

  return throttle;
}

int colAvoid(int st)
{
  int new_st = st + map(rc_values[RC_CH5], 1000, 2000, -60, 60);
  
  if (new_st > st_max)
    new_st = st_max;
  if (new_st < st_min)
    new_st = st_min;

  return new_st;

}


//---------- Speed filters (not in use) -------------------- 

int spdFilter(int val, int new_val)
{
  if (val < new_val)
    val += 1;
  else  if (val > new_val)
    val -= 1; 
  else 
    val = new_val;
  return val;
}

void spdFilter2(int new_st, int new_th)
{
  while (st != new_st && th != new_th)
  {
    if (st < new_st)
      st += 1;
    else  if (st > new_st)
      st -= 1; 
    else 
      st = new_st;

    if (th < new_th)
      th += 1;
    else  if (th > new_th)
      th -= 1; 
    else 
      th = new_th;
    drive(st,th);
    delay(15);
  }
}
