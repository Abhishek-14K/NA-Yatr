/*Connections: 
2=tx
3=rx

a4=sda
a5=scl

13=trig
12=echo

9=5v of l298n
11=enA
8=in1
7=in2

10=enB
6=in3
5=in4

4=servo
*/

#include <TinyGPS.h>
#include <HMC5883L.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
int f1=0;
long cm1;
int i=0;
float heading;
TinyGPS gps;
SoftwareSerial ss(3,2);
float x2lon = radians(17.456684), x2lat = radians(78.667074); //enter final location here as lon and lat
HMC5883L comp;
int16_t mx, my, mz;
float head, distance = 0.0;
#define trigPin 13
#define echoPin 12

//Right motor
int enA = 11;
int in1 = 8;
int in2 = 7;

//Left Motor
int enB = 10;
int in3 = 6;
int in4 = 5;

const int danger = 50; //danger distance in cm
int leftDistance, rightDistance;

Servo uservo;
long dur;
long dist;
void setup() {
  // put your setup code here, to run once:
  digitalWrite(9,HIGH);
  uservo.attach(4);
  uservo.write(90);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(enA, OUTPUT);   
  pinMode(enB, OUTPUT);   
  pinMode(in1, OUTPUT);   
  pinMode(in2, OUTPUT);   
  pinMode(in3, OUTPUT);   
  pinMode(in4, OUTPUT);
Wire.begin();
  comp.initialize();
  ss.begin(9600);
  Serial.begin(9600);
  Serial.println("Complete");
  Serial.println("Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
 if (f1 != 0)
    Forward();
  cm1 = ping();
  delay(50);
  cm1 = ping();
  Serial.print("cm:");
  Serial.print(cm1);
  if (cm1>danger){
    comp.getHeading(&mx, &my, &mz);
    heading = atan2(my, mx);
    if (heading < 0)
      heading += 2 * M_PI;
    heading = heading * 180 / M_PI;
    if ((i % 2) == 0)
    { head = heading + 100;
      if (head > 360)
        head = head - 360;
      Left();
      
    }
    else
    { head = heading - 100;
      if (head < 0)
        head = head + 360;
      Right();
    
    }
    while ((heading > head + 8) || (heading < head - 8)) // this loop turns the bot till its facing (head)degrees east of north
    {
      turn();
      delay(5);
      comp.getHeading(&mx, &my, &mz);
      heading = atan2(my, mx);
      if (heading < 0)
        heading += 2 * M_PI;
      heading = heading * 180 / M_PI;
    }
    Stop();
    delay(100);
  }
  else {
  
    Forward();
   
  
    delay(500);
    Stop();

    do {
      gpshead();
    } while (distance == 0.0);
    if (distance < 15)
      while (1)
        Stop();
    if (f1 == 0){ 
      comp.getHeading(&mx, &my, &mz);
      heading = atan2(my, mx);
      if (heading < 0)
        heading += 2 * M_PI;
      heading = heading * 180 / M_PI;


      while ((heading > head + 8) || (heading < head - 8))
      {
        turn();
        delay(5);
        comp.getHeading(&mx, &my, &mz);
        heading = atan2(my, mx);
        if (heading < 0)
          heading += 2 * M_PI;
        heading = heading * 180 / M_PI;
      }
      f1 = 4;
    }
    f1--;
  }
}
  void turn(){ 
  float tur = heading - head;
if (tur < 0.0)
  { if (tur > -180.0)
      Right();
    else
      Left();
  }
  else
  { if (tur < 180.0)
      Left();
    else Right();
  }
}
void gpshead()
{
  bool newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
       Serial.write(c); 
      if (gps.encode(c))
        newData = true;
    }
  }
  if (newData)
  {
    float flat1, flon1;
    unsigned long age;
    gps.f_get_position(&flat1, &flon1, &age);
    flon1 = radians(flon1);  //also must be done in radians
    flat1 = radians(flat1);  //also must be done in radians
    head = atan2(sin(x2lon - flon1) * cos(x2lat), cos(flat1) * sin(x2lat) - sin(flat1) * cos(x2lat) * cos(x2lon - flon1));
    head = head * 180 / 3.1415926535; // convert from radians to degrees
    float dist_calc = 0;
    float diflat = 0;
    float diflon = 0;
    diflat = x2lat - flat1; //notice it must be done in radians
    diflon = (x2lon) - (flon1); //subtract and convert longitudes to radians
    distance = (sin(diflat / 2.0) * sin(diflat / 2.0));
    dist_calc = cos(flat1);
    dist_calc *= cos(x2lat);
    dist_calc *= sin(diflon / 2.0);
    dist_calc *= sin(diflon / 2.0);
    distance += dist_calc;
    distance = (2 * atan2(sqrt(distance), sqrt(1.0 - distance)));
    distance *= 6371000.0; //Converting to meters
    if (head < 0) {
      head += 360; //if the heading is negative then add 360 to make it positive
    }
  }
  else
  {
    head = 0.0;
    distance = 0.0;
  }
}
void movement()
{
  int distanceFwd = ping();
  if (distanceFwd>danger) //if path is clear
  {
    Forward(); //move forward
  }
  else //if path is blocked
  {
    Stop(); 
    uservo.write(0); 
    delay(500);
    rightDistance = ping(); //scan to the right
    delay(500);
    uservo.write(180);
    delay(700);
    leftDistance = ping(); //scan to the left
    delay(500);
    uservo.write(90); //return to center
    delay(100);
    compareDistance();
  }
}
void compareDistance()
{
  if (leftDistance>rightDistance) //if left is less obstructed 
  {
    Left(); //turn left
    delay(500); 
  }
  else if (rightDistance>leftDistance) //if right is less obstructed
  {
    Right(); //turn right
    delay(500);
  }
   else //if they are equally obstructed
  {
    Backward(); //turn 180 degrees
    delay(1000);
  }
}

long ping()
{
 //Trigger the sensor to send out a ping
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 dur = pulseIn(echoPin, HIGH);
 dist = 0.034*dur/2;
 Serial.println(+distance);
 return dist;
}

void Forward() //This function tells the robot to go forward 
{
  //for (int i=0; i <= 50; i++){
  Serial.println("");
  Serial.println("Moving forward");
  // turn on left motor   
  digitalWrite(in1, LOW);   
  digitalWrite(in2, HIGH);   
  // set speed out of possible range 0~255
  analogWrite(enA, 155);   
  // turn on right motor   
  digitalWrite(in3, HIGH);   
  digitalWrite(in4, LOW);   
  // set speed out of possible range 0~255   
  analogWrite(enB, 155);   
  delay(100);
  //}
  //Stop();
}

void Backward() //This function tells the robot to move backward
{
  //for (int i=0; i <= 50; i++){
  Serial.println("");
  Serial.println("Moving backward");
  // turn on left motor   
  digitalWrite(in1, HIGH);   
  digitalWrite(in2, LOW);   
  // set speed out of possible range 0~255
  analogWrite(enA, 155);   
  // turn on right motor   
  digitalWrite(in3, LOW);   
  digitalWrite(in4, HIGH);   
  // set speed out of possible range 0~255   
  analogWrite(enB, 155);   
  delay(100);
  //}
  //Stop();
}

void Left() //This function tells the robot to turn left
{
  //for (int i=0; i <= 50; i++){
  Serial.println("");
  Serial.println("Moving left");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW); 
  //  set speed out of possible range 0~255
  analogWrite(enA, 140); 
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //set speed out of possible range 0~255   
  analogWrite(enB, 155);   
  delay(100);
  //   }
  //   Stop(); 
}

void Right() //This function tells the robot to turn right
{
//   for (int i=0; i <= 50; i++){ 
   Serial.println("");
   Serial.println("Moving right");
   digitalWrite(in1, LOW);
   digitalWrite(in2, HIGH);
//  set speed out of possible range 0~255
   analogWrite(enA, 135); 
   digitalWrite(in3, LOW); //Was High
   digitalWrite(in4, HIGH);
   analogWrite(enB, 155);
   delay(100);
//    }
//     Stop();
}

void Stop() //This function tells the robot to stop moving
{
  Serial.println("");
  Serial.println("Stopping");
// now turn off motors   
  digitalWrite(in1, LOW);   
  digitalWrite(in2, LOW);
  //analogWrite(enA, 0);  
  digitalWrite(in3, LOW);   
  digitalWrite(in4, LOW);
  //analogWrite(enB, 0);
}

  
