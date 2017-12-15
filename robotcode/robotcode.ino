//This program allows an operator to communicate with the robot via serial to test ESC functionality.
//Set Serial monitor to: BAUD: 9600, with no line endings
//Open serial port with ctrl+shift+m
//Requires Servo library

//SWITCH POSITIONS ON ESC
//1 (far left) ON - mixing is enabled in the ESC. This means that forward and turning are the channels instead of left motors and right motors.
//2 OFF - exponential control off, linear response instead of exponential response (exponential is used for sensitive robots)
//3 ON - for NiMH battery pack
//4 OFF - acceleration ramping is on when the switch is off. This makes the robot accelerate a little slower to prevent extreme current draw. Try both on and off for this switch.
//5 OFF - Auto calibrate off. We don't want the ESC messing with what the microcontroller sends out.
//6 OFF - Turns off calibrate and store mode. Since we do not care about calibration, we also don't care about storing settings.

#include <Servo.h> //required for controlling an ESC

//Constants
const uint8_t DRIVE = 2; //PWM out on pin 2
const uint8_t ECHO = 7; //The echo pin on the HC-SR04 ultrasonic sensor
const uint8_t TRIG = 8; //The trigger pin on the HC-SR04 ultrasonic sensor
const uint16_t MAXDISTANCE = 130; //This makes it so the sensor won't try to resolve distances outside the sensor's range
const uint16_t MINDISTANCE = 0; //This sets the minimum range of the sensor to 0 for obvious reasons
//Variables
uint16_t duration = 0; //Used to calculate distance
uint16_t distance = 130; //cm, Distance to obstacle, starts at 255 because the ultrasonic sensor won't return a value larger than this
uint8_t throttle = 75; //The speed of the robot as understood by the Sabertooth 2x12 RC ESC

Servo SABERTOOTH; //Creating a servo object to represent forward/rearward motion input to the ESC

uint8_t FindDistance() //This function uses the HC-SR04 ultrasonic sensor to find the distance to the wall 
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH); //Finding the time between the
  distance = duration/58.2; //This will truncate the value so that it's an integer

  if (distance >= MAXDISTANCE) //If the distance is out of range or larger than the MAXDISTANCE value, the sensor will just assume it is the MAXDISTANCE
  {
    return MAXDISTANCE;
  }
  if (distance <= MINDISTANCE) //Likewise for the MINDISTANCE
  {
    return 0;
  }
  else
  {
    return distance; //The function returns the distance in cm
  }
}

void setup()
{
  pinMode(TRIG, OUTPUT); //Initializing the sonar control pins
  pinMode(ECHO, OUTPUT);
  
  SABERTOOTH.attach(DRIVE); //the servo object is bound to pin 2 (DRIVE) and SABERTOOTH is used to refer to the ESC from here on out
  SABERTOOTH.writeMicroseconds(1500); //Initializing ESC at zero throttle
}

void loop()
{
  distance = FindDistance();
  //Do some PID shit here
  throttle = (throttle*10)+1000; //Converts the throttle value (0-50-100) to a pulsewidth in microseconds understandable by the MEGA
  SABERTOOTH.writeMicroseconds(throttle); //Sending the command to the ESC
  
}







