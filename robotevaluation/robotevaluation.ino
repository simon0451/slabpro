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

const uint8_t DRIVE = 2; //PWM out on pin 2
const uint32_t BAUD = 9600; //communications rate with arduino

Servo SABERTOOTH; //Creating a servo object to represent forward motion input to the ESC

void setup()
{
  SABERTOOTH.attach(DRIVE); //the servo object is bound to pin 2 (DRIVE) and SABERTOOTH is used to refer to the ESC from here on out
  Serial.begin(BAUD); //open up communications for testing purposes
  Serial.println("Enter a motor speed value (0 = back full, 50 = all stop, 100 = ahead full");
  SABERTOOTH.writeMicroseconds(1500); //Initializing ESC at zero throttle

}

void loop()
{
    while (Serial.available())
  {
    uint16_t pulse = Serial.parseInt(); //whatever number the user just typed in is the new motor pulse speed IN PERCENT!
    Serial.print("Current Speed: ");
    Serial.print(pulse);
    Serial.print('\n'); //new line for the next output
    pulse = (pulse*10)+1000; //converting percentage to a pulse width for use with theThunderbird 18 ESC
    SABERTOOTH.writeMicroseconds(pulse); //Sending the command to the ESC - note that this takes about a second for an Arduino Uno to do, a more powerful microcontroller is recommended
  }

}
