//Multi axis stepping for ECL
//Version: 1.3.3
//Minor improvements from 1.3.1, stable
//General code housekeeping done since 1.3.1
//Relative coordinates are unstable, only works as the first movement command.
//Updated feedrate calculation methods
//LCD screen introduced

//Serial settings:
//BAUD: 57600
//End lines with "newline"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Globals//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Libraries
#include <SPI.h> //Serial peripheral interface for motor controllers
#include <AMIS30543.h> //Command library for ON Semiconductor AMIS-30543 stepper driver
#include <LiquidCrystal_I2C.h> //Library for LCD
#include <Wire.h> //Library referenced by the LCD library

//Motor Control
#define MOSI (51) //Defining the Arduino's communications to the motor controllers as pin 51 (requires a #define!)
const int CLOCK = 52; //The clock regulating communications with the motor controllers is on pin 52 or the ICSP bus.
const int STEP_M1 = 46; //
const int STEP_M2 = 47;
const int SS_M1 = 48; //Pin 48 specifies the chip select for motor 1 on the Y axis.
const int SS_M2 = 49; //pin 49 specifies the chip select for motor 2 on the Z axis.

//Button (if used)
const int BUTTONPIN = 31;  //The pin connected to the start button.
bool buttonState = 1; //The button starts in the "off" position, 1

//Communications
#define BAUD (57600) //Arduino to PC serial communication rate
#define MAX_BUF (64) //Maximum serial communications message length
char buffer[MAX_BUF]; //Storage location of the buffer, held in buffer until a semicolon is recieved
int sofar; //Shows how much of the buffer is being used

//ECL default parameters
const int DEFAULT_FEEDRATE = 1;  //Default units are mm/s
char mode_abs = 1; //Absolute coordinates are the default////////////////////////////////////////////////////////////////Change to boolean? - probably

//Machine operating variables
float pz = 0; //Z position
float py = 0; //Y position
float fr = 1; //Feedrate for humans to understand
long step_delay = 2500; //Feedrate for the machine to understand
#define STEPS_PER_MM (400) //200 steps, which becomes 400 due to half stepping
#define STEPS_PER_IN (STEPS_PER_MM*25.4)
#define MIN_STEP_DELAY (2500) //The fastest the steppers can go without losing steps is with a 2,500 microsecond delay (some safety margin included - delays as low as 2,200 microseconds have appeared to work)
#define MAX_FEEDRATE (2500/MIN_STEP_DELAY) //A 2,500 microsecond delay is a feedrate of rougly 1 mm per second (a value of 400).
#define MIN_FEEDRATE (0.01525) //Calculated figure based on the capabilities of the delayMicrosecondsLong() function
bool units = 1; //1 = millimeters 0 = inches (the default is millimeters)

//Arc drawing
#define ARC_CW (1)
#define ARC_CCW (-1)
#define MM_PER_SEGMENT (1)

//Initializations
AMIS30543 stepper1; //For M1 - Y-axis
AMIS30543 stepper2; //For M2 - Z-axis
LiquidCrystal_I2C lcd(0x27,16,2); //LCD display. Address: 0x27, 16 characters wide, 2 characters high
bool semiStart = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Supporting functions/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Tells motor 1 (Y-axis) to take a step
void step1()//STABLE - DO NOT MODIFY
{
  //The NXT/STEP minimum high pulse width is 2 microseconds.
  digitalWrite(STEP_M1, HIGH);
  delayMicroseconds(3);
  digitalWrite(STEP_M1, LOW);
  delayMicroseconds(3);
}
//Tells motor 2 (Z-axis) to take a step
void step2()//STABLE - DO NOT MODIFTY
{
  //The NXT/STEP minimum high pulse width is 2 microseconds.
  digitalWrite(STEP_M2, HIGH);
  delayMicroseconds(3);
  digitalWrite(STEP_M2, LOW);
  delayMicroseconds(3);
}

//Shows the human information regarding interface with the machine
void info()
{
  Serial.print('\n');
  Serial.print(F("ECM LATHE "));
  Serial.println(F("Commands:"));
  Serial.println(F("G00 - rapid positioning"));
  Serial.println(F("G01 - linear interpolation (feedrates are in units per second)"));
  Serial.println(F("G02 - circular interpolation: clockwise"));
  Serial.println(F("G03 - circular interpolation: counter-clockwise"));
  Serial.println(F("G04 - dwell"));
  Serial.println(F("G20 - programming in inches"));
  Serial.println(F("G21 - programming in millimeters"));
  Serial.println(F("G90 - absolute coordinates")); 
  Serial.println(F("G91 - relative coordinates"));
  Serial.println(F("G92 - set origin to current position"));
  Serial.println(F("M17 - enable and reinitialize steppers"));
  Serial.println(F("M18 - disable steppers"));
  Serial.println(F("M100 - code list (brings up this message)"));
  Serial.println(F("M101 - help"));
  Serial.println(F("M114 - displays position, units, coordinate system, and feedrate"));
  Serial.print('\n');
}

//Shows examples of the user interface
void examples()
{
  Serial.println(F("GENERAL:"));
  Serial.println(F("Spaces are required between every code block."));
  Serial.println(F("All G-codes must be written on a single line."));
  Serial.println(F("All commands must end with a new line. Select this option from the menu at the bottom right corner of serial comms window."));
  Serial.println(F("Semicolons should be used to terminate every line."));
  Serial.println(F("The BAUD should be set to 57,600. If you're reading this in the terminal the BAUD has already been properly set."));
  Serial.print('\n');
  Serial.println(F("G-CODES"));
  Serial.println(F("RAPID POSITIONING COMMAND FORMAT:"));
  Serial.println(F("G00 Y3.2 Z-1.23;"));
  Serial.println(F("Leaving a coordinate undefined may lead to unexpected movement and should never be done i.e.: G00 Y3.2;"));
  Serial.print('\n');
  Serial.println(F("LINEAR INTERPOLATION COMMAND FORMAT:"));
  Serial.println(F("G01 Y-1.22 Z4 F.5;"));
  Serial.print('\n');
  Serial.println(F("CLOCKWISE ARC INTERPOLATION - under construction"));
  Serial.println(F("COUNTER-CLOCKWISE ARE INTERPOLATION - under construction"));
  Serial.print('\n');
  Serial.println(F("DWELL COMMAND FORMAT:"));
  Serial.println(F("G04 P4;"));
  Serial.println(F("The pause units are in seconds, however the user does not have to enter an integer value"));
  Serial.print('\n');
  Serial.println(F("UNITS AND COORDINATES:"));
  Serial.println(F("To change units to inches, enter: G20;"));
  Serial.println(F("To change units to millimeters, enter: G21;"));
  Serial.println(F("To set the coordinate system to absolute, enter:"));
  Serial.println(F("G90;"));
  Serial.println(F("To set the coordinate system to relative, enter:;"));
  Serial.println(F("G91;"));
  Serial.println(F("To make the current position the origin, enter:"));
  Serial.println(F("G92;"));
  Serial.print('\n');
  Serial.println(F("M-CODES"));
  Serial.println(F("To enable and reinitialize the steppers, enter:"));
  Serial.println(F("M17;"));
  Serial.println(F("To disable the stepper motor drivers to save power, enter:"));
  Serial.println(F("M18;"));
  Serial.println(F("To bring up a list of supported G and M codes, enter:"));
  Serial.println(F("M100;"));
  Serial.println(F("To bring up this message, enter:"));
  Serial.println(F("M101;"));
  Serial.println(F("To display position, feedrate, and coordinate system, enter:"));
  Serial.println(F("M114;"));
}

//Pause Function
void pause(long ms)
{
  lcd.clear();
  lcd.print("DWELLING");
  delay(ms); //The user enters a value in seconds which was converted to milliseconds before being passed to this function
  lcd.clear();
  lcd.print("READY");
}

//Step delay function
//delayMicroseconds() only works for values less then 16,383
//This function is not as precise as delayMicroseconds(), but is well within the precision required for motor control
void delayMicrosecondsLong(long del)
{
  if (del<=16383)
  {
    delayMicroseconds(del);
  }
  else if(del>16383 && del<=32766)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
   // Serial.print("MOD 1");
  }
  else if(del>32766 && del<=49149)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if(del>49149 && del<=65532)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if(del>65532 && del<=81915)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if(del>81915 && del<=98298)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if(del>98298 && del<=114681)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if(del>114681 && del<=131064)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if(del>131064 && del<=147447)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if(del>147447 && del<=163830)
  {
    long delmod = del%16383;
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(16383);
    delayMicroseconds(delmod);
  }
  else if (del>163830)
  {
    Serial.println("Fatal movement error: feedrate exceeds software limitations");
    Serial.println("Position data now innacurate");
  }
}

//Feedrate definition
void feedrate(float nfr)
{
  if(fr == nfr) return; //If feedrate is the same as before, skip redefining it - just break from the function

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) //If the selected feedrate is out of the allowable range
  {
    Serial.print(F("Feedrate cannot be lower than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps per second, and cannot be greater than "));
    Serial.print(MAX_FEEDRATE);
    Serial.print(F("steps per second due to step loss."));
    return;
  }
  //Otherwise, redefine the feedrate
  step_delay = 2500L/nfr; //The L makes sure that the resulting number remains a 'Long' type variable
  fr = nfr;
}

//Position function for Bresenham's algorithm
void position(float npy, float npz)
{
  py = npy;
  pz = npz;
}

//Stepping function for line segments using Bresenham's algorithm.
void line(float newy, float newz) //newy and newz are the new Y and Z coordinates extracted from the G-code.  The new coordinates and the current position define a line.
{
  long i;
  long err = 0;

  //Finding the slope of the line - m=z/y (?)
  long dy = newy-py; //newy is the new Y coordinate, py is the current position on the Y axis.
  long dz = newz-pz; //newz is the new Z coordinate, pz is the current position on the Z axis.
  lcd.clear(); //Clears any text that may be on the LCD screen
  lcd.setCursor(0,0); //Sets cursor to the top left corner
  lcd.print("MOVING"); //Shows a status message

  int diry = dy>0?0:1; //The sign of diry and dirz determines direction (0 is Y+. The tool is moving away from the spindle)
  int dirz = dz>0?0:1; //These values are for testing and may need to be flipped.

  dy = abs(dy); //For mathematical purposes we are dealing in absolutes - the quadrant of motion on the cartesian plane has already been defined.
  dz = abs(dz); //Based on the magnitudes of these values, knowing the quadrant, we find the octant to operate on.

  if(dy>dz) //If the line is going mostly in the Y direction (choosing which octant to operate Bresenham's algorithm in)
  {
    err = dy/2; //Scenario where the Y direction is governing Bresenham's algorithm
    for(i=0; i<dy; ++i)
    {
      stepper1.setDirection(diry);
      step1(); //Step function for motor 1 (Y axis) the direction is a function input
      err += dz; //Increments over by the value of dz to represent , then checks to see if this value has exceeded dy
      if(err >= dy) //If over has exceeded dy, we give it a step
      {
        err -= dy;
        stepper2.setDirection(dirz);
        step2(); //Step function for motor 2 (Z axis)
      }
      delayMicrosecondsLong(step_delay); //Does not work for values over 16,393
    }
  }
  else //In the case that the line is going mostly in the Z direction, we have the Z axis stepper governing movement
  {
   err = dz/2;
   for(i=0; i<dz; ++i)
   {
    stepper2.setDirection(dirz);
    step2(); //Step function for motor 2 (Z axis)
    err += dy;
    if(err >= dz)
    {
      err -= dz;
      stepper1.setDirection(diry);
      step1(); 
    }
    delayMicrosecondsLong(step_delay);///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //pause(step_delay); //Minimum recommended step delay: 2500 microseconds
   }
  }
  py = newy;
  pz = newz;
  lcd.clear();
  lcd.print("READY");
}

//Returns an angle from dz/dy
float atan3(float dz, float dy)
{
  float a = atan2(dz,dy);
  if(a<0) a = (PI*2.0)+a;
  return a;
}

//Arc Drawing
//Assumes:
//limits have been checked
//constant radius
//ARCS ARE NOT GREATER THAN 180 DEGREES
//cy/cz is the center of the circle
// y/z is the end position
//direc - ARC_CW or ARC_CCW to control direction of the arc
void arc(float cy, float cz, float y, float z, float direc)
{
  //Finding radius
  float dy = py-cy;
  float dz = pz-cz;
  float radius=sqrt(dy*dy+dz*dz);

  //Find arc angle
  float angle1=atan3(dz,dy);
  float angle2=atan3(z-cz, y-cy);
  float theta=angle2-angle1;

  if(direc>0 && theta<0) angle2 += 2*PI;
  else if(direc<0 && theta>0) angle1 += 2*PI;
  theta=angle2-angle1;

  //Find arclength
  float len = abs(theta)*radius;
  int i;
  float segments = ceil(len*MM_PER_SEGMENT);
  float ny;
  float nz;
  float angle3;
  float scale;

  for(i=0;i<segments;++i)
  {
    scale = ((float)i)/((float)segments);

    angle3 = (theta*scale)+angle1;
    ny = cy+cos(angle3)*radius;
    nz = cz+sin(angle3)*radius;

    //Output the results as a line
    line(ny,nz);
  }
  line(y,z);
}

//Debugging output
void output(const char *code, float val)
{
  Serial.print(code);
  Serial.println(val);
}

//Display position, feedrate, and whether the machine is using absolute coordinates or relative coordinates
void where()
{
  if (units == 1)
  {
    float pyo = py/STEPS_PER_MM;
    float pzo = pz/STEPS_PER_MM;
    
    output("Y Position: ",pyo);
    output("Z Position: ",pzo);
    output("Feedrate: ",fr);
    Serial.println("Feedrates are in mm/s");
  }
  else
  {
    float pyo = py/(STEPS_PER_IN);
    float pzo = pz/(STEPS_PER_IN);
    float fri = fr/25.4;
    output("Y Position:",pyo);
    output("Z Position:",pzo);
    
    output("Feedrate:",fri);
    Serial.println("Feedrates are in in/s");
  }
  Serial.println(mode_abs?"Coordinates are absolute":"Coordinates are relative");
  Serial.println(units?"Units are millimeters":"Units are inches");
}

//Parsenumber - this function looks for key letters in the buffer and finds the number immediately after it.
//The variable "code" is the letter that has information following it - this can be G, M, Y, Z, etc. however values are ASCII number equivalents of the actual character.
//The variable "val" is the value that gets returnend if no
float parsenumber(char code, float val)
{
  char *ptr = buffer;
  while(ptr && *ptr && ptr<buffer+sofar) //Removed >1 from (ptr>1 && *ptr....
  {
    if(*ptr==code)
    {
      return atof(ptr+1); //Converts string to floating point, returns this value and terminates function
    }
    ptr = strchr(ptr,' ')+1; //Converts string to character (added +1 to end of statement)
  }
  return val; //Returns "val" as function output, terminates function
}

//Comms ready function
void ready()
{
  sofar = 0; //Clears the input buffer
  Serial.print(F(">")); //Signal ready to recieve input
}


//Initializing Motor Drivers
void motorInit()
{
  delay(1);
  stepper1.resetSettings();
  stepper2.resetSettings();
  stepper1.setCurrentMilliamps(1200); //Current range 2, see page 26 in AMIS-30543 PDF from ON Semiconductor
  stepper2.setCurrentMilliamps(1200); 
  stepper1.setStepMode(2); //A function input of 2 is half stepping
  stepper2.setStepMode(2); //Caution: changing the step mode will change feedrates.
  stepper1.enableDriver();
  stepper2.enableDriver();
}


//Reads the input buffer looking for any G or M commands - there can be one G or M command per line
void processCommand()
{
  //Processing G-codes
  int cmd = parsenumber('G',-1);
  switch(cmd)
  {
    case 0: //Rapid Linear Move
    {
      float yUnits = 0;
      float zUnits = 0;
      //The line drawing function takes floats
      yUnits = parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py); //The number directly following the Y code is stored as a float (this number has units)
      zUnits = parsenumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz);
      //The dimensioned values are converted to steps for the machine to execute
      if (units == 1)
      {
        float stepsY = yUnits*STEPS_PER_MM;
        float stepsZ = zUnits*STEPS_PER_MM;
        feedrate(1); //1 mm/s
        line(stepsY,stepsZ);
        break;
      }
      else
      {
        float stepsY = yUnits*(STEPS_PER_IN);
        float stepsZ = zUnits*(STEPS_PER_IN);
        feedrate(1); //Feedrates run natively in the software as mm/s for mathematical convenience
        //when using inches, feedrates are converted to mm/s before input to the feedrate() funtion
        //when using inches, feedrates are converted back to in/s for display purposes
        line(stepsY,stepsZ);
        break;
      }
    }
    
    case 1: //For line drawing with a specified feedrate
    {
      float feed = parsenumber('F',fr);
      float yUnits = 0;
      float zUnits = 0;
      //The line drawing function takes floats
      yUnits = parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py); //The number directly following the Y code is stored as a float (this number has units)
      zUnits = parsenumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz);
      
      if (units == 1)
      {
        float stepsY = yUnits*STEPS_PER_MM;
        float stepsZ = zUnits*STEPS_PER_MM;
        feedrate(feed);
        line(stepsY,stepsZ);
        break;
      }
      else
      {
        float stepsY = yUnits*(STEPS_PER_IN);
        float stepsZ = zUnits*(STEPS_PER_IN);
        float feedInches = feed*25.4;
        feedrate(feedInches);
        line(stepsY,stepsZ);
        break;
      }
    }
    
    case 2: //For clockwise arcs
    
    case 3: //For counter-clockwise arcs
    {
      feedrate(parsenumber('F',fr));
      arc(parsenumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
         parsenumber('K',(mode_abs?pz:0)) + (mode_abs?0:pz),
         parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
         parsenumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz),
         (cmd==2) ? -1 : 1);
      break;
    }
    
    case 4: //Dwell command (SECONDS)
      pause(parsenumber('P',0)*1000);
      break;

    case 20: //Set units to inches
      units = 0;
      break;

    case 21: //Set units to mm
      units = 1;
      break;

    case 90: //Absolute coordinates mode
      mode_abs = 1;
      break;

    case 91: //Relative coordinates mode
      mode_abs = 0;
      break;

    case 92: //Set logical position
      position(parsenumber('Y',0),
               parsenumber('Z',0));
      break;
       
    default: break;
  }

  //Processing M-codes
  cmd = parsenumber('M',-1);
  switch(cmd)
  {
    case 17: //Enabling and reinitializing steppers
      motorInit();
      lcd.clear();
      lcd.print("READY");
      break;
      
    case 18: //Disable steppers
      stepper1.disableDriver();
      stepper2.disableDriver();
      lcd.clear();
      lcd.print("LOW POWER MODE");
      break;

    case 100: //Shows information to the operator
      info();
      break;

    case 101: //Shows examples of code to the operator
      examples();
      break;
      
    case 114: //Show position
      where();
      break;
      
    default: //Default case
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Function///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(BAUD); //Open comms
  SPI.begin(); //Tells the Arduino to start talking!
  lcd.init(); //Initialize LCD
  lcd.backlight(); //Turn on backlight
  lcd.clear();
  feedrate(DEFAULT_FEEDRATE); //Setting default feedrate
  position(0,0); //The machine assumes where it starts is the origin of the coordinate plane - be careful!
  pinMode(BUTTONPIN, INPUT); //Initializing the start/stop button's pin as an input type pin
  //Initializing motors - commands will be sent to any motors initialized here
  //The init function writes the slave select pin (function input) to high and sets it as an output pin.
  stepper1.init(SS_M1); //Initializing M1 - Y axis.
  stepper2.init(SS_M2); //Initializing M2 - Z axis
  //Initializing control pins: STEP, DIR
  digitalWrite(STEP_M1, LOW);
  pinMode(STEP_M1, OUTPUT);
  digitalWrite(STEP_M2, LOW);
  pinMode(STEP_M2, OUTPUT);
  //Wait for AMIS-30543 and Pololu driver carrier to power up
  delay(1); //Miliseconds
  //Initialize drivers 1 and 2 to their default settings - clears any past configurations or bit errors.
  stepper1.resetSettings();
  stepper2.resetSettings();
  //Initialize the current limiter.  The NEMA 23 steppers draw 1 A per phase at 8.6 V, ECL systems total operating current is typically 1.7 A max at 9.2 V
  stepper1.setCurrentMilliamps(1200); //Current range 2, see page 26 in AMIS-30543 documentation
  stepper2.setCurrentMilliamps(1200); 
  //Specifying the amount of microsteps in a full step
  stepper1.setStepMode(2); //Half stepping...
  stepper2.setStepMode(2); //Caution: changing the step mode will change feedrates.
  //Enable the motor outputs
  stepper1.enableDriver();
  stepper2.enableDriver();
  ready(); //Displays a '>' character to let the operator know the processor is ready for commands
  info();
  lcd.print("READY");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main function////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{  
  while(Serial.available() > 0) //See if there is a message waiting
  {
    char c = Serial.read(); //If there is a message, read it
    Serial.print(c); //Echo back to operator
    if(sofar<MAX_BUF-1) buffer[sofar++]=c; //Store the message
    if((c == '\n') || (c == '\r'))
    {
      buffer[sofar] = 0;
      Serial.print(F("\r\n"));
      processCommand();
      ready();
    }
  }
}













  
