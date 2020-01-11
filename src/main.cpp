#include <Arduino.h>
#include <PID_v1.h>
#include <RobojaxBTS7960.h>
#include <PID_AutoTune_v0.h>

#define RPWM 9 // define pin 3 for RPWM pin (output)
#define R_EN 3 // define pin 2 for R_EN pin (input)
#define R_IS 11 // define pin 5 for R_IS pin (output)

#define LPWM 6 // define pin 6 for LPWM pin (output)
#define L_EN 4 // define pin 7 for L_EN pin (input)
#define L_IS 12 // define pin 8 for L_IS pin (output)
#define debug 0 //change to 0 to hide serial monitor debugging infornmation or set to 1 to view

// >>> AutoTune Parameters
byte ATuneModeRemember=2;
double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

//set to false to connect to the real world
boolean useSimulation = false;
// <<< AutoTune Parameters



// #define PIN_INPUT 0
// #define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

PID_ATune aTune(&Input, &Output);

//Specify the links and initial tuning parameters
// double Kp=2, Ki=5, Kd=1;
double Kp=2.5, Ki=0.05, Kd=0.01;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
RobojaxBTS7960 motor(R_EN,RPWM,R_IS, L_EN,LPWM,L_IS,debug);

const int MotorInPin = A0;
const int ReferenceInPin = A1;
int direction = 1;

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(MotorInPin);
  Setpoint = analogRead(ReferenceInPin);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(20);
  motor.begin();

    if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;

  Serial.begin(9600);
}

void loop()
{
  // >>> New
  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    Input = analogRead(0);
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      myPID.SetTunings(Kp,Ki,Kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=Output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
     analogWrite(0, Output); 
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
  // <<< New 
  
  // Input = analogRead(PIN_INPUT);
    Input = analogRead(MotorInPin);
    Setpoint = analogRead(ReferenceInPin);
    if (Input <= Setpoint) {
      myPID.SetControllerDirection(DIRECT);
      myPID.Compute();
      if (direction == 0) {
        motor.stop();
        direction = 1;
      }
      motor.rotate(Output, direction);
      Serial.print("Not Here: ");
      Serial.print(direction);
      if (Output == 0) {
        motor.stop();
        delay(10);
      }
      }
    else if (Input > Setpoint) {
      myPID.SetControllerDirection(REVERSE);
      myPID.Compute();
      if (direction == 1) {
        motor.stop();
        direction = 0;
      }
      motor.rotate(Output,direction);
      Serial.print("Here: ");
      Serial.print(direction);
    }
    Serial.print(" Set Point: ");
    Serial.print(Setpoint);
    Serial.print(" Input: ");
    Serial.print(Input);
    Serial.print(" Output: ");
    Serial.println(Output);
    
  // analogWrite(PIN_OUTPUT, Output);
} 


void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    Output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("SetPoint: ");Serial.print(Setpoint); Serial.print(" ");
  Serial.print("Input: ");Serial.print(Input); Serial.print(" ");
  Serial.print("Output: ");Serial.print(Output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("Kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("Ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("Kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  Input = (kpmodel / taup) *(theta[0]-outputStart) + Input*(1-1/taup) + ((float)random(-10,10))/100;

}