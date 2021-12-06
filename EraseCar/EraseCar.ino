can
#include <IRremote.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "SparkFun_MMA8452Q.h"    // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q
#include <IRremote.h>

#define yellow 13
#define red 12
#define green 11


#define enA 6 //Left-Hand Motor (Right Hand Wheel)
#define enB 5 // Right-Hand Motor (Left Hand Wheel)
#define in1 8 //Black
#define in2 7 //Light Grey
#define in3 1 //Dark Grey
#define in4 0 //Purple Wire

#define IRPin 10
#define defaultSpeed 200 // x of 255
#define turnSpeed 200 // x of 255
//Hex definitions for IR remote - the remote sends serial information that gets encoded into hex values. These are accurate and tested with the current remote.
#define POWER 0xFF629D
#define A 0xFF22DD
#define B 0xFF02FD
#define C 0XFFC23D
#define UP 0xFF9867
#define DOWN 0xFF38C7
#define LEFT 0xFF30CF
#define RIGHT 0xFF7A85
#define SELECT 0xFF18E7
IRrecv irrecv(10);
decode_results results;
Adafruit_VL6180X vl = Adafruit_VL6180X(); // Create the TOF object
MMA8452Q accel;                   // create instance of the MMA8452 class

int startNum = 1;
int base = -1;
int loopNum = -1;
#define calibrated 0


int calibrateTOF()
{
  digitalWrite(yellow, HIGH);
  int val1 = vl.readRange();
  delay(50);
  int val2 =  vl.readRange();
  delay(50);
  int val3 =  vl.readRange();
  digitalWrite(yellow, LOW);

  return (val1+val2+val3)/3; // We will call calibrate at the beginning and see what the initial data
  // Roughly looks like. We can change this as needed.

}
/*
double* calibrateAccel()
{
  double ret[3] = {accel.getX(), accel.getY(), accel.getZ()};
  return ret;
}
*/

void stop()
{
  int stopFlag = 1;
  while(stopFlag);
}

void driveStraight()
{
  //digitalWrite(red, 1);
  digitalWrite(in1, 1); // Swap these values if mototres are going wrong direction
  digitalWrite(in2, 0);
  analogWrite(enA, defaultSpeed);
  analogWrite(enB, defaultSpeed);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0); // Swap these values if motors are going wrong direction
  //digitalWrite(red, 0);
}
void driveStep()
{
  int stepDuration = 1000;
  digitalWrite(in1, 1); // Swap these values if mototres are going wrong direction
  digitalWrite(in2, 0);
  analogWrite(enA, defaultSpeed);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0); // Swap these values if motors are going wrong direction
  analogWrite(enB, defaultSpeed);
  
  delay(stepDuration);
  digitalWrite(in1, 0); // Swap these values if mototres are going wrong direction
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}

void turnCCW()
{
  digitalWrite(in1, 0); // Swap these values if mototres are going wrong direction
  digitalWrite(in2, 1);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1); // Swap these values if motors are going wrong direction
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
}

void turnCW()
{
  digitalWrite(in1, 1); // Swap these values if mototres are going wrong direction
  digitalWrite(in2, 0);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0); // Swap these values if motors are going wrong direction
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
}

String readIR(decode_results inp)
{
  if (irrecv.decode())
    {
      long input = inp.value;
      //And now print that value according to the definitions we set at the beginning of the program
      if (input == POWER)
      {
        return "POWER";
      }
      else if (input == A)
      {
        return "A";
      }
      else if (input == B)
      {
        return "B";
      }
      else if (input == C)
      {
        return "C";
      }
      else if (input == UP)
      {
        return "UP";
      }
      else if (input == DOWN)
      {
        return "DOWN";
      }
      else if (input == LEFT)
      {
        return "LEFT";
      }
      else if (input == RIGHT)
      {
        return "RIGHT";
      }
      else if (input == SELECT)
      {
        return "SELECT";
      }
      else {
        return "Bad Signal";
      }
    }
}


void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(7,OUTPUT);
    digitalWrite(7,LOW);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(IRPin, INPUT);
    Wire.begin();
    accel.begin();
    //(accel.begin())?Serial.println("Good"):Serial.println("Bad");
    //Serial.println("TOF Test!!");
    vl.begin();
    IrReceiver.begin(10, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);

    //int base = calibrateTOF();

   
}



void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(green, HIGH);
    bool acc = accel.available();
    uint8_t range = vl.readRange();
    uint8_t stat = vl.readRangeStatus();
    int noise = 5;
    loopNum+=1;
    //long int IRMessage = results.value;

//    Serial.println("starting loop");
//    Serial.println("DEBUG start");
//    Serial.println("Base: " + base);
    //Serial.println("startNum :" + startNum);
    //if(IR)
      Serial.println("IR Input: " + IrReceiver.decodedIRData.command);
    //delay(500);
    //Serial.println(acc);

    if(base = -1)
    {
      base = calibrateTOF();
      Serial.println("Base: " + base);
    }

    if(irrecv.decode())
    {
      if(IrReceiver.decodedIRData.command == 0x46)
        stop(); // Would like better code for a PAUSE vs a hard-stop
      if(IrReceiver.decodedIRData.command == 0x18)
        startNum++;
        Serial.println("added one");
    }

    
    //read accel data for orientation 
        // Turn based off of direction
          //Turn until accel == down
            //Drive forward given amount
              //Turn until accel == direction it WASN'T before
                //Loop
                //startNum % 2 != 1
  if(startNum % 2 == 1)//Main code execution won't start until the "Select" button is pressed. The modulus operator will make it a toggle.
  {
    if(range > base + noise || range < base - noise) //Is an edge detected? This will take some tuning
    {
      if(accel.isRight())
      {
        //Start turning CW
        while(accel.isDown() == false)
        {
          turnCW();
          Serial.println("CP1");//Turning CW after sensing isRight

          if(irrecv.decode())
          {
            if(IrReceiver.decodedIRData.command == 0x18)
            startNum++;
            Serial.println("added one");
            break;
          }
        }
        //Drive step now that we're down
        driveStep();
        Serial.println("stepping");
        while(accel.isLeft() == false)
        {
          turnCW();
          Serial.println("CP2");//Turning CW after taking a step after sensing isRight (Waiting until isLeft)
          if(irrecv.decode())
          {
            if(IrReceiver.decodedIRData.command == 0x18)
            startNum++;
            Serial.println("added one");
            break;
          }
        }

      if(accel.isLeft())
      {
        //Start turning CCW
        while(accel.isDown() == false)
        {
          turnCCW();
          Serial.println("CP3");//Turning CCW after sensing isLeft
          if(irrecv.decode())
          {
            if(IrReceiver.decodedIRData.command == 0x18)
            startNum++;
            Serial.println("added one");
            break;
          }
        }
        //Drive step now that we're down
        driveStep();
        Serial.println("step");
        while(accel.isRight() == false)
        {
          turnCW();
          Serial.println("CP4");//Turning CW after sensing isLeft (Waiting until isRight)
          if(irrecv.decode())
          {
            if(IrReceiver.decodedIRData.command == 0x18)
            startNum++;
            Serial.println("added one");
            break;
          }
        }
      }
    
      }
    // Drive STRAIGHT
      Serial.println("Going straight");
      driveStraight();
   
  }
    irrecv.resume();
    digitalWrite(green, LOW);

}
}
