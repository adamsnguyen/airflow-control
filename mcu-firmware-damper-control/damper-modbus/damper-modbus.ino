// Include the Arduino Stepper Library
#include <Stepper.h>

// Number of steps per output rotation
const int stepsPerRevolution = 200;

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 2, 3, 4, 5);
int backLimitSwitch = 12;
int frontLimitSwitch = 11;




void setup()
{
  
  pinMode(backLimitSwitch,INPUT_PULLUP);
  pinMode(frontLimitSwitch,INPUT_PULLUP);
  myStepper.setSpeed(20);
  Serial.begin(9800);
}

void loop() 
{
   
  
  int incomingByte = 0; // for incoming serial data
  int backLimitSwitchStatus = digitalRead(backLimitSwitch);
  int frontLimitSwitchStatus = digitalRead(frontLimitSwitch);
  bool limitNotReached = false;
  int steps = 0;
  int stepped = 0;
  int fine = 10;
  int coarse = 50;
  int stepDirection = 0; //-1 is back, 1 is forward
  int active = false;

//  char message[100];
//  sprintf(message, "p9: %d, p10: %d, p11: %d, p12: %d, p15: %d", digitalRead(9) , digitalRead(10), digitalRead(11), digitalRead(12), digitalRead(15));  

  //Serial.println(message);

  if (Serial.available() > 0) {

    incomingByte = Serial.read();
    Serial.println(incomingByte);

    if(backLimitSwitchStatus == 0)
    {
      Serial.println("back limit switch");
    }

    if(frontLimitSwitchStatus == 0)
    {
      Serial.println("front limit switch");
    }

      if (incomingByte == 1)
      {
          steps = fine;
          stepDirection = 1;
          active = true;
          limitNotReached = backLimitSwitchStatus;
      }
      if (incomingByte == 2)
      {
          steps = fine;
          stepDirection = -1;
          active = true;
          limitNotReached = frontLimitSwitchStatus;
      }
      if (incomingByte == 3)
      {
          steps = coarse;
          stepDirection = 1;
          active = true;
          limitNotReached = backLimitSwitchStatus;
      }
      if (incomingByte == 4)
      {
          steps = coarse;
          stepDirection = -1;
          active = true;
          limitNotReached = frontLimitSwitchStatus;     
      }

      char message[100];
      sprintf(message, "steps: %d, step direction: %d, active: %d, limitNotReached: %d", steps , stepDirection, active, limitNotReached);  

      Serial.println(message);
      
      while(limitNotReached && stepped < steps && active)
      {
        if((digitalRead(backLimitSwitch) == 0 && stepDirection == 1) || (digitalRead(frontLimitSwitch) == 0 && stepDirection ==-1))
        {
          break;
          Serial.println("limit reached");
        }
        
        myStepper.step(1*stepDirection);
        stepped++;
      }

      active = false; 
  }
      
}
