#include <AccelStepper.h>
#include <Servo.h>


// SENSOR DEFINITIONS //

int laser_sensor_TOP = 40;
bool topSensorMeasurementTaken;
int topSensorCounter;

int laser_sensor_MIDDLE = 42;
bool middleSensorMeasurementTaken;
int middleSensorCounter;

int laser_sensor_BOTTOM = 44;   
bool bottomSensorMeasurementTaken;
int bottomSensorCounter;
                    
int endstop_sucktion_sensor = 48;   
int endstop_bottom_sensor = 50;    
int endstop_top_sensor = 52;   


// MOTOR DEFINITIONS //

const int stepX = 2;
const int dirX = 5;
const int stepY = 3;
const int dirY = 6;
const int stepZ = 4;
const int dirZ = 7;
const int stepA = 12;
const int dirA = 13;


AccelStepper stepperX_TOP(AccelStepper::DRIVER, stepX, dirX); 
AccelStepper stepperY_MIDDLE(AccelStepper::DRIVER, stepY, dirY);
AccelStepper stepperZ_BOTTOM(AccelStepper::DRIVER, stepZ, dirZ); 
AccelStepper stepperA_SUCKTION(AccelStepper::DRIVER, stepA, dirA); 

Servo servo;

int maxSpeedForXYZ = 300;                                                                            ////////////// ADDED LINE /////////////////
int accelerationForXYZ = 180;
float discHORIZONTALPositionSteps[7] = {-87.5, -175, -262.5, -350, 262.5, 175, 87.5};
float discVERTICALPositionSteps[2] = {-700, -1400};

 
// GLOBALS //

bool check = false;
String terminal;
int machine_state = 1;
int main_state = 1;
int takeCard_state = 1;
int dropCard_state = 1;
int distanceHORIZONTAL;                                                                              ////////////// ADDED LINE /////////////////
float overshootDegrees = 5;                                                                          ////////////// ADDED LINE /////////////////
int laserCounter = 0;                                                                                ////////////// ADDED LINE /////////////////
int lastLaserMeasurement = 1;                                                                        ////////////// ADDED LINE /////////////////

bool timeMeasurementTaken = false;////////////// ADDED LINE /////////////////



bool positionFound = false;
int positionCounter = 0;
String userInput;
String currentPlacement[2] = {"0", "0"};
String pickUpPlacement[2] = {"0", "0"};
String destination[2] = {"0", "0"};
String RESETcurrentPlacement[2] = {"0", "0"};
String pickUpPlacementDepth = "";
bool BOOT = true;
bool destinationCurrent;
bool pickUpPlacementCurrent;


String dataFromRaspberryPi;
const char *seperator = ",";
char *token;
char charBuf[8];

#define handSwitch 5

int pos = 0;
bool cardLifted = false;
bool movementDone = true;
bool messageProtocolOver;
bool computerVisionInitiated = false;
int indexCardsCounter = 0;


 
void setup() {  

  stepperX_TOP.setMaxSpeed(maxSpeedForXYZ);                 // used by run() - motor accelerates up to this value
  stepperX_TOP.setAcceleration(accelerationForXYZ);             
  stepperX_TOP.setCurrentPosition(0);                       // Resets current position of motor to 0 position (also sets motor speed to 0) 

  stepperY_MIDDLE.setMaxSpeed(maxSpeedForXYZ);             // used by run() - motor accelerates up to this value
  stepperY_MIDDLE.setAcceleration(accelerationForXYZ);             
  stepperY_MIDDLE.setCurrentPosition(0);                   // Resets current position of motor to 0 position (also sets motor speed to 0) 

  stepperZ_BOTTOM.setMaxSpeed(maxSpeedForXYZ);             // used by run() - motor accelerates up to this value
  stepperZ_BOTTOM.setAcceleration(accelerationForXYZ);             
  stepperZ_BOTTOM.setCurrentPosition(0);                   // Resets current position of motor to 0 position (also sets motor speed to 0) 

  stepperA_SUCKTION.setMaxSpeed(900);                      // used by run() - motor accelerates up to this value
  stepperA_SUCKTION.setAcceleration(1500);             
  stepperA_SUCKTION.setCurrentPosition(0);                 // Resets current position of motor to 0 position (also sets motor speed to 0) 

  pinMode(laser_sensor_BOTTOM,INPUT);
  pinMode(laser_sensor_MIDDLE, INPUT);
  pinMode(laser_sensor_TOP, INPUT);
  
  pinMode(endstop_bottom_sensor, INPUT);
  pinMode(endstop_top_sensor, INPUT);
  pinMode(endstop_sucktion_sensor, INPUT);

  pinMode(handSwitch, INPUT_PULLUP);
  servo.attach(46);
  servo.write(0);
  
  Serial.begin(115200); 
}

void loop() {
 
  machine_state_machine();
 
}


void machine_state_machine() {

  switch(machine_state) {

  case 1: // INITIALIZE MACHINE
     if (BOOT == true) {
     //Serial.println("MACHINE BOOT: Initializing machine");
     BOOT = false;
     }
     findStartPosition();  
     machine_state = 2;                                                             

     //Serial.println("MACHINE READY: Waiting for movement command");
  break;
  case 2: // AWAIT COMMAND
    
    messageProtocolOver = false;
    SerialCommunicationProtocol();
    
  break;
  case 3: // RUN MACHINE  
  
  move_state_machine();

  Serial.flush();
  Serial.println("DONE");
  
  
  break;
  case 4: // TEST ENVIROMENT
  
     if(digitalRead(laser_sensor_BOTTOM) == 0){
      Serial.println("HEJ");
     }
     
  break;
  }
}


void move_state_machine() {
  switch(main_state)      // STATES: FIND CARD, TAKE CARD, FIND PLACE, PLACE CARD, ERROR
  {
    case 1:          
      //Serial.println("STATE 1: FIND PICKUP");
      
      calculateVERTICALPath(pickUpPlacement);
      motorChangeHORIZONTALPosition(calculateHORIZONTALChangePosition(pickUpPlacement));
   
      for (int i = 0; i < 2; i++) {
        currentPlacement[i] = pickUpPlacement[i];
      }
      
      main_state = 2; 
      
      break;     
    case 2:          
      if (takeCard_state == 1) {
        //Serial.println("STATE 2: TAKE CARD");
        //Serial.println(pickUpPlacementDepth);
      }
      if (pickUpPlacementDepth.toInt() != 0) {
        indexingCards();
      } else {
        takeCard_state_machine();
      }
      main_state = 3;
      break;
    case 3:          
        //Serial.println("STATE 3: FIND PLACEMENT");

        calculateVERTICALPath(destination);
        motorChangeHORIZONTALPosition(calculateHORIZONTALChangePosition(destination));
      
      main_state = 4;
      break;       
    case 4:
      if (computerVisionInitiated == false) {          
        if (dropCard_state == 1) {
          Serial.println("STATE 4: PLACE CARD");
        }
        dropCard_state_machine();
        
      } else {
        //Serial.println("STATE 4: READ CARD");

        Serial.println("ready");
        
        while(1) {
          while (Serial.available() == 0) {}
          dataFromRaspberryPi = Serial.readStringUntil('\n');

          if (dataFromRaspberryPi == "DONE") {
            Serial.println("I AM HERE");
 
            calculateVERTICALPath(pickUpPlacement);
            motorChangeHORIZONTALPosition(calculateHORIZONTALChangePosition(pickUpPlacement));

            dropCard_state_machine();
            
            computerVisionInitiated = false;
            break;
          }
        }    
      }
      
        machine_state = 2;
        main_state = 1;
        
      break;
    case 5:          
      Serial.println("STATE 5: ERROR");
      break;
  }
}

void takeCard_state_machine() {
  
     // MOVE DOWN NO CARD
      //Serial.println("\tTake_Card sub-state 1: MOVE DOWN"); 
      takeCard_moveDownNOCARD();      
      // SUCKTION 
      //Serial.println("\tTake_Card sub-state 2: SUCKTION");   
      takeCard_sucktion(); 
      takeCard_state = 3;  
      // LIFT   
      //Serial.println("\tTake_Card sub-state 3: LIFT"); 
      takeCard_liftWITHCARD();       
}

void dropCard_state_machine() {
    // MOVE DOWN WITH CARD
      //Serial.println("\tDrop_Card sub-state 1: MOVE DOWN WITH CARD"); 
      dropCard_moveDownWITHCARD();
      // NO SUCKTION
      //Serial.println("\tDrop_Card sub-state 2: NO SUCKTION"); 
      dropCard_releaseSucktion();
       // MOVE UP
      //Serial.println("\tDrop_Card sub-state 3: MOVE UP"); 
      dropCard_liftNOCARD();
      movementDone = true;
}

void takeCard_moveDownNOCARD() {
  //Serial.println("\t\tMoves down no Card");
  stepperA_SUCKTION.move(-1000);
  while (stepperA_SUCKTION.currentPosition() != -1000) {
      stepperA_SUCKTION.run();
      if (digitalRead(endstop_bottom_sensor) == LOW) {
            //Serial.println("\t\t\tSENSOR: bottom sensor activated");
            while(1) {}
        }
      else if (digitalRead(endstop_sucktion_sensor) == LOW) {
            //Serial.println("\t\t\tSENSOR: sucktion sensor activated");
            takeCard_state = 2;
            break;      
        } 
      }
      stepperA_SUCKTION.setCurrentPosition(0);
    }

void dropCard_moveDownWITHCARD() {
  //Serial.println("\t\tMoves down with Card");
  stepperA_SUCKTION.move(-500);
  while (stepperA_SUCKTION.currentPosition() != -500) {
      stepperA_SUCKTION.run();
      if (digitalRead(endstop_bottom_sensor) == LOW) {
            //Serial.println("\t\t\tSENSOR: bottom sensor activated");
            while(1) {}
      }
   }
   stepperA_SUCKTION.setCurrentPosition(0);
}


void takeCard_sucktion() {
    //Serial.println("\t\tSucktion activating");
    for (pos = 0; pos <= 230; pos += 1) { 
      // in steps of 1 degree
      servo.write(pos);              
      delay(15);                       
    }
    //Serial.println("\t\tSucktion complete");
}

void dropCard_releaseSucktion() {
  //Serial.println("\t\tRelease sucktion activated");
  for (pos = 230; pos != 0; pos -= 1) { 
      // in steps of 1 degree
      servo.write(pos);             
      delay(15);                        
  }
  //Serial.println("\t\tRelease sucktion complete");
}

void takeCard_liftWITHCARD() {
   //Serial.println("\t\tLift");
   stepperA_SUCKTION.move(1000);
   while (stepperA_SUCKTION.currentPosition() != 1000) {
      stepperA_SUCKTION.run();
      if (digitalRead(endstop_top_sensor) == LOW) {
            check = true;
            break;  
        } 
    }
   stepperA_SUCKTION.setCurrentPosition(0);   
}

void dropCard_liftNOCARD() {
   //Serial.println("\t\tLift");
   stepperA_SUCKTION.move(500);
   while (stepperA_SUCKTION.currentPosition() != 500) {
      stepperA_SUCKTION.run();
      if (digitalRead(endstop_top_sensor) == LOW) {
            check = true;
            break;  
        } 
    }
    stepperA_SUCKTION.setCurrentPosition(0);  
}

int calculateHORIZONTALChangePosition(String inputDestination[2]) {

  int tmpHolder;
  float amountOfStepsHORIZONTAL;

  distanceHORIZONTAL = inputDestination[1].toInt() - currentPlacement[1].toInt();
  /*
  Serial.print("\n\tMOVING CURRENT DISC [");
  Serial.print(inputDestination[0]);
  Serial.println("]");

  Serial.println("\t\tCalculating Movement:");
  Serial.print("\t\tDestination: [");
  Serial.print(inputDestination[0]);
  Serial.print("|");
  Serial.print(inputDestination[1]);
  Serial.print("]");
  Serial.print("\t\tCurrent Placement: [");
  Serial.print(currentPlacement[0]);
  Serial.print("|");
  Serial.print(currentPlacement[1]);
  Serial.println("]");
  Serial.print("\t\tDistance: [");
  Serial.print(distanceHORIZONTAL);
  Serial.println("]");
  */
  if (distanceHORIZONTAL < 0) {
    tmpHolder = -(distanceHORIZONTAL);
  } else {
    tmpHolder = distanceHORIZONTAL;
  }

  amountOfStepsHORIZONTAL = discHORIZONTALPositionSteps[tmpHolder - 1];

  if (distanceHORIZONTAL < 0) {
    amountOfStepsHORIZONTAL = amountOfStepsHORIZONTAL * -1;
  }
  /*
  Serial.print("\t\tCalculating Movement: Returning amount of steps: [");
  Serial.print(amountOfStepsHORIZONTAL);
  Serial.println("]");
  */
  currentPlacement[1] = inputDestination[1];
  
  return amountOfStepsHORIZONTAL;
}



int calculateVERTICALChangePosition(String inputDestination[2]) {
  
  int distanceVERTICAL, amountOfStepsVERTICAL = 0;
  /*
  Serial.println("\n\tCHANGING DISCS");
  Serial.print("\t\tFrom Disc [");
  Serial.print(currentPlacement[0]);
  Serial.print("] to [");
  Serial.print(inputDestination[0]);
  Serial.println("]");
  */

  if(inputDestination[0].toInt() < currentPlacement[0].toInt()) { ///////////////////////////////////// HERE
    distanceVERTICAL = inputDestination[0].toInt() - currentPlacement[0].toInt();
    distanceVERTICAL = -(distanceVERTICAL);

    amountOfStepsVERTICAL = discVERTICALPositionSteps[distanceVERTICAL];
    amountOfStepsVERTICAL = -(amountOfStepsVERTICAL);
  } else {
    distanceVERTICAL = inputDestination[0].toInt() - currentPlacement[0].toInt();
    
    amountOfStepsVERTICAL = discVERTICALPositionSteps[distanceVERTICAL];
  }
  /*
  Serial.print("\t\tCalculating Movement: Returning amount of steps: [");
  Serial.print(amountOfStepsVERTICAL);
  Serial.println("]");

  Serial.println(currentPlacement[0]);
  Serial.println(inputDestination[0]);
  */
  currentPlacement[0] = inputDestination[0];
  
  return amountOfStepsVERTICAL;
}


void motorChangeHORIZONTALPosition(int amountOfSteps) {
  
  switch(currentPlacement[0].toInt()) {

    case 2:
      //Serial.println("\t\t\t- Using StepperMotor BOTTOM [0]");
      
      stepperZ_BOTTOM.move(amountOfSteps);
      bottomSensorMeasurementTaken = true; // Ensure it wont measure current position!
      while (stepperZ_BOTTOM.currentPosition() != amountOfSteps) {
        
        stepperZ_BOTTOM.run();
        /*
        if (digitalRead(laser_sensor_BOTTOM) == 0 && bottomSensorMeasurementTaken == false) {
          bottomSensorCounter++;
          Serial.print("\t\t\tBottomSensor Distance Measurement: ");
          Serial.println(bottomSensorCounter);
          bottomSensorMeasurementTaken = true;
        } else if (digitalRead(laser_sensor_BOTTOM) == 1 && bottomSensorMeasurementTaken == true) {
          bottomSensorMeasurementTaken = false;
        }
        if (bottomSensorCounter == distanceHORIZONTAL) {
          Serial.println("\t\t- BottomSensor: Destination reached, stopping motor [0]");
          bottomSensorCounter = 0;
          break;
        }*/
      }  
      
      stepperZ_BOTTOM.setCurrentPosition(0);          
    
    break;
    case 1:
      //Serial.println("\t\t\t- Using StepperMotor MIDDLE [1]");

      middleSensorMeasurementTaken = true;
      stepperY_MIDDLE.move(amountOfSteps);
      while (stepperY_MIDDLE.currentPosition() != amountOfSteps) {
        
        stepperY_MIDDLE.run();
        /*
        if (digitalRead(laser_sensor_MIDDLE) == 0 && middleSensorMeasurementTaken == false) {
          middleSensorCounter++;
          Serial.print("\t\t\tMiddleSensor Distance Measurement: ");
          Serial.println(middleSensorCounter);
          middleSensorMeasurementTaken = true;
        } else if (digitalRead(laser_sensor_MIDDLE) == 1 && middleSensorMeasurementTaken == true) {
          middleSensorMeasurementTaken = false;
        }
        if (middleSensorCounter == distanceHORIZONTAL) {
          Serial.println("\t\t- MiddleSensor: Destination reached, stopping motor [1]");
          middleSensorCounter = 0;
          break;
        } */
      }  
        
      stepperY_MIDDLE.setCurrentPosition(0); 
    break;
    case 0:
      //Serial.println("\t\t\t- Using StepperMotor TOP [2]");

      topSensorMeasurementTaken = true;
      stepperX_TOP.move(amountOfSteps);
      while (stepperX_TOP.currentPosition() != amountOfSteps) {
        
        stepperX_TOP.run();
       /*
        if (digitalRead(laser_sensor_TOP) == 0 && topSensorMeasurementTaken == false) {
          topSensorCounter++;
          Serial.print("\t\t\tTopSensor Distance Measurement: ");
          Serial.println(topSensorCounter);
          topSensorMeasurementTaken = true;
        } else if (digitalRead(laser_sensor_TOP) == 1 && topSensorMeasurementTaken == true) {
          topSensorMeasurementTaken = false;
        }
        if (topSensorCounter == distanceHORIZONTAL) {
          Serial.println("\t\t- TopSensor: Destination reached, stopping motor [2]");
          topSensorCounter = 0;
          break;
        } */
      }  
        
      stepperX_TOP.setCurrentPosition(0); 
    break;
  }  
}

void motorChangeVERTICALPosition(int amountOfSteps) {
  stepperA_SUCKTION.move(amountOfSteps);
    while (stepperA_SUCKTION.currentPosition() != amountOfSteps) {
      stepperA_SUCKTION.run();
    }
  stepperA_SUCKTION.setCurrentPosition(0);
  
}

void calculateVERTICALPath(String inputDestination[2]) {

    if (inputDestination[0] != currentPlacement[0]) {

       //Serial.println("\n\t-! Initiating Disc Change !-");

       RESETcurrentPlacement[0] = currentPlacement[0];
       
       motorChangeHORIZONTALPosition(calculateHORIZONTALChangePosition(RESETcurrentPlacement));
       
       motorChangeVERTICALPosition(calculateVERTICALChangePosition(inputDestination));

       //Serial.println("\n\t-! Disc Change Complete !-");
      
    }
}

void SerialCommunicationProtocol() {
    
    char *tmpHolderPTR;

    dataFromRaspberryPi = "";
    while (Serial.available() == 0) {}
    dataFromRaspberryPi = Serial.readStringUntil('\n');
    

    if (dataFromRaspberryPi == "readCard") {
      Serial.println(dataFromRaspberryPi);

      while (Serial.available() == 0) {}
      dataFromRaspberryPi = Serial.readStringUntil('\n');

      if (dataFromRaspberryPi == "CONFIRMED") {
        Serial.println("Ready");

        while (Serial.available() == 0) {}
        dataFromRaspberryPi = Serial.readStringUntil('\n');
        dataFromRaspberryPi = "";
        while (Serial.available() == 0) {}
        dataFromRaspberryPi = Serial.readStringUntil('\n');

        dataFromRaspberryPi.toCharArray(charBuf, 8);
        
        pickUpPlacement[0] = charBuf[0];
        pickUpPlacement[1] = charBuf[2];
        pickUpPlacementDepth = charBuf[4];

        Serial.println(dataFromRaspberryPi);  

        while (Serial.available() == 0) {}
        dataFromRaspberryPi = Serial.readStringUntil('\n');
        
        if (dataFromRaspberryPi == "CONFIRMED") { 
      
          destination[0] = "0";
          destination[1] = "0";

          computerVisionInitiated = true;

          messageProtocolOver = true;
          machine_state = 3;
          
        } else {
          //Serial.println("Dropped at data");
        }
      } else {
        //Serial.println("Dropped at command");
      }
    } else if (dataFromRaspberryPi == "moveCardTable") {

            Serial.println(dataFromRaspberryPi);

      while (Serial.available() == 0) {}
      dataFromRaspberryPi = Serial.readStringUntil('\n');

      if (dataFromRaspberryPi == "CONFIRMED") {
        Serial.println("Ready");

        while (Serial.available() == 0) {}
        dataFromRaspberryPi = Serial.readStringUntil('\n');

        dataFromRaspberryPi.toCharArray(charBuf, 8);
        
        pickUpPlacement[0] = charBuf[0];
        pickUpPlacement[1] = charBuf[2];
        destination[0] = charBuf[4];
        destination[1] = charBuf[6];

        Serial.println(dataFromRaspberryPi);

        while (Serial.available() == 0) {}
        dataFromRaspberryPi = Serial.readStringUntil('\n');
        
        if (dataFromRaspberryPi == "CONFIRMED") {

          pickUpPlacementCurrent = true;
          messageProtocolOver = true;
          machine_state = 3;
   
        } else {
          //Serial.println("Dropped at data");
        }
      } else {
        //Serial.println("Dropped at command");
      }
    } 
}

void findStartPosition() {

  stepperA_SUCKTION.move(3500);
  while (digitalRead(endstop_top_sensor) != LOW) {
    stepperA_SUCKTION.run();
  }
  stepperA_SUCKTION.setCurrentPosition(0);
 
} 

void indexingCards() {

    int stepsVar = 87.5;

    if (pickUpPlacement[1] == "1") {
      stepsVar = 175;
    }
    
   while (indexCardsCounter != pickUpPlacementDepth.toInt()) {

      takeCard_state_machine();
      motorChangeHORIZONTALPosition(stepsVar);
      dropCard_state_machine();
      motorChangeHORIZONTALPosition(stepsVar * -1);
      
      indexCardsCounter++;
   }

    takeCard_state_machine();
    motorChangeHORIZONTALPosition(stepsVar * -1);
    dropCard_state_machine();
    motorChangeHORIZONTALPosition(stepsVar);

    while (indexCardsCounter != 0) {
      motorChangeHORIZONTALPosition(stepsVar);
      takeCard_state_machine();
      motorChangeHORIZONTALPosition(stepsVar * -1);
      dropCard_state_machine();
      indexCardsCounter--;
    }

   
   motorChangeHORIZONTALPosition(-(stepsVar));
   takeCard_state_machine();
   motorChangeHORIZONTALPosition(stepsVar);
}
