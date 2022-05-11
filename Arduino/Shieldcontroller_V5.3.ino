#include <SoftwareSerial.h>                         //libraries for the processing of the serial command and to controll the stepper motors
#include <SerialCommand.h>
#include <AccelStepper.h>
#include <CheapStepper.h>


SerialCommand SCmd;                                 // The SerialCommand object

const int NumOfNemaSteppers = 3;
const int NumOfCheapSteppers = 2;

AccelStepper stepperNema[NumOfNemaSteppers] = {
  AccelStepper(AccelStepper::DRIVER, 2, 5),     // Stepper X
  AccelStepper(AccelStepper::DRIVER, 3, 6),     // Stepper Y
  AccelStepper(AccelStepper::DRIVER, 4, 7)      // Stepper Z
  //AccelStepper(AccelStepper::DRIVER, 12, 13)    // Stepper A
};

CheapStepper stepperCheap[NumOfCheapSteppers] = {
  CheapStepper (22,23,24,25),
  CheapStepper (26,27,28,29)
  //CheapStepper (40,41,42,43)
  } ; 


float gearRatioNema[NumOfNemaSteppers] = {
    2.57,
    1,
    4.3
    };

float gearRatioCheap[NumOfCheapSteppers] = {
  3.24,
  1
  };

int stepperEnablePin = 8;                           //pin to enable stepper drivers on the CNC shield, must be tied low to enable drivers
int uddir = 1;
unsigned long lastMillis;
unsigned long J_lastMillis;
bool b_move_complete = true;

bool moveClockwise = true;  //stepperCheap

const byte limitSwitch_x = 30; //pin for the microswitch using attachInterrupt()
const byte limitSwitch_y = 31; //pin for the microswitch using attachInterrupt()
const byte limitSwitch_z = 32;

int lockx = 0;
int locky = 0;
int lockz = 0;

// variables joystick
int VRx = A14;
int VRy = A15;
int SW = 33;
bool joystick_activated = 0;
int XAVG = 0;
int YAVG = 0;
int mapX = 0;
int mapY = 0;



void setup() {

  pinMode(stepperEnablePin, OUTPUT);
  digitalWrite(stepperEnablePin, LOW);

  for (int i = 0; i <= (NumOfNemaSteppers-1); i++) {                    //set the maximum speed and acceleration for the stepper motors
    stepperNema[i].setMaxSpeed(2500);
    stepperNema[i].setAcceleration(1000);
    stepperNema[i].setSpeed(25);
  }
  for (int i = 0; i <= (NumOfCheapSteppers-1); i++) {                    
    stepperCheap[i].setRpm(2);
  }

  SCmd.addCommand("M_rel", move_stepper_relative);
  SCmd.addCommand("M_abs", move_stepper_absolute);
  SCmd.addCommand("V", change_velocity);
  SCmd.addCommand("A", change_acceleration);
  SCmd.addCommand("J", use_joystick);
  SCmd.addCommand("Home", homing);
  SCmd.addCommand("STOP", stop_all);
  SCmd.addCommand("Info", send_info);
  SCmd.addCommand("Pos", send_position);
  SCmd.addCommand("Ready", check_move_complete);
  SCmd.addDefaultHandler(unrecognized);

  Serial.begin(9600);
  //Serial.println("Robotic Arm"); 

  
  pinMode(limitSwitch_x, INPUT_PULLUP); // internal pullup resistor (debouncing)
  pinMode(limitSwitch_y, INPUT_PULLUP); // internal pullup resistor (debouncing)
  pinMode(limitSwitch_z, INPUT_PULLUP); // internal pullup resistor (debouncing)
  
}



void loop() {
  SCmd.readSerial();                             //check if there are serial commands, if so, process them
  
  limitswitch();
  
  for (int i = 0; i <= (NumOfNemaSteppers-1); i++) {                    
    stepperNema[i].run();
  }

  for (int i = 0; i <= (NumOfCheapSteppers-1); i++) {                    
    stepperCheap[i].run();
  }

  if (millis() - J_lastMillis >= 100UL) {
    
    if (joystick_activated) {
      changejoystick();
      joystick();
      J_lastMillis = millis();  //get ready for the next iteration
    }
  }
  
  if (millis() - lastMillis >= 2 * 60 * 1000UL) {
    
    lastMillis = millis();  //get ready for the next iteration
    check_move_complete();
  }
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
  Serial.println("Not recognized");            //returns not ok to software

}


void send_info() {
  //Serial.println("Robot Arm");
  //Serial.println("Distance Control");
  Serial.println("Robot Zuurkast CORE-lab");
}


void send_position() {
  Serial.println("Motor positions:");
  int M0;
  int M1;
  int M2;
  M0 = (stepperNema[0].currentPosition()) / (17.77778*gearRatioNema[0]);  // currentPosition() returns the current motor position in steps
  M1 = (stepperNema[1].currentPosition()) / (17.77778*gearRatioNema[1]);  // 17.77778 [microsteps/°]
  M2 = (stepperNema[2].currentPosition()) / (17.77778*gearRatioNema[2]);
  Serial.println(String(M0) + " " + String(M1) + " " + String(M2));
}


void change_velocity()    //function called when a serial command is received
{
  char *arg;
  int step_idx;
  float velocity;

  arg = SCmd.next();
  if (arg == NULL) {
    Serial.println("Not recognized: Stepper Number");
    return;
  }

  step_idx = atoi(arg);
  if (step_idx < 0) {
    Serial.print("Not recognized: ");
    Serial.println(step_idx);  
    return;
  }

  arg = SCmd.next();
  if (arg == NULL)   {
    Serial.println("Not recognized: No parameter given");
    return;
  }
  
  velocity = atof(arg);
  if (velocity == 0) {
    Serial.println("Not recognized: Velocity parameter could not get parsed");
    return;
  }

  Serial.print("velocity motor ");
  Serial.print(step_idx);
  Serial.print(" changed to ");
  Serial.print(velocity);

  if (step_idx <= (NumOfNemaSteppers-1)){
    stepperNema[step_idx].setMaxSpeed(velocity);
    Serial.println(" steps/sec");
    }
  else if (step_idx > (NumOfNemaSteppers-1)){
    step_idx = step_idx - NumOfNemaSteppers;
    stepperCheap[step_idx].setRpm(velocity);
    Serial.println(" RPM");
  }  

}


void change_acceleration()    //function called when a serial command is received
{
  char *arg;
  int step_idx;
  float acceleration;

  arg = SCmd.next();
  if (arg == NULL) {
    Serial.println("Not recognized: Stepper Number");
    return;
  }

  step_idx = atoi(arg);
  if (step_idx < 0) {
    Serial.print("Not recognized: ");   
    Serial.println(step_idx);
    return;
  }

  arg = SCmd.next();
  if (arg == NULL)   {
    Serial.println("Not recognized: No parameter given");
    return;
  }
  
  acceleration = atof(arg);
  if (acceleration == 0) {
    Serial.println("Not recognized: Acceleration parameter could not get parsed");
    return;
  }

  if (step_idx <= (NumOfNemaSteppers-1)){
    stepperNema[step_idx].setAcceleration(acceleration);
    Serial.print("Acceleration motor ");
    Serial.print(step_idx);
    Serial.print(" changed to ");
    Serial.print(acceleration);
    Serial.println(" steps/sec²");
    }
  else if (step_idx > (NumOfNemaSteppers-1)){
    Serial.println("Changing acceleration steppermotor 28BYJ-48 not possible");
  }  

}


void check_move_complete() {

  if (b_move_complete) {
    Serial.println("Ready for next command");
    return;
  }

  bool b_all_done = true;
  for (int i = 0; i <= (NumOfNemaSteppers-1); i++) {
    if (stepperNema[i].distanceToGo() != 0) {
      b_all_done = false;
    }
  }
  for (int i = 0; i <= (NumOfCheapSteppers-1); i++) {
    if (stepperCheap[i].getStepsLeft() != 0) {
      b_all_done = false;
    }
  }

  if (b_all_done) {
    Serial.println("Ready for next command");
    b_move_complete = true;
  }
  else {
    Serial.println("Busy");
  }

}

void stop_all() {
  for (int i = 0; i <= (NumOfNemaSteppers-1); i++) {
    stepperNema[i].stop();
  }
  for (int i = 0; i <= (NumOfCheapSteppers-1); i++) {
    stepperCheap[i].stop();
  }
}


void homing(){
  //for (int i = 0; i <= (NumOfNemaSteppers-1); i++) {
    int clamp_distance = 60; //[mm]
    int GripperMaxOpen = 45; //[mm]
    // stepperCheap[0].moveDegrees(true, clamp_distance*gearRatioCheap[0]);  // moveDegrees = blocking move, newMoveDegrees = non-blocking move

    // stepperCheap[0].newMoveDegrees(false, GripperMaxOpen*gearRatioCheap[0]);
    stepperNema[0].setMaxSpeed(800);
    stepperNema[0].move(-100000);
    stepperNema[2].setMaxSpeed(800);
    stepperNema[2].move(-100000);
  //}
  }

void stop_spec(int step_idx) {
  if (step_idx <= (NumOfNemaSteppers-1)){
    stepperNema[step_idx].stop();
  }
  else if (step_idx > (NumOfNemaSteppers-1)){
    step_idx = step_idx - NumOfNemaSteppers;
    stepperCheap[step_idx].stop();
  }
}

void limitswitch(){
  
  if (digitalRead(limitSwitch_x) == 0 && lockx==0) {
    stepperNema[0].setCurrentPosition(0);
    stop_spec(0);
    stepperNema[0].setMaxSpeed(2500); // setting the standard speed after homing
    lockx = lockx+1;    
    }
  if (digitalRead(limitSwitch_y) == 0 && locky==0) {
    stepperNema[1].setCurrentPosition(0);
    stop_spec(1);
    stepperNema[1].setMaxSpeed(2500);
    locky = locky+1;
    }
  if (digitalRead(limitSwitch_z) == 0 && lockz==0) {
    stepperNema[2].setCurrentPosition(0);
    stop_spec(2);
    stepperNema[2].setMaxSpeed(2500);
    lockz = lockz+1;
    }
   
  
  if (digitalRead(limitSwitch_x) == 1 && lockx==1) {
    stepperNema[0].setCurrentPosition(0);
    stop_spec(0);
    stepperNema[0].setMaxSpeed(2500);
    lockx = lockx-1;    
    }
  if (digitalRead(limitSwitch_y) == 1 && locky==1) {
    stepperNema[1].setCurrentPosition(0);
    stop_spec(1);
    stepperNema[1].setMaxSpeed(2500);
    locky = locky-1;
    }
  if (digitalRead(limitSwitch_z) == 1 && lockz==1) {
    stepperNema[2].setCurrentPosition(0);
    stop_spec(2);
    stepperNema[2].setMaxSpeed(2500);
    lockz = lockz-1;
    }
   
}

void use_joystick() {
  char *arg;
  
  arg = SCmd.next();
  if (arg == NULL)  {
    Serial.println("Not recognized: on(1)/off(0) state" );
    return;
  }

  joystick_activated = atoi(arg) != 0;
  if (joystick_activated) {
    Serial.println("joystick activated");
  }
  else if (not joystick_activated) {
    Serial.println("joystick deactivated");
  }
}

void changejoystick(){
  
  int xValue = analogRead(VRx);
  int yRead = analogRead(VRy);
  int yValue = -yRead;
  int SW_state = digitalRead(SW);
  mapX = map(xValue, 0, 1023, -512, 512);
  mapY = map(yValue, 0, -1023, 512, -512);
  }

void joystick(){
  //Serial.println(mapX);
  //Serial.println(mapY);
  int currentx = stepperNema[0].currentPosition();
  int currenty = stepperNema[2].currentPosition();
  int SW_state = digitalRead(SW);
    if (abs(mapX)>100) {
      if (mapX > 0){stepperNema[0].moveTo(150+currentx);}
      else {stepperNema[0].moveTo(-150+currentx);}
      }
    
    if (abs(mapY)>100){
      //Serial.println("yes");
      if (mapY > 0){stepperNema[2].moveTo(150+currenty);}
      else {stepperNema[2].moveTo(-150+currenty);}
      }
}


void move_stepper_relative() {
  move_stepper(true);
}

void move_stepper_absolute() {
  move_stepper(false);
}

void move_stepper(bool MoveRelative) {
  Serial.println("New message");

  char *arg;
  int step_idx;
  float distance;
  int stepsLeft[3];
  

  arg = SCmd.next();
  if (arg == NULL)  {
    Serial.println("Not recognized: Stepper Number" );
    return;
  }

  step_idx = atoi(arg);
  if (step_idx < 0) {
    Serial.print("Not recognized: ");
    Serial.println(step_idx);
    return;
  }

  arg = SCmd.next();
  if (arg == NULL)   {
    Serial.println("Not recognized: No distance parameter given");
    return;
  }

  distance = atof(arg);
  /*
  if (distance == 0) {
    Serial.println("Not recognized: Distance parameter could not get parsed");
    return;
  }
  */

  Serial.print("moving ");
  Serial.print(distance);
  Serial.println(" mm");
  /*
  if (step_idx == 0) {
    Serial.println(" mm");
  }
  else {
    Serial.println(" Degrees");
  }
  */
  
  if (step_idx <= (NumOfNemaSteppers-1)){
    Serial.println(distance*17.77778*gearRatioNema[step_idx]);     //   360[°] / 1.8[°/step] = 200[step],   200[step] * 32 [microstep/step] = 6400[microstep],  6400[microstep] / 360[°] = 17.77778[microstep/°]
    if (MoveRelative == true) {
      stepperNema[step_idx].move(distance*17.77778*gearRatioNema[step_idx]);
    }
    else {    // absolute movement
      stepperNema[step_idx].moveTo(distance*17.77778*gearRatioNema[step_idx]);
    }
  }
  else if (step_idx > (NumOfNemaSteppers-1)){
    step_idx = step_idx - NumOfNemaSteppers;

    for (int i = 0; i <= (NumOfCheapSteppers-1); i++) {
      stepsLeft[i] = stepperCheap[i].getStepsLeft();
    }
    
    if (stepsLeft[step_idx] == 0){    // if the motor doesn't move then you can send a new command
        if (distance > 0){
          moveClockwise = true;  
        }
        
        else if(distance < 0){
          moveClockwise = false;
        }
        
        Serial.println(distance*gearRatioCheap[step_idx]);
        if (MoveRelative == true) {
          stepperCheap[step_idx].newMoveDegrees(moveClockwise, abs(distance*gearRatioCheap[step_idx]));
        }
        else {    // absolute movement
          stepperCheap[step_idx].newMoveToDegree(moveClockwise, abs(distance*gearRatioCheap[step_idx]));
        }
         
    }
  }
  b_move_complete = false;
}
