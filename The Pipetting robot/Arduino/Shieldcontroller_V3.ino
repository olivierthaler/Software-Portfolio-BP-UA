#include <SoftwareSerial.h>                         //libraries for the processing of the serial command and to controll the stepper motors
#include <SerialCommand.h>
#include <AccelStepper.h>
#include <CheapStepper.h>


SerialCommand SCmd;                                 // The SerialCommand object

AccelStepper stepperNema[4] = {
  AccelStepper(AccelStepper::DRIVER, 2, 5),
  AccelStepper(AccelStepper::DRIVER, 3, 6),
  AccelStepper(AccelStepper::DRIVER, 4, 7),
  AccelStepper(AccelStepper::DRIVER, 12, 13)  
};

CheapStepper stepperCheap[3]={
  CheapStepper (22,23,24,25),
  CheapStepper (26,27,28,29),
  CheapStepper (40,41,42,43)
  } ; 





int stepperEnablePin = 8;                           //pin to enable stepper drivers on the CNC shield,must be tied low to enable drivers
int uddir = 1;
unsigned long lastMillis;
bool b_move_complete = true;



bool moveClockwise = true;


void setup() {

  pinMode(stepperEnablePin, OUTPUT);
  digitalWrite(stepperEnablePin, LOW);

  for (int i = 0; i <= 3; i++) {                    //set the maximum speed and acceleration for the stepper motors
    stepperNema[i].setMaxSpeed(2500);
    stepperNema[i].setAcceleration(1000);
    stepperNema[i].setSpeed(25);
  }
  for (int i = 0; i <= 2; i++) {                    
    stepperCheap[i].setRpm(6);
  }

  SCmd.addCommand("M", move_stepper);
  SCmd.addCommand("C", move_stepper_continues);
  SCmd.addCommand("V", change_velocity);
  SCmd.addCommand("A", change_acceleration);
  SCmd.addCommand("STOP", stop_all);
  SCmd.addCommand("Info", send_info);
  SCmd.addCommand("Pos", send_position);
  SCmd.addCommand("Ready", check_move_complete);
  SCmd.addDefaultHandler(unrecognized);

  Serial.begin(9600);
  Serial.println("Robotic Arm");  

  
  
}



void loop() {
  SCmd.readSerial();                             //check if there are serial commands, if so, process them
  for (int i = 0; i <= 3; i++) {                    
    stepperNema[i].run();
  }


  for (int i = 0; i <= 2; i++) {                    
    stepperCheap[i].run();
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
  Serial.println("Robot Arm");
}


void send_position() {
  Serial.println("Robot Arm");
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
    Serial.print("Not recognized:");
    Serial.println(step_idx);  return;
    Serial.print("ID ");
    Serial.print(step_idx);
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

  if (step_idx <= 3){
    stepperNema[step_idx].setMaxSpeed(velocity);
    Serial.println(" steps/sec");
    }
  else if (step_idx > 3){
    step_idx = step_idx - 4;
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
    Serial.print("Not recognized:");   
    Serial.println(step_idx);  return;
    Serial.print("ID ");
    Serial.print(step_idx);
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

  if (step_idx <= 3){
    stepperNema[step_idx].setAcceleration(acceleration);
    Serial.print("Acceleration motor ");
    Serial.print(step_idx);
    Serial.print(" changed to ");
    Serial.print(acceleration);
    Serial.println(" steps/sec²");
    }
  else if (step_idx > 3){
    Serial.println("Changing acceleration steppermotor 28BYJ-48 not possible");
  }  

}


void check_move_complete() {

  if (b_move_complete) {
    Serial.println("Ready for next command");
    return;
  }

  bool b_all_done = true;
  for (int i = 0; i <= 3; i++) {
    if (stepperNema[i].distanceToGo() != 0) {
      b_all_done = false;
    }
  }
  for (int i = 0; i <= 2; i++) {
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
  for (int i = 0; i <= 3; i++) {
    stepperNema[i].stop();
  }
  for (int i = 0; i <= 2; i++) {
    stepperCheap[i].stop();
  }
}


void move_stepper() {

Serial.println("New message");

  char *arg;
  int step_idx;
  float distance;
  float gearRatio[7]={
    2.6,
    5.10,
    4.85,
    5.3,
    1,
    1,
    1
    };
  int stepsLeft[3];
  

  arg = SCmd.next();
  if (arg == NULL)  {
    Serial.println("Not recognized: Stepper Number" );
    return;
  }

  step_idx = atoi(arg);
  if (step_idx < 0) {
    Serial.print("Not recognized:");   Serial.println(step_idx);  return;
    Serial.print("ID ");
    Serial.print(step_idx);
  }

  arg = SCmd.next();
  if (arg == NULL)   {
    Serial.println("Not recognized: No height parameter given");
    return;
  }

  distance = atof(arg);
  if (distance == 0) {
    Serial.println("Not recognized: Height parameter not parsed");
    return;
  }

  Serial.print("moving ");
  Serial.print(distance);
  if (step_idx == 0) {
    Serial.println(" mm");
  }
  else {
    Serial.println(" Degrees");
  }
  
  
  if (step_idx <= 3){
    Serial.println(distance*17.77778*gearRatio[step_idx]);     //   360[°] / 1.8[°/step] = 200[step],   200[step] * 32 [microstep/step] = 6400[microstep],  6400[microstep] / 360[°] = 17.77778[microstep/°]
    stepperNema[step_idx].move(distance*17.77778*gearRatio[step_idx]);
    }
  else if (step_idx > 3){
    step_idx = step_idx - 4;

    for (int i = 0; i <= 2; i++) {
      stepsLeft[i] = stepperCheap[i].getStepsLeft();
    }
    
    if (stepsLeft[step_idx] == 0){    // if the motor doesn't move then you can send a new command
        if (distance > 0){
          moveClockwise = true;  
        }
        
        else if(distance < 0){
          moveClockwise = false;
        }
        Serial.println(distance*gearRatio[step_idx]);
        stepperCheap[step_idx].newMoveDegrees(moveClockwise, abs(distance)); 
    }
  }
  
  b_move_complete = false;


}




//////////////////////Dit werkt nog niet, het lijkt me niet mogelijk om een continu beweging te krijgen 
void move_stepper_continues() {
  
  char *arg;
  int step_idx;
  int cont_on;

  arg = SCmd.next();
  if (arg == NULL) {
    Serial.println("Not recognized: CCCCCCCCCCCCC");
    return;
  }

  step_idx = atoi(arg);
  if (step_idx < 0) {
    Serial.print("Not recognized:");   
    Serial.println(step_idx);  return;
    Serial.print("ID ");
    Serial.print(step_idx);
  }

  arg = SCmd.next();
  if (arg == NULL)   {
    Serial.println("Not recognized: No parameter given");
    return;
  }
  
  cont_on = atoi(arg);
  if (cont_on == 0) {
    Serial.println("Not recognized: Acceleration parameter could not get parsed");
    return;
  }

//  while (cont_on ==1){
//    stepperNema[step_idx].runSpeed();
//    }

//  if (cont_on == 1){
//    stepperNema[step_idx].runSpeed();
//    }

  
  }
