// Author: Nick Bonidie
// 12/7/2021
// All code was written by Nick Bonidie-no code was taken from other projects


// define what pins control which servo 
#define rotationServo 5
#define upDownServo 3
#define openCloseServo 7
#define rotateArmServo 4
#define forwardBackwardServo 6
#define switchPin  9


// global variables needed to be initialized 
byte incomingByte;

int up_movement = 45;
int rotate = 45;
int forwardBackward = 90;
const int handOpenDegree = 45; 
const int handCloseDegree = 90; 
bool mode_selected;
int mode;
byte base; byte shoulder; byte elbow;






void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000); // Start the Serial monitor
  mode_selected = false; // Initialize the mode to be not selected yet.
  mode = 10; // Set to an int that isn't one of the criterion
  byte base = 45; byte shoulder = 90; byte elbow = 45;
  
 
  // Initialize operation of the pins needed
  pinMode(A5,INPUT);
  pinMode(A3,INPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  
// Initialize the position of the robot arm
  SG90(rotate,rotationServo);
  SG90(up_movement,upDownServo);
  SG90(forwardBackward,forwardBackwardServo);
  SG90(handOpenDegree,openCloseServo);
  
// run a loop until a mode is selected by the user is Processing
 while (mode_selected == false){
   if (Serial.available() > 0){ // check to see if bytes are being sent over the serial monitor
     incomingByte = Serial.read(); // read what is sent of the serial monitor
     // Check to see if the incoming byte associates with one of the modes selected by the user
      if (incomingByte == 'B'){ mode = 0; mode_selected = !mode_selected; break;}
      if (incomingByte == 'A'){ mode = 1; mode_selected = !mode_selected; break;}
      if (incomingByte == 'V'){ mode = 2; mode_selected = !mode_selected; break;}

   }
  }
  

}

void loop() {

  
  int forceReading = analogRead(3); // Read the voltage from the voltage divider force sensor

  if (forceReading >= 3){digitalWrite(2,HIGH);} // If the claw closes and it comes into contact with something turn the LED on
  if (forceReading < 3){digitalWrite(2,LOW);}  // If the claw doesnt come in contact with anything or is released turn the LED off
 
  //SG90(45,rotationServo);
  //SG90(46,forwardBackwardServo);
  //SG90(66,upDownServo);
  // check to see which mode was selected and run the function for that mode
  if (mode == 0){mode0();}  // if the mode established in the setup is mode 0 run robot based on control panel
  if (mode == 1){mode1();}  // if the mode is 1 then run it based on the drawing
  if (mode == 2){ mode2();} // if the mode is 2 then run it based on the joystick


    

}


// function is called SG90 because the Servos are just metal geared SG90s 
void SG90(int angle, int pin){
  int servo_period = 20000;  // Period of the PWM signal
  int t_low = 500;  // PWM pulse in microseconds that correlates to 0 degrees
  int t_neutral = 1500; // PWM pulse in microseconds that correlates to 90 degrees
  int t_high = 2500; // PWM pulse in microseconds that correlates to 180 degress

  int pulse_t = map(angle,0,180,t_low,t_high); // convert the input angle to a pulse time for PWM
  digitalWrite(pin,HIGH);
  delayMicroseconds(pulse_t);
  digitalWrite(pin,LOW);
  delayMicroseconds(servo_period - pulse_t);
  
}

void mode2(){

    pinMode(A1,INPUT);  // pin to read the voltage from the y position of the nob
    pinMode(A0,INPUT); // pin to read the voltage from the x position of the nob
 
    
    int rightLeft = analogRead(0);  // read the x position of the nob
    int upDown = analogRead(1); // read the y position of the nob
    rightLeft = map(rightLeft,0,1023,0,180);  // map the x position to its corresponding angle
    upDown = map(upDown,0,1023,45,180); // map the y position to its corresponding angle

    // move the servo motors to the specified angles
    SG90(90,forwardBackwardServo);
    SG90(rightLeft,rotationServo);
    SG90(upDown,upDownServo);
  
}

void mode1(){
    // if draw mode is selected run this code 
    if (Serial.available() >= 3){  // 3 bytes of data need to be sent for this code to operate. only continue if 3 bytes are availible in the buffer 

      byte base = Serial.read(); byte elbow = Serial.read(); byte shoulder = Serial.read();  // read the three bytes in buffer in the order that they are sent
      
      SG90(base,rotationServo); // actuate the base
      SG90(elbow,upDownServo); // actuate the elbow servo
      SG90(shoulder,forwardBackwardServo); // actuate the shoulder servo
  
      
    }

    //SG90(base,rotationServo); // actuate the base
    //SG90(elbow,upDownServo); // actuate the elbow servo
    //SG90(shoulder,forwardBackwardServo); // actuate the shoulder servo
      
 }



void mode0(){
  
   if (Serial.available() > 0){  // Mode 0 takes a single byte over the serial monitor so once a byte is sent run this code
     byte incomingByte2 = Serial.read();  // read the byte sent by the serial monitor

    // The info coming over the serial monitor is a Char so a switch/case statement can be used to easily check what the user clicked
    switch(incomingByte2){
      case 'D': if (up_movement <= 180){up_movement = up_movement + 3; SG90(up_movement,upDownServo);} break;  // The case where a U comes across the serial monitor indicating the robot to move up
      case 'U': if(up_movement >=45){up_movement = up_movement - 3; SG90(up_movement,upDownServo);} break;  // The case where D comes across the serial monitor indicating a down movement. 
      case 'R': if(rotate >= 0){rotate = rotate - 5; SG90(rotate,rotationServo);} break;// The case where R comes across the serial monitor indicating rotation to the right
      case 'L': if(rotate <= 180){rotate = rotate + 5; SG90(rotate,rotationServo);}break; // The case where L comes across the serial monitor inidicating rotation to the left
      case 'B': if(forwardBackward <= 100) {forwardBackward = forwardBackward + 5; SG90(forwardBackward,forwardBackwardServo);} break; // Case where the forward command comes over the serial monitor
      case 'F': if(forwardBackward >= 60) {forwardBackward = forwardBackward - 5; SG90(forwardBackward,forwardBackwardServo);} break; // Case where the forward command comes over the serial monitor
      case 'C': SG90(handCloseDegree,openCloseServo); break; // close the claw hand
      case 'O': SG90(handOpenDegree,openCloseServo); break; // open the claw hand
      case 'H': SG90(90,forwardBackwardServo); SG90(45,upDownServo); SG90(45,rotationServo); up_movement = 45; rotate = 45; forwardBackward = 90; break;
      }
     }
    // SG90(rotate,rotationServo);
    // SG90(up_movement,upDownServo);
   //  SG90(forwardBackward,forwardBackwardServo);
  
}


  
