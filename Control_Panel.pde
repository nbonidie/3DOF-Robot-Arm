// Nick Bonidie 

import processing.serial.*; // add the serial library
Serial myPort; // define a serial port object to monitor



// Serial Commands to Send:
// H = send robot home
// U = go up
// D = go down
// L = go left
// R = go right
// F = move shoulder arm forward
// B = move shoulder arm backward

// Variables that hold the mode the user wants to use;
int mode = 10;  // initialize a mode that doesn't equal a loop indicating mode
boolean mode_selected = false; // Initialize mode selected to be false
boolean backgroundONOFF = false; // Let code know to change the background when point control is selected
float pix2mm = 0.2; // the ration for point mode. 0.2 mm/pix
float z = 30;  // the z cooridnate you want the end of the arm to stay at in point control mode
float base; float elbow; float shoulder;  // the relevent angles for inverse kinematics of point control mode
float d1 = 85; float d2 = 135; // length of the two joint arms. base-first servo = 85mm, first servor to end of arm = 150mm



void setup(){
  // Setup serial port connection for coms
  myPort = new Serial(this, Serial.list()[4], 250000); // define input port
  myPort.clear(); // clear the port of any initial junk

  
  size(500,500);  // Set window size to 500 by 500 pixels
  cursor(ARROW); // set the cursor to arrow
  
  // Initilizatoin of the mode selection widow
  fill(0,0,0);
  textSize(50);
  text("Please Select a Mode:",30,100);
  fill(255,255,255);
  rect(100,(height/2)-50,100,100);
  rect(300,(height/2)-50,130,100);
  rect(210,350,100,100);
  textSize(20);
  fill(0,0,0);
  text("Point",110,height/2);
  text("Control Panel",310,height/2);
  text("Joystick",215,400);
  
  
  
}

void draw(){
  
    // if the mouse is pressed and a mode hasn't been selected, this code will run
    if (mousePressed && mode_selected == false){
      // check to see if the point where the mouse is pressed is in point selection mode
      if (mouseX >=100 && mouseX <=200){ if (mouseY >= (height/2)-50 && mouseY <= (height/2)+50){
        println("mode1 selected");
        mode = 1; // set mode to 1 which is the point selection mode
        mode_selected = true; // tell the code that the mode is selected
        myPort.write('A'); // write to the arduino the charactor that coressponds to the point selection mode
        delay(50);
        }
      }
      // check to see if the mouse was clicked inside the control panel box
      if (mouseX >= 300 && mouseX <=430){ if (mouseY >= (height/2)-50 && mouseY <= (height/2)+50){
        println("mode0 selected"); 
        mode = 0; // tell the code that mode 0 has been selected
        mode_selected = true; // tell the code that a mode has been selected
        myPort.write('B'); // write the character corresponding to control panel mode to arduino
        }
      }
      // check to see if the joystick box was selected
      if (mouseX >=210 && mouseX <= 310){ if (mouseY >= 350 && mouseY <= 450) {
        println("mode 2 selected");
        fill(255,0,0); // fill the box red to show user the click has been processed
        rect(210,350,100,100); // redraw the rectangle
        mode = 2; // tell the code that mode 2 has been selected
        mode_selected = true; // tell the code that a mode has been selected
        myPort.write('V'); // send the corresponding character that is associated with joystick mode to arduino
      }
      }
    }


  // Check to see if mode 0 has been selected. If it has then run this code
  if (mode == 0){
  
    home_screen(); // Reset the home screen after every loop
  
    if (mousePressed){   // Check if the mouse is being pressed. 
      // Statement that checks to see if the up and down buttons are pressed.
      upDown();
      // Statement that checks to see if the left or right buttons are being pressed.
      leftRight();
       // Checks to see if the forward or backward buttons have been pressed.
      forwardBackward();
       // Checks to see if the home button has been pressed
      HOME();
      }
     // Robot can be controlled by arrow keys on the computer as well
      keyPressCheck();
  
  }
  
  // check to see if mode 1 is selected. If so, run this code
  if (mode == 1){
    if (backgroundONOFF == false){background(100,100,100);backgroundONOFF = true;} // check is the backround has been changed, if not change it and then set the change variable to on.
    
    // check to see if the mouse has been pressed
    if (mousePressed == true){
      // draw an circle to indicate the point selected by the user
      fill(255,0,0);
      float x = mouseX; // save the x position of the mouse relative to the window
      float y = mouseY; // save the y position of the mouse relative to the window
      ellipse(mouseX,mouseY,10,10); // draw the ellipse
      
      float y_mm = y*pix2mm + 123;  // convert the y position real life. A min of 123 was calculated in my inverse kinematics calculations
      float x_mm = x*pix2mm; // convert the x position to real life position.
      
      
      float D = sqrt((y_mm*y_mm) + (z*z));  // find the length of the vector in the yz plane from the origin to the end point
      
      // I am finding the line of intersection to two circles drawn from the endpoint with a radius of 150 and at the origin with a radius of 85
      // a = slope of the intersection line; b = the y intersept of the intersection line
      // plug that line back into the equation of the circle from the origin, simplify the terms and solve for x with the quadratic formula. There will
      // be 2 intersection points for each point. We want the smaller solution which will tilt the elbow arm down. 
      float a = -.1*y_mm; 
      float b = ((y_mm*y_mm)/20) -543.5;
      float y1 = quadForm((1+(a*a)),2*a*b,((b*b)-7255)); // self written quadForm function that returns the smaller x solution
      
      shoulder = (acos(y1/85.0))*(180/PI); // Calculate the shoulder angle based on the smaller solution to the quad formula and the length of the shoulder arm
      
      base = (atan(y/x))*(180/PI); // Calculate the base angle based on the ratio of x and y coordinates picked
      elbow = acos(((D*D) - (d1*d1) - (d2*d2))/(-2*d1*d2))*(180/PI);  // use the law of cosines to find the elbow angle based on the shoulder angle calculated, the two arm distances, and the D vector
      elbow = 180 - elbow; // subtract the elbow angle from 180 to account for the position of the servo
     
      // print the values to make sure they make sense
      println(a,b,y1);
      println(D,byte(base),byte(elbow),byte(shoulder));
      
      // write the three needed angles to the arduino so it can actuate the servos so the end of the arm goes to the selected point 
      myPort.write(byte(base));
      myPort.write(byte(elbow));
      myPort.write(byte(shoulder));
     
  
   
    }
   
  }
}

// The function that is called for the control panel home screen
void home_screen(){
  
  // make sure that the user isn't pressing a key or the mouse
  if (keyPressed == false && mousePressed == false){
    background(100,100,100); // Reset the background of the window
    
    //Draw the up, down, left, right rectangles for control
    fill(0,255,0);
    rect(10,(height/2)-45,90,90);
    rect(width - 100,(height/2)-45,90,90);
    rect((width/2)-45,10,90,90);
    rect((width/2)-45,height - 100,90,90);
    fill(0,0,0); // Make the text black
    textSize(30);
    text("UP",(width/2)-20,70);
    text("DOWN",(width/2)-40,450);
    text("LEFT",15,(height/2));
    text("RIGHT",410,(height/2));
    
    
    
    // Draw the forward and backward rectangles for control
    fill(0,255,0);
    rect(10,10,140,90);
    rect(390,10,100,90);
    fill(0,0,0);
    text("Foward",395,60);
    text("Backward",12,60);
    
      // Fill the home button orange
    fill(255,100,0);
  
    // Draw the home button and put home text inside
    rect((width/2)-45,(height/2)-45,90,90);
    fill(0,0,0);
    textSize(20);
    text("HOME",(width/2)-25,(height/2));
    
  }
  
  
}

// fucntion used to check if the user is pressing a key and if so sends the associated char to the arduino
void keyPressCheck (){
  
  // check to see if a key is being pressed and it is one of the keys coded by processing
  if (keyPressed == true && key == CODED){
     if (keyCode == UP){myPort.write('U');}
     if (keyCode == DOWN){myPort.write('D');}
     if (keyCode == LEFT){myPort.write('L'); }
     if (keyCode == RIGHT){myPort.write('R');}
   }
   
   // Check to see if a non-coded key is being pressed and send the character to the arduino
  if (keyPressed == true && key == 'h'){myPort.write('H');delay(100);}
  if (keyPressed == true && key == 'c'){myPort.write('C');}
  if (keyPressed == true && key == 'o'){myPort.write('O');}
  if (keyPressed == true && key == 'g'){myPort.write('G');delay(100);}
 
 }

// function that checks if the up or down button is being pressed. Moves the elbow arm up or down
void upDown(){
  
  // Check to see if the up or down buttons are being pressed
    if (mouseX > (width/2) - 45 && mouseX < (width/2) + 45){
      if (mouseY > 10 && mouseY < 100){
        println("In Box, moving up");
        fill(255,0,0);
        rect((width/2)-45,10,90,90);
        myPort.write('U');
        
      }
      if (mouseY > 400 && mouseY < 490){
        println("In Box, moving down");
        fill(255,0,0);
        rect((width/2)-45,height - 100,90,90);
        myPort.write('D');
      }
      }
}

// funtion that makes the robot go to the home position
void HOME(){
  // check to see if the home button is being pressed. If so, run this code
   if (mouseX > (width/2) - 45 && mouseX < (width/2) + 45){
       if (mouseY > (height/2)-45 && mouseY < (height/2) + 45){
        println("home button clicked");
        fill(255,0,0);
        rect((width/2)-45,(height/2)-45,90,90);
        fill(0,0,0);
        textSize(20);
        text("HOME",(width/2)-25,(height/2));
        myPort.write('H');
      }
   }
}

// function that moves the rotation servo
void leftRight(){
  
  // Check to see if the right or left buttons are being pressed
    if (mouseY > (height/2) - 45 && mouseY < (height/2) + 45){
      if (mouseX > 400 && mouseX < 490){
        fill(255,0,0);
        rect(width - 100,(height/2)-45,90,90);
        myPort.write('R');
        delay(10);
      }
      if (mouseX > 10 && mouseX < 100){
        fill(255,0,0);
        rect(10,(height/2)-45,90,90);
        myPort.write('L');
        delay(10);
      }
    }
}

// function that moves the shoulder arm
void forwardBackward(){
  if (mouseX >=10 && mouseX <=150){ if (mouseY >= 10 && mouseY <= 100){ myPort.write('B');}}
  if (mouseX >= 390 && mouseX <= 490){ if (mouseY >= 10 && mouseY <= 100){myPort.write('F');}}
  
}

// Quadratic formula function that returns the smaller solution to the quadratic
float quadForm(float a,float b,float c){
  
  float x1 = (-b - sqrt((b*b) - 4*a*c))/(2*a);
  return x1;
  
  
}
