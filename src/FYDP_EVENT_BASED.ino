#include <LiquidCrystal_I2C.h>
#include <Wire.h> 

LiquidCrystal_I2C lcd(0x27, 20, 4);

// constants won't change. They're used here to set pin numbers:

//Axis definitions
const int X_AXIS = 0;
const int Y_AXIS = 1;
const int Z_AXIS = 2;

//SETTINGS BEGIN---------------------------------------------
const int JOG_AXIS = X_AXIS;

//Factor to convert from steps to mm for each axis
const float AXIS_STEPS_TO_MM[3] = {0.01,0.01,0.01};

//Make the variable true to flip an axis.
const bool AXIS_DIRECTIONS[3] = {true, true, false};

//Axis speed limits in mm/s
const float AXIS_SPEED_LIMIT[3] = {7.0,7.0,7.0};

//axes software limits convention: [Axis][0] = Plus limit for Axis; [Axis][1] = Minus limit for Axis. 
const float AXIS_SOFTWARE_LIMITS[][2] = {{100000,100000},{4000,10000},{100000,100000}}; 

//Default Speeds for move and jog
float move_axis_speed = 4.0;            
float jog_speed = 2.0; 

//SETTINGS END-----------------------------------------------


//PINS CONFIG BEGIN------------------------------------------

//Motor pulse pins
const int MOTOR_PULSE_PIN[3] = {22,26,34};

//Motor Direction pins
const int MOTOR_DIR_PIN[3] = {24,28,36};

//LCD button pins
const int LCD_BUTTON_PIN[4] = {7,6,5,4};

//Limit switches pins
int LIMIT_PIN[][2] = {{48,46},{44,42},{40,38}};

//Interlock pin
const int INTERLOCK_PIN = 3;

//PINS CONFIG END------------------------------------------------


//State variables
bool motors_to_run[3] = {false,false,false};
bool dir_to_run[3] = {false,false,false};

int lcd_button_state[4] = {0,0,0,0};
bool button_state_pressed[4] = {false,false,false,false};
bool button_state_cont[4] = {false,false,false,false};

int main_state = 0;
int screenState = 1;
bool abs_meas = true;         //variable that determines whether in ABS or REF measurement mode. True for ABS and false for REF
bool jog_state = false; 

//State variables
const int DETECT_DISPLAY_BUTTONS = 100;
const int MOVE_PLUS_CHECK_FOR_TERMINATION = 101;
const int MOVE_MINUS_CHECK_FOR_TERMINATION = 102;
const int JOG_CHECK_FOR_TERMINATION = 103;
const int CAL_CHECK_FOR_TERMINATION = 104;

bool interlock_on = true;
int moveAxisSelected = 0;     //variable that determines which axis to move in move screen (0:X, 1:Y, 2:Z)
int temperature =  23;
int humidity = 93;

int ref_pos[3] = {0,0,0};
int abs_pos[3] = {0,0,0};


//shared variables
bool axis_to_move[3] = {false,false,false};
bool dir_to_move[3] = {true,true,true};

//variables for jog
bool stop_now = false;
bool already_pressed = false;

//variables for cal
bool calibrated = false;        //Global variable indicating whether machine is calibrated
bool calibrating = false;
bool update_motion = false;

int axis_calibrated = 0;        //Variable used in calibrate machine function
int limit_state[3] = {LOW,LOW,LOW};  //Variable used in calibrate machine function to store limit switch states

void setup() {

  main_state = DETECT_DISPLAY_BUTTONS;
  
  //DEBUG
  Serial.begin(9600);
  Serial.print("Hello");

  for(int i = 0;i < 4;i++){
    pinMode(LCD_BUTTON_PIN[i], INPUT_PULLUP);
  }

  //Motor pins and limit switches pin
  for(int i = 0;i < 3;i++){
    pinMode(MOTOR_PULSE_PIN[i],OUTPUT);
    pinMode(MOTOR_DIR_PIN[i],OUTPUT);
    pinMode(LIMIT_PIN[i][0],INPUT_PULLUP);
    pinMode(LIMIT_PIN[i][1],INPUT_PULLUP);
  }

  //Interlock pin attach and interrupth
  pinMode(INTERLOCK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERLOCK_PIN), interlockTriggered, CHANGE);
  
  //Initialize LCD
  lcd.begin();
  lcd.noCursor();
  displayScreen_1();

  //Flip limit switch direction if needed
  for(int i = 0; i < 3; i++){
    if(AXIS_DIRECTIONS[i] == true){
      //need to flip
      flipArray(LIMIT_PIN,i);
    }
  }
}

void loop() {

  if(interlock_on == true){
    Serial.println("Door open");
    displayScreen_message("Door open");
    
    //turn off stepper motors
    step_off();

    //wait till door is closed
    while(digitalRead(INTERLOCK_PIN) == LOW){}

    interlock_on = false;
    main_state = DETECT_DISPLAY_BUTTONS;

    //Go back to the screen that was displayed before message
    displayScreenBeforeMessage();
    
  }else{
    switch(main_state){
      case DETECT_DISPLAY_BUTTONS:

        //Detect which display button is pressed and delegate
        for(int i=0; i < 4; i++){
          lcd_button_state[i] = digitalRead(LCD_BUTTON_PIN[i]);
        }
      
        for(int i=0; i < 4; i++){
          if(lcd_button_state[i] == LOW && button_state_pressed[i] == false){
            button_state_pressed[i] = true;  
          }else if((lcd_button_state[i] == HIGH || button_state_cont[i] == true) && button_state_pressed[i] == true){
            button_state_pressed[i] = false;
            Serial.println("Button " + String(i) + " pressed");
            switch(i){
              case 0:
                buttonPressed_1();
                break;
              case 1:
                buttonPressed_2();
                break;
              case 2:
                buttonPressed_3();
                break;
              case 3:
                buttonPressed_4();
                break;
            }
          }
        }
        
      break;
      
      case MOVE_PLUS_CHECK_FOR_TERMINATION:
        //Check for termination: limit switch hit or button released or y axis software limit not crossed
        if(digitalRead(LCD_BUTTON_PIN[2]) == LOW && digitalRead(LIMIT_PIN[moveAxisSelected][0]) == HIGH && abs_pos[moveAxisSelected] < AXIS_SOFTWARE_LIMITS[moveAxisSelected][0]){
          //Can keep moving and update display
          
          //update display
          lcd.setCursor(6,1);
          if(abs_meas){
            lcd.print(String(abs_pos[moveAxisSelected] * AXIS_STEPS_TO_MM[moveAxisSelected]) + "mm");
          }else{
            lcd.print(String((abs_pos[moveAxisSelected] - ref_pos[moveAxisSelected]) * AXIS_STEPS_TO_MM[moveAxisSelected]) + "mm");
          }
        }else{
          //Turn off all motors
          step_off();
          main_state = DETECT_DISPLAY_BUTTONS;
        }
      break;
      
      case MOVE_MINUS_CHECK_FOR_TERMINATION:
        
        //Check for termination: limit switch hit or button released
        if(digitalRead(LCD_BUTTON_PIN[1]) == LOW && digitalRead(LIMIT_PIN[moveAxisSelected][1]) == HIGH){
          //update display
          lcd.setCursor(6,1);
          
          if(abs_meas){
            lcd.print(String(abs_pos[moveAxisSelected] * AXIS_STEPS_TO_MM[moveAxisSelected]) + "mm");
          }else{
            lcd.print(String((abs_pos[moveAxisSelected] - ref_pos[moveAxisSelected]) * AXIS_STEPS_TO_MM[moveAxisSelected]) + "mm");
          }
        }else{
          //Turn off all motors
          step_off();
          main_state = DETECT_DISPLAY_BUTTONS;
        }

       break;

       case JOG_CHECK_FOR_TERMINATION:

        //Check if limit reached
        if(already_pressed == false && digitalRead(LIMIT_PIN[JOG_AXIS][!dir_to_move[JOG_AXIS]]) == LOW){
          //Stop motors
          step_off();
          
          //Switch direction
          dir_to_move[JOG_AXIS] = !dir_to_move[JOG_AXIS];          

          //Start motors again
          step_axis_cont(axis_to_move,jog_speed,dir_to_move);

          //In order to prevent multiple direction changes
          already_pressed = true;
          
        }else if(already_pressed == true && digitalRead(LIMIT_PIN[JOG_AXIS][dir_to_move[JOG_AXIS]]) == HIGH){
          already_pressed = false;
        }

        //check if stop button pressed
        if (digitalRead(LCD_BUTTON_PIN[1]) == LOW){
          //wait till the button is released
          while(digitalRead(LCD_BUTTON_PIN[1]) == LOW){}

          step_off();
          Serial.println("Stopping");
          jog_state = false;
          displayScreen_5();
          delay(200);
          main_state = DETECT_DISPLAY_BUTTONS;
        }
        

       break;

       case CAL_CHECK_FOR_TERMINATION:
        if(!calibrated){
          bool new_ax_to_move[3] = {false,false,false};
          
          for(int i=0; i < 3; i++){
            if(digitalRead(LIMIT_PIN[i][1]) == LOW){
              new_ax_to_move[i] = false;
              axis_calibrated += 1;
            }else{
              //Special case: move x axis only after y and z axis
              if(i == X_AXIS && digitalRead(LIMIT_PIN[Z_AXIS][1]) != LOW){
                //z axis has not been calibrated
                new_ax_to_move[i] = false;
              }else{
                new_ax_to_move[i] = true;
              }
            }
          }
          
          
          if(!areEqual(axis_to_move,new_ax_to_move,3)){
            //if something changed, then need to update motion
            copyArrayBool(new_ax_to_move,axis_to_move,3);
            step_axis_cont(axis_to_move,move_axis_speed,dir_to_move);
          }
          if(axis_calibrated >= 3){
            calibrated = true;
            calibrating = false;
            abs_pos[X_AXIS] = 0;
            abs_pos[Y_AXIS] = 0;
            abs_pos[Z_AXIS] = 0;
           
            displayScreen_message("Success");
            delay(1000);
            displayScreen_1();

            main_state = DETECT_DISPLAY_BUTTONS;
          }else{
            axis_calibrated = 0;
          }
        }

       break;
    }
  }
  
}

//Routines
void calibrate_machine(){
  calibrated = false;
  calibrating = true;
  
  int i = 0;
  
  update_motion = false;
  
  axis_calibrated = 0;
  
  limit_state[X_AXIS] = LOW;
  limit_state[Y_AXIS] = LOW;
  limit_state[Z_AXIS] = LOW;
  
  //Start moving all the motors. Want to move y and z axis first and then x axis
  axis_to_move[X_AXIS] = false;
  axis_to_move[Y_AXIS] = true;
  axis_to_move[Z_AXIS] = true;

  dir_to_move[X_AXIS] = false;
  dir_to_move[Y_AXIS] = false;
  dir_to_move[Z_AXIS] = false;
  
  //start moving the motors 
  step_axis_cont(axis_to_move,move_axis_speed,dir_to_move);

  main_state = CAL_CHECK_FOR_TERMINATION;

}

//event handlers
void buttonPressed_1(){

  
  switch(screenState){
      case 1:
        //screen 1: Move button pressed
        displayScreen_2();

        break;

      case 2:
        //screen 2: Back button pressed

        displayScreen_1();

        break;
        
      case 3:
        //screen 3: Back button pressed

        displayScreen_2();

        break;

      case 4:
         //Screen 4: Back button pressed

         displayScreen_3();

         break;

       case 5:
         displayScreen_1();
  
         break;
   }
}

void buttonPressed_2(){

  switch(screenState){
      case 1:
        {
          abs_meas = !abs_meas;
          
          if(!abs_meas){
            copyArrayInt(abs_pos,ref_pos,3);
          }else{
            ref_pos[X_AXIS] = 0;
            ref_pos[Y_AXIS] = 0;
            ref_pos[Z_AXIS] = 0;
          }

          Serial.println("Absolute mode: " + String((abs_meas ? "ON" : "OFF")));
          //refresh screen
          displayScreen_1();
        }
        break;
      
      case 2:
        //screen 2: X button pressed
        {
          moveAxisSelected = 0;
          displayScreen_3();
        }
        break;

      case 3:
        //screen 3: - button pressed
        {
          //Set direction
          dir_to_move[X_AXIS] = false;
          dir_to_move[Y_AXIS] = false;
          dir_to_move[Z_AXIS] = false;

          dir_to_move[moveAxisSelected] = false;
          
          //Set motor to move
          axis_to_move[X_AXIS] = false;
          axis_to_move[Y_AXIS] = false;
          axis_to_move[Z_AXIS] = false;
          
          axis_to_move[moveAxisSelected] = true;
      
          //start moving the motors 
          step_axis_cont(axis_to_move,move_axis_speed,dir_to_move);

          //Change main loop state to check for termination mode
          main_state = MOVE_MINUS_CHECK_FOR_TERMINATION;
          
        }

        break;

      case 4:
        {
          //Screen 4: - button pressed
          if (move_axis_speed > 0){
            move_axis_speed = move_axis_speed - 0.1;
          }
  
          //refresh screen
          displayScreen_4();
        }
        break;

       case 5:
        {
          if (jog_state == false){
            //need to reset state variables
            jog_state = true;
            already_pressed = false;
            
            displayScreen_5();
            
            //Start jogging

            //Set direction to move to true (positive direction)
            dir_to_move[X_AXIS] = false;
            dir_to_move[Y_AXIS] = false;
            dir_to_move[Z_AXIS] = false;
            
            dir_to_move[JOG_AXIS] = true;
  
            //Set axis to move to true
            axis_to_move[X_AXIS] = false;
            axis_to_move[Y_AXIS] = false;
            axis_to_move[Z_AXIS] = false;
            
            axis_to_move[JOG_AXIS] = true;
  
            //start moving the motors 
            step_axis_cont(axis_to_move,jog_speed,dir_to_move);
  
            //Monitor for interrupt condition - limit switches or stop button
            main_state = JOG_CHECK_FOR_TERMINATION;
            
          }
        }
        break;
   }
}

void buttonPressed_3(){

  bool result = false;
  
  switch(screenState){
    case 1:
      {
        //Screen 1: CAL button pressed      
        
        displayScreen_message("Calibrating...");
        calibrate_machine();

      }
      break;
      
    case 2:
      {
        //Screen 2: Y axis button pressed
        
        moveAxisSelected = 1;
        displayScreen_3();
      }
      break;

    case 3:
      {
        //Screen 3: + button pressed
        
        bool axis_to_move[3] = {false,false,false};
        
        bool dir_to_move[3] = {false,false,false};
  
        axis_to_move[moveAxisSelected] = true;
        dir_to_move[moveAxisSelected] = true;
        
        //start moving the motors 
        step_axis_cont(axis_to_move,move_axis_speed,dir_to_move);

        main_state = MOVE_PLUS_CHECK_FOR_TERMINATION;
      }
      
      break;

     case 4:
       {
         //Screen 4: + button pressed
         Serial.println("Speed increased");
         if (move_axis_speed < AXIS_SPEED_LIMIT[moveAxisSelected]){
            move_axis_speed = move_axis_speed + 0.1;
         }
  
         //refresh screen
         displayScreen_4();
       }
       break;

      case 5:
      {
        jog_speed -= 0.1;
        displayScreen_5();
      }
        break;
      
  }
    
}

void buttonPressed_4(){
  
  
  switch(screenState){
    case 1:
      //Screen home: JOG button pressed
      displayScreen_5();

      break;
    case 2:
      //Screen 2: Z axis button pressed
      
      moveAxisSelected = Z_AXIS;
      displayScreen_3();

      break;
      
    case 3:
      //Screen 3: Speed button pressed
      
      displayScreen_4();

      break;

     case 5:
      jog_speed += 0.1;
      displayScreen_5();
      break;
  }
}

//Screen refreshers
void displayScreen_1(){
  screenState = 1;
  Serial.println("screen State:");
  Serial.println(screenState);
  lcd.clear();
  button_state_cont[0] = false;
  button_state_cont[1] = false;
  button_state_cont[2] = false;
  button_state_cont[3] = false;

  float val[3];
  for(int i=0; i < 3; i++){
    val[i] = (abs_meas == true ? abs_pos[i] : abs_pos[i] - ref_pos[i]) * AXIS_STEPS_TO_MM[i];
  }
  lcd.setCursor(0,0);
  lcd.print("X:"+ String(val[0]) );
  lcd.setCursor(0,1);
  lcd.print("Y:"+ String(val[1]) );
  lcd.setCursor(0,2);
  lcd.print("Z:"+ String(val[2]) );
  
  lcd.setCursor(13,0);
  lcd.print(getCurrentStatus());

  lcd.setCursor(0,3);
  lcd.print("MOV   " + String((abs_meas ? "REF" : "ABS")) +  "   CAL  JOG");
}

void displayScreen_2(){
  screenState = 2;
  Serial.println("screen State:");
  Serial.println(screenState);
  lcd.clear();
  button_state_cont[0] = false;
  button_state_cont[1] = false;
  button_state_cont[2] = false;
  button_state_cont[3] = false;
  
  lcd.setCursor(4,1);
  lcd.print("SELECT AXIS");

  lcd.setCursor(0,3);
  lcd.print("BACK  X      Y     Z");
}

void displayScreen_3(){
  screenState = 3;
  Serial.println("screen State:");
  Serial.println(screenState);
  lcd.clear();
  button_state_cont[0] = false;
  button_state_cont[1] = true;
  button_state_cont[2] = true;
  button_state_cont[3] = false;
  lcd.setCursor(17,0);
  lcd.print((abs_meas ? "ABS" : "REF"));
  lcd.setCursor(4,1);
  
  switch(moveAxisSelected){
    case 0:
      lcd.print("X:" + String( (abs_meas ? abs_pos[0] * AXIS_STEPS_TO_MM[0]:(abs_pos[0] - ref_pos[0]) * AXIS_STEPS_TO_MM[0])) + "mm");
    break;

    case 1:
      lcd.print("Y:" + String( (abs_meas ? abs_pos[1] * AXIS_STEPS_TO_MM[1]:(abs_pos[1] - ref_pos[1]) * AXIS_STEPS_TO_MM[1])) + "mm");
    break;
    case 2:
      lcd.print("Z:" + String( (abs_meas ? abs_pos[2] * AXIS_STEPS_TO_MM[2]:(abs_pos[2] - ref_pos[2]) * AXIS_STEPS_TO_MM[2])) + "mm");
    break;
  }

  lcd.setCursor(0,3);
  lcd.print("BACK   -   +   SPEED");
}

void displayScreen_4(){
  screenState = 4;
  Serial.println("screen State:");
  Serial.println(screenState);
  lcd.clear();
  button_state_cont[0] = false;
  button_state_cont[1] = false;
  button_state_cont[2] = false;
  button_state_cont[3] = false;
  
  lcd.setCursor(2,1);
  lcd.print("SPEED:" + String(move_axis_speed) + "mm/s");

  lcd.setCursor(0,3);
  lcd.print("BACK   -   +");
}

void displayScreen_message(String msg){
  Serial.println("screen State:");
  Serial.println(screenState);

  int msg_len = msg.length();
  String out = "";
  //Center text
  if(msg_len <= 20){
    int spaces = (20 - msg_len) / 2;
    for(int i = 0; i < spaces; i++){
      out = out + " ";
    }
    out = out + msg;
  }else{
    out = msg;
  }
  
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(out);
}

void displayScreen_5(){
  //JOG screen
  
  screenState = 5;
  Serial.println("screen State:");
  Serial.println(screenState);
  lcd.clear();
  button_state_cont[0] = false;
  button_state_cont[1] = false;
  button_state_cont[2] = false;
  button_state_cont[3] = false;
  
  lcd.setCursor(4,1);
  lcd.print("Jog Speed: " + String(jog_speed));

  String st_stop = "START";
  if(jog_state == true){
    st_stop = "STOP";
  }
  
  lcd.setCursor(0,3);
  lcd.print("BACK  " + st_stop  + "  -   + " );
}

void set_axis_dir(bool dir[]){
  for(int i = 0; i < 3; i++){
    digitalWrite(MOTOR_DIR_PIN[i],dir[i]);
  }
}

void interlockTriggered(){
  if(digitalRead(INTERLOCK_PIN) == HIGH){
    interlock_on = false;
  }else{
    interlock_on = true;
  }
}

//Moves given stepper motor axis at a desired axis_speed in mm/s. Uses interrupt timers
void step_axis_cont(bool axis[],float axis_speed,bool dir_plus[]){
  //Ensure speed is within min/max range for each axis
  axis_speed = truncate_speed(axis_speed);

  //Convert axis_speed from mm/s to steps/second
  axis_speed = axis_speed / AXIS_STEPS_TO_MM[0];

  //Set which motors to run
  copyArrayBool(axis,motors_to_run,3);
  copyArrayBool(dir_plus,dir_to_run,3);
  
  //Set directions appropriately (Flip directions if needed)
  for(int i = 0; i < 3; i++){
    if(AXIS_DIRECTIONS[i] == true){
      dir_to_run[i] = !dir_to_run[i];
    }
  }
  
  //SPECIAL FEATURE (Dont allow user to run y axis without calibration, except during calibration)
  if(calibrated != true && axis[Y_AXIS] == true && calibrating != true){
    axis[Y_AXIS] = false;

    //Display error on screen saying to calibrate 
    displayScreen_message("Please calibrate");
    delay(2000);

    //Go back to previous screen
    displayScreenBeforeMessage();
    
    return;
  }
  
  
  //Set axis directions
  set_axis_dir(dir_to_run);

  //Setup and start interrupt timer
  noInterrupts();             // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  OCR3A = 62500/axis_speed;
  TCCR3B |= (1 << WGM32);     // CTC mode
  TCCR3B |= (1 << CS32);      // 256 prescaler
  TIMSK3 |= (1 << OCIE3A);   // Enable timer 
  interrupts();               // Enable all interrupts
}

//Turns off any and all stepper motors by turning off the timer.
void step_off(){
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  TIMSK3 &= ~(1 << OCIE3A);
  interrupts();
}

//Interrupt Routine that steps the motor
ISR(TIMER3_COMPA_vect){
  
  for(int i = 0;i < 3; i++){
    if(motors_to_run[i]){
      digitalWrite(MOTOR_PULSE_PIN[i],HIGH);
      //Minimum pulse width is 10microseconds so doing 15ms here
      delayMicroseconds(15);
      digitalWrite(MOTOR_PULSE_PIN[i],LOW);
      
      if(AXIS_DIRECTIONS[i] == true){
        abs_pos[i] = (dir_to_run[i] ? (abs_pos[i] - 1) : (abs_pos[i] + 1));    
      }else{
        abs_pos[i] = (dir_to_run[i] ? (abs_pos[i] + 1) : (abs_pos[i] - 1));  
      }
      
    }
  }
}

//if the speed is higher than any of the axis speed limits, sets it to the limit.
float truncate_speed(float spd){
  float speed_to_ret = 0.0;
  
  for(int i = 0; i<3; i++){
    if(spd > AXIS_SPEED_LIMIT[i]){
      spd = AXIS_SPEED_LIMIT[i];
    }
    if(spd > speed_to_ret){
      speed_to_ret = spd;
    }
  }

  return speed_to_ret;
}

//Helper function to display axis values in proper format (ex: 001 instead of 1)
String getFormattedValue(int x){
  String y = "";
  if (x < 10){
    y = String("00" + String(x));
  }else if(x < 100){
    y = String("0" + String(x));
  }else{
    y = String(x);
  }
  
  return y;
}

bool areEqual(bool arr1[], bool arr2[], int n) 
{ 
    // Linearly compare elements 
    for (int i=0; i<n; i++) 
         if (arr1[i] != arr2[i]) 
            return false; 
  
    // If all elements were same. 
    return true; 
} 

void copyArrayInt(int from[], int to[], int n){
  for (int i = 0; i < n; i++){
    to[i] = from[i];
  }
}

void copyArrayBool(bool from[], bool to[], int n){
  for (int i = 0; i < n; i++){
    to[i] = from[i];
  }
}

void displayScreenBeforeMessage(){
  
  //Display previous screen
  switch(screenState){
    case 1:
      displayScreen_1();
    break;
    case 2:
      displayScreen_2();
    break;
    case 3:
      displayScreen_3();
    break;
    case 4:
      displayScreen_4();
    break;
    case 5:
      //need to disable jog status
      jog_state = false;
      displayScreen_5();
    break;
  }
}

void flipArray(int arr[][2],int axis){
  int temp = arr[axis][0];
  arr[axis][0] = arr[axis][1];
  arr[axis][1] = temp;
}
//Helper function to display status bar in screen
String getCurrentStatus(){
  String abs_string = "ABS";
  if (abs_meas == false){
    abs_string = "REF";
  }
  String status_var = String(abs_string + "|mm");
  return status_var;
}
