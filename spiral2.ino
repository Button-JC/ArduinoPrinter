/****************************** Arduino Printer ****************************** 
******************************************************************************
* Simple alghoritmic 2D printing using floppy drives as stepper motor        *
* controlers                                                                 *
*                                                                            *
* http://junkcode.net/forum/wiki/doku.php/projects:arduino_printer           *
******************************************************************************

  The circuit:
  * pin 9, 6 direction and step of x-axis
  * pin 18, 10 direction and step of y-axis
  */

  //Constants
  #define SPEED_X 3200  // sets speed
  #define SPEED_Y 6400  //3200  // sets speed
  #define GRID_SIZE 4 //4
  #define Y_MULT 1
  #define X_MULT 2
  #define X_DIRECTION 1
  #define Y_DIRECTION 1   // -1
  #define STOP 0
  #define RUN 1

  // Stepper pins
  //int enable_pin = 10;   
  const int dir_pin = 9;     
  const int step_pin = 6;   
  const int dir_pin2 = 18;     
  const int step_pin2 = 10;    
  const int buttonPin = 4;             // start/stop pushbutton pin
  const int buttonPinMoveOut = 20;     // the number of the pushbutton pin to move tray out
  const int buttonPinMoveIn  = 8;      // the number of the pushbutton pin to move tray in
  const int ledPinRun =  21;           // the number of the on LED pin
  const int ledPinStop =  19;          // the number of the stoped LED pin
  
  int dir = 0;
  int x_pos = 0;
  int y_pos = 0;
  int mode = STOP;
  long cycle =0;

  /// testing params for various alghoritms
  int a = 1;
  float b = 0.4;
  int spiral_length = 2880;
  int buttonState = 0; 
  int buttonLastState = 0; 
  
  int step_size = 4;
  int square_size = 10;
  double eqt_triangle_height = (1.73205)/2.0;
  
void setup() {
  // Initial setup of pins   
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  pinMode(step_pin2, OUTPUT);
  
  pinMode(buttonPin, INPUT);   
  pinMode(buttonPinMoveOut, INPUT);
  pinMode(ledPinRun, OUTPUT);  
  pinMode(ledPinStop, OUTPUT);
  //pinMode(enable_pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  //digitalWrite(enable_pin, HIGH);
  delay(5);   
  digitalWrite(dir_pin, HIGH);  // clockwise
  digitalWrite(dir_pin2, HIGH);  // clockwise
  
  delay(1);  
  double angle;
  
  buttonState = digitalRead(buttonPin);
  if (buttonState==1 && buttonLastState==0){
    if (mode == STOP ){
      mode = RUN;
      dir=0;
      x_pos = 0;
      y_pos = 0;
      cycle = 0;
      digitalWrite(ledPinRun, HIGH);
      digitalWrite(ledPinStop, LOW);
    }else if (mode == RUN ){
      digitalWrite(ledPinStop, HIGH);
      //goToCoords(  0,  0);
      stopRun();
    }
  }
  buttonLastState=buttonState;
  if(cycle>200){
    stopRun();
  }else if(mode==RUN){
    
    /*   // Spiral 1
    for (int i = 0; i < spiral_length; i++) {
          angle = 0.1 * i;
          int x = (a + b * angle) * cos(angle);
          int y = (a + b * angle) * sin(angle);
  
          goToCoords(x, y);
          Serial.print("move to x=\t");
          Serial.print(x, DEC);
          Serial.print("\t y=\t");
          Serial.print(y, DEC);
          Serial.println();
          
    }*/
    
     /*  // calibration
      doCross(250/GRID_SIZE);
      goToCoords(  0,  0); 
      doStar(250/GRID_SIZE);
      goToCoords(  0,  0);
      doRhombus(250/GRID_SIZE); */
      /*for (long test_size=250/GRID_SIZE; test_size >50/GRID_SIZE; test_size-=50/GRID_SIZE) {
          doCross(test_size);
          doSquare(test_size);  
          doRhombus(test_size);  
          goToCoords(  0,  0);  
      }*/
    
      /* // "squared spiral"
      dir=(dir+1)%4;
      perform_step((cycle-cycle/3)*(dir>1?1:-1),dir%2);
      */
      
      
      // rotating square inception
      /*goToCoords(  0,  step_size*cycle);
      goToCoords(  square_size-step_size*cycle,  0);
      goToCoords(  square_size, square_size-step_size*cycle);
      goToCoords(  step_size*cycle, square_size);
      goToCoords(  0,  step_size*cycle);*/
      
      // rotating square inception v2
      int b = step_size*cycle;
      int a = b;//min(min(b,square_size/4),min(square_size-b,square_size/4));
      
      goToCoords(              - a,              + b);
      goToCoords(  square_size - b,              - a);
      goToCoords(  square_size + a,  square_size - b);
      goToCoords(              + b,  square_size + a);
      goToCoords(              - a,              + b);
      
       // rotating triangle inception
      /*goToCoords(  0,  step_size*cycle);
      goToCoords(  square_size-step_size*cycle,  0);
      goToCoords(  min(square_size/2+step_size*cycle,square_size),min(square_size,square_size+square_size/2-step_size*cycle));
      goToCoords(  0,  step_size*cycle);*/
      
      // rotating triangle inception v2
      /*goToCoords(  -cycle*step_size,  -cycle*step_size/2);
      goToCoords(  cycle*step_size+cycle/2,  -cycle*step_size/2);
      goToCoords(  0,  eqt_triangle_height*cycle*step_size);*/
      
    //}
    //delay(50000);
    cycle ++;
  }else{
    digitalWrite(ledPinStop, HIGH);
    if(buttonState==0){
      if(digitalRead(buttonPinMoveOut)==1){
        //moveXY(0,-10);
        moveXY(-10,0);
      }else if(digitalRead(buttonPinMoveIn)==1){
        //moveXY(0,5);
        moveXY(5,0);
      }
    }
  }
}
void stopRun(){
  mode = STOP;
  digitalWrite(ledPinRun, LOW);
  digitalWrite(ledPinStop, HIGH);
}
void doCross(int test_size){
   goToCoords( test_size,  0);  
   goToCoords(  0,  0);  
   goToCoords(-test_size,  0);  
   goToCoords(  0,  0);  
   goToCoords(  0, test_size);  
   goToCoords(  0,  0); 
   goToCoords(  0,-test_size);
}
void doSquare(int test_size){
  goToCoords( test_size,-test_size);  
  goToCoords( test_size, test_size);  
  goToCoords(-test_size, test_size);  
  goToCoords(-test_size,-test_size);
}
void doRhombus(int test_size){
  goToCoords(  0,-test_size);  
  goToCoords( test_size,  0);  
  goToCoords(  0, test_size);  
  goToCoords(-test_size,  0); 
  goToCoords(  0,-test_size);
}
void doStar(int test_size){
  goToCoords(  test_size,test_size);  
  goToCoords( -test_size,-test_size);  
  goToCoords(  0, 0);  
  goToCoords(  test_size,-test_size);  
  goToCoords( -test_size, test_size);  
}

void goToCoords(int x, int y){
  moveXY(x-x_pos,y-y_pos);
} 
void moveXY(int deltax, int deltay){
  x_pos +=deltax;
  y_pos +=deltay;
  //deltay*=Y_MULT;
  if(deltax==0){
    perform_step(deltay,1);
  }else{
    double error = 0;
    double deltaerr = abs ((double)deltay / (double)deltax);    // Assume deltax != 0 (line is not vertical),
    int xdir = deltax/abs(deltax);
    int ydir = deltay/abs(deltay);
    for(int xa=0; xa < abs(deltax); xa++){
      //x += xdir;
      perform_step(xdir,0); // move x
      error += deltaerr;
      while(error >= 0.5){
        perform_step(ydir,1); // move x
        //y := y + sign(y1 - y0)
        error -= 1.0;        
      }
    }
  }
  
}

void perform_step(long steps,int axis) {
  steps*=GRID_SIZE;
  int pin;
  int dir_p;
  int speed_delay;
  if(axis==0){
    pin = step_pin;
    dir_p= dir_pin;
    steps*=X_DIRECTION; //flip x axis
    steps*=X_MULT; 
    speed_delay = SPEED_X;
  }else{
    pin = step_pin2;
    dir_p = dir_pin2;
    steps*=Y_DIRECTION;
    steps*=Y_MULT; //strech y axis
    speed_delay = SPEED_Y;
    //steps*=-1; //flip y axis
  }
  if(steps<0){
    digitalWrite(dir_p, LOW);
    steps*=-1;
  }else{
    digitalWrite(dir_p, HIGH);
  }
  /*Serial.print("setps =");
  Serial.print(steps, DEC);
  Serial.print(" axis=");
  Serial.print(axis, DEC);
  Serial.println();*/
  for (long i=0; i < steps; i++) {
    digitalWrite(pin, LOW);
    delayMicroseconds(100);
    digitalWrite(pin, HIGH);
    delayMicroseconds(speed_delay);
  }
  // Set the pin low before we end
  digitalWrite(pin, LOW);
}
