float deg=45; // Rotation degree
float s = 0;  //Encoder counts
int sm1 = 0;  //Built-in chanel 1
int sm2 = 0;  //Built-in chanel 2
int r = 0;  //indicator for reading builtin encoder to avoid the reading redundancy
float er; //Proportional error for PI controller
float eri; //Integral error for PI controller
int t=0; //time in ms
int t0=0; //memory for time in ms
int finish=0; //finish indicator
int rep=1; //Repetition indicator
int res = 11.25;
int T5 = A1;
int T4 = A2;
int T3 = A3;
int T2 = A4 ;
int T1 = A5;
byte start_pos_gray; 
int current_pos = 0;   
byte final_pos_gray = 0; 
int final_pos_angle;
int threshold = 3.5; 

void setup() {
  Serial.begin(250000); //Baud rate of communication 
  Serial.println("Enter the desired rotation in degree.");  
  while (Serial.available() == 0) //Obtaining data from user
  { 
    //Wait for user input
  }  
  deg = Serial.readString().toFloat(); //Reading the Input string from Serial port.
  if (deg<0)
  {
    analogWrite(3,255); //change the direction of rotation by applying voltage to pin 3 of arduino
  }
  deg=abs(deg);   
  start_pos_gray = readGray(); 
  current_pos = 0;    
  //int gray = 4; 
  //int bin = grayTobinary(gray); 
  //print(bin);                                  
}

float kp = .6*90/deg; //proportional gain of PI
float ki = .02; //integral gain of PI 

void loop() {
//THIS SECTION IS FOR MOVING THE THING, DON'T TOUCH IT
t=millis(); //reading time
t0=t; //saving the current time in memory
  while (t<t0+4000 && rep<=10){ //let the code to ran for 4 seconds each with repetitions of 10
    
if (t%10==0) //PI controller that runs every 10ms
{ 
    if (s < deg * 114 * 2 / 360)
    {
      er = deg - s *360/228;
      eri = eri + er;
      analogWrite(6, kp * er + ki * eri);
    }

    if (s >= deg * 228/360)
    {
      analogWrite(6, 0);
      eri = 0;
    }
    delay(1);
}

//OUR SENSOR


//FOR READING THE SENSOR
    sm1 = digitalRead(7); //reading chanel 1 
    sm2 = digitalRead(8); //reading chanel 2
   
    if (sm1 != sm2 && r == 0) { //counting the number changes for both chanels
      s = s + 1;
      r = 1;  // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
    }
    if (sm1 == sm2 && r == 1) {
      s = s + 1;
      r = 0;  // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }

t=millis();
finish=1;

  }

if (finish==1){                                
  //this part of the code is for displaying the result
      delay(500);
      rep=rep+1;
      final_pos_gray = readGray(); 
      int opticalAngle = angleMoved(start_pos_gray, final_pos_gray);
      int angleFromHome = angleMoved(0, final_pos_gray); 

      //FROM OUR ENCODER BUILDS
      Serial.print("shaft possition from optical absolute sensor from home position: ");
      Serial.println(angleFromHome);
      
      Serial.print("shaft displacement from optical absolute sensor: ");
      Serial.println(opticalAngle);
      
      Serial.print("Shaft displacement from motor's builtin encoder: ");
      Serial.println(s * 360 / 228); //every full Revolution of the shaft is associated with 228 counts of builtin s = built in encoder counts
                                                                    
      float Error=opticalAngle-s*360/228;
      Serial.print("Error :");
      Serial.println(Error); //displaying error
      Serial.println();
      s = 0;
      finish=0; 
  
}
analogWrite(6,0); //turning off the motor
}

byte readGray() {
  byte g = 0;
  g |= ((analogRead(T1) > threshold) << 0);
  g |= ((analogRead(T2) > threshold) << 1);
  g |= ((analogRead(T3) > threshold) << 2);
  g |= ((analogRead(T4) > threshold) << 3);
  g |= ((analogRead(T5) > threshold) << 4);
  return g; 
}

byte grayToBinary(byte g) {
  g ^= g >> 1;
  g ^= g >> 2;
  g ^= g >> 4;
  return g;
}

int angleMoved(byte start, byte end) {
  int diff = grayToBinary(start) - grayToBinary(end); 
  if (diff > 9) {
    diff = diff -32; 
  } else if ( diff < -9) {
    diff = diff + 32;
  }

  return diff * res; 
}