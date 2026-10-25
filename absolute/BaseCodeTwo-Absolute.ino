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
float res = 11.25;
float error_vector[10];

// Sensor analogue reading for phototransistors, T5 = MSB, T1 = lSB
int T5 = A1;
int T4 = A2;
int T3 = A3;
int T2 = A4;
int T1 = A5;
byte start_pos_gray; 
byte home;
int current_pos = 0;   
byte final_pos_gray = 0; 
int final_pos_angle;
int threshold = 200;
// Code to test what the absolute sensor is reading as low and high
////////////////////////////////////////////////////
void printBits5(byte g) {
  for (int i = 4; i >= 0; i--) Serial.print((g >> i) & 1);
}
/////////////////////////////////////////////////////


void setup() {
  // Set Pins for LED power as output
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  // Set power on for LED
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
 
  // Set phototransistor readings as inputs
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  
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
  start_pos_gray = readGray(); // Get position at start of the 10 readings
  home = readGray(); // Set current position as home 
  current_pos = 0;                               
}

float kp = .6*90/deg; //proportional gain of PI
float ki = .02; //integral gain of PI 

// This function reads all the values of the transistors and converts them to either being 
// on or off if the read value from their pins is greater than or less than the threshold number, 
// Then adds them onto a byte to read as the encoder value. 
byte readGray() {
  byte g = 0;
  g |= ((analogRead(T1) < threshold) << 0);
  g |= ((analogRead(T2) < threshold) << 1);
  g |= ((analogRead(T3) < threshold) << 2);
  g |= ((analogRead(T4) < threshold) << 3);
  g |= ((analogRead(T5) < threshold) << 4);
  
  return g; 
}

// Prints out the Sector and gray code value of a byte given, used for testing our encoder. 
// Iterates through each bit in the byte printing it and then printing out the sector. 
void printGray(byte g) {
  byte sector = grayToBinary(g);
  for (int i = 4; i >= 0; i--) Serial.print((g >> i) & 1);
  Serial.print("  sector ");
  Serial.println(sector);
}

// Gray code table refering to each of the 32 different sectors. 
const byte GRAY_TABLE[32] = {
  0b00000, 0b00001, 0b00011, 0b00010,   // 0-3
  0b00110, 0b00111, 0b00101, 0b00100,   // 4-7
  0b01100, 0b01101, 0b01111, 0b01110,   // 8-11
  0b01010, 0b01011, 0b01001, 0b01000,   // 12-15
  0b11000, 0b11001, 0b11011, 0b11010,   // 16-19
  0b11110, 0b11111, 0b11101, 0b11100,   // 20-23
  0b10100, 0b10101, 0b10111, 0b10110,   // 24-27
  0b10010, 0b10011, 0b10001, 0b10000    // 28-31
};

// Function takes gray code reading and returns the sector number that it is in our encoder, s\So that we can 
// interpret it as the sector we are in. It iterates through the gray code table in a for loop until it finds 
// the gray code value we have given to the function g, it then returns what the iteration is which is also 
// the sector number. 
byte grayToBinary(byte g) {
  for (byte n = 0; n < 32; n++) {
    if (GRAY_TABLE[n] == g) return n;
  }
  return 0;   // not reachable: all 32 five-bit codes are in the table
}

// Function takes the two bytes as the end and start byte on the encoder, it converts it to binary (getting the 
// sector number) calculating the difference, if the difference is less than or greater than 10, then overflow 
// has occured and we add or subtract 32 sectors, we then return the angle moved as the difference of sectors 
// times the resolution. 
float angleMoved(byte end, byte start) {
  int diff = grayToBinary(end) - grayToBinary(start); 
  if (diff > 10) {
    diff = diff - 32; 
  } else if ( diff < -10) {
    diff = diff + 32;
  }
  return diff * res; 
}

// Calculates the average of the best 8 angle tests. 
float bestEightAverage() {
  float sum = 0;
  float max1 = -1, max2 = -1; // largest and second largest

  for (int i = 0; i < 10; i++) {
    float e = error_vector[i];
    sum += e;
    if (e > max1)      { max2 = max1; max1 = e; }
    else if (e > max2) { max2 = e; }
  }
  return (sum - max1 - max2) / 8.0;
}

void loop() {
// Keep track of the sector and if sector changes print the new sector
if (readGray() != current_pos) {
  current_pos = readGray();
}

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
      final_pos_gray = readGray(); // Get gray code of sensor current position at stationary end position
      float opticalAngle = angleMoved(final_pos_gray, start_pos_gray); // calculate angle moved from start and end
      float angleFromHome = angleMoved(final_pos_gray, home); // calculate angle moved from end and home 
      if (angleFromHome < 0) { // add 360 to the angle from home if its done a full revolution
        angleFromHome = angleFromHome + 360; 
      }
      String direction = "";
      if (opticalAngle>0){ // If angle is positive then make direction CW
        direction = "CW";
      } else if (opticalAngle < 0){ // If angle is negative then make direction CCW
        direction = "CCW";
      }
      opticalAngle = abs(opticalAngle); // Get absolute value of angle as we print direction
      //FROM OUR ENCODER BUILDS
      Serial.print("shaft possition from optical absolute sensor from home position: ");
      Serial.println(angleFromHome);
      Serial.print("shaft displacement from optical absolute sensor: ");
      Serial.println(opticalAngle);
      Serial.println("Direction: " + direction);
      Serial.print("Start gray: ");
      Serial.print("Shaft displacement from motor's builtin encoder: ");
      Serial.println(s * 360 / 228); //every full Revolution of the shaft is associated with 228 counts of builtin s = built in encoder counts

      float Error = opticalAngle -s*360/228;
      start_pos_gray = readGray();
      error_vector[rep-1] =  abs(Error);
      Serial.print("Error :");
      Serial.println(Error); //displaying error
      Serial.println();
      delay(500);
      s = 0;
      finish=0; 
      analogWrite(6,0);

      if (rep > 10) {
        Serial.print("Average error, best 8 of 10: ");
        Serial.println(bestEightAverage());
      }
    }
}
