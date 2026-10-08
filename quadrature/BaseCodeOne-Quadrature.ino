int b=0; //reading the time for main loop to be run for 15s
int c=0; //memory for the time in mainloop

float s=0;   //built-in encoder counts
float s_2;   //built-in encoder counts for RPM calculation for PI controler

float rpmm;  //rpm obtained each 5s from built-in encoder

int s1=0;    //built-in encoder chanel one outpot
int s2=0;    //built-in encoder chanel two outpot
int r=0;     //repetition indicator for reading counts of bult-in encoder
int s2m=0;   //memory of built-in encoder chanel two outpot
int directionm=0;  //indicator for direction read by built-in encoder
int dirm;          //indicator for direction read by built-in encode
int RPM;           //Commanded RPM

int exitt=0;       //mainloop exit condition

float ctrl;      //PI controller outpot
float kp=.4;     //proportional gain of PI controller 
float ki=.01;    //integral gain of PI controller
float eri;       //integral of error of PI controller

int repc=1;      //repetition condition of PI controller
int t0;          //memory of time for the Purpose of displaying the results
int repeat=0;    //repeat indicator to only let the memory of time for the Purpose of displaying the results be updated once


// Optical quadrature encoder: 2 LED/phototransistor pairs, pass-through disk,
// 24 slots + 24 blocked per track, track B offset a quarter cycle from track A.
// Convention from our hand tests: CW = outer track (A) leads inner (B), CCW = inner leads outer.

const int QUAD_LED_PIN = A1;   // drives both quadrature IR LEDs
const int QUAD_PIN_A   = A5;   // : outer track phototransistor (channel A)
const int QUAD_PIN_B   = A4;   // : inner track phototransistor (channel B)

const int QUAD_TH_HIGH = 200;  // : reading (0-1023) must rise above this to count as "open"
const int QUAD_TH_LOW  = 130;  // : reading must fall below this to count as "blocked"

const long QUAD_CPR = 96;      // : counts per rev = 24 cycles x 4 (x4 decoding: every edge of A and B)

const bool QUAD_INVERT_DIR = false;   // : set true if our CW/CCW label comes out backwards vs the built-in encoder

bool quadA = false;                   // clean digital state of channel A (true = light passing)
bool quadB = false;                   // clean digital state of channel B
byte quadState = 0;                   // current state as 2 bits: (A << 1) | B
byte quadPrevState = 0;               // previous state
long quadNet = 0;                     // signed count, +1 per CW step and -1 per CCW step (reset every 5 s)
unsigned long quadWindowStart = 0;    // millis() when the current 5 s window began
bool quadCW = true;                   // direction result for the current window

// lookup table indexed by (previousState << 2) | currentState
// CW sequence  (A,B): 00 -> 10 -> 11 -> 01 -> 00  gives +1
// CCW sequence (A,B): 00 -> 01 -> 11 -> 10 -> 00  gives -1
// no change, or an impossible jump (both channels flip at once), gives 0
const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};
// ===== END NEW =============================================================


void setup() {
  // put your setup code here, to run on
  Serial.begin(250000);                  //Baud rate of communication 

  // ===== NEW ==================================================
  setupQuadrature();   // : turn on the quadrature LEDs (A1), speed up the ADC and record the starting A/B state
  // ===== END ===========================================================

  Serial.println("Enter the desired RPM.");  
  
  while (Serial.available() == 0)   
  { 
    //Wait for user input
  }  
  
   RPM = Serial.readString().toFloat(); //Reading the Input string from Serial port.
  if (RPM<0)
  {
    analogWrite(3,255);                 //changing the direction of motor's rotation
  }
  RPM=abs(RPM);
  
}

void loop() {

b=millis();    //reading time
c=b;           //storing the current time 

while ((b>=c) && (b<=(c+15500)) && exitt==0)   //let the main loop to be run for 15s
{

  if (b%13==0 && repc==1)                   //PI controller
  {
  eri=ki*(RPM-rpmm)+eri;
  ctrl=50+kp*(RPM-rpmm)+eri;
  analogWrite(6,ctrl);
  repc=0;
  }
  if(b%13==1)
  {
    repc=1;
  }

  // ===== NEW ==================================================
  readQuadratureChannels();      // read both phototransistors and turn them into clean 0/1 states (with hysteresis)
  countQuadratureEdges();        // if A or B changed, add +1 (CW step) or -1 (CCW step) to the running count
  updateQuadratureDirection();   // decide CW/CCW from which channel is leading (sign of the running count)
  // ===== END ===========================================================

  s1=digitalRead(7);           //reading Chanel 1 of builtin encoder
  s2=digitalRead(8);           //reading Chanel 2 of builtin encoder
 if (s1!=s2 && r==0)
 {
  s=s+1;      //counters for rpm that displyed every 5s
  s_2=s_2+1;  //counters for rpm that used in PI contoller
  r=1;        // this indicator wont let this condition, (s1 != s2), to be counted until the next condition, (sm1 == sm2), happens
 }

 if (s1==s2 && r==1)
 {
  s=s+1;                                                //counters for rpm that displyed every 5s
  s_2=s_2+1;                                            //counters for rpm that used in PI contoller
  r=0;                                                  // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
 }

b=millis();                                             //updating time
if (b%100<=1 && repeat==0)
{
  t0=b;                                                 //storing the current time once
  repeat=1;
}


if (b%100==0)
{
  Serial.print("time in ms: ");
  Serial.print(b-t0);
  
  Serial.print("  spontaneous speed from builtin encoder:  ");
  rpmm=(s_2/(2*114))*600;                               //formulation for rpm in each 100ms for PI controller
  Serial.println(rpmm);
  s_2=0;                                                //reseting the counters of PI controller rpm meter
  

  if ((b-t0)%5000==0)
  {
  // ===== NEW  ==================================================
  float quadRPM = calculateQuadratureRPM();   // convert the 5 s edge count into RPM (done before printing)
  // ===== END ===========================================================

  Serial.println();
  Serial.print("RPM from builtin encoder: ");
  Serial.println((s/(228))*12);                         //formula for rpm in each 5s
  
  Serial.print("RPM from optical quadrature encoder: ");
  // Serial.println(0);                                 // BASE line (replaced by the NEW line below)
  Serial.println(quadRPM);                              // NEW: print our encoder's RPM
  
  Serial.print("Error: ");
  // Serial.println(-(s/(228))*12);                     // BASE line (replaced by the NEW line below)
  Serial.println(quadRPM - (s/(228))*12);               // NEW: error = ours - built-in (as the brief defines it)
  
  Serial.print("direction read by motor's sensor: ");
  if (dirm==0){Serial.print("CW");}
  else{Serial.print("CCW");}
  Serial.print("  ,   ");
  
  Serial.print("direction read by sensor:  ");
  // Serial.println("");                                // BASE line (replaced by the NEW line below)
  Serial.println(getQuadratureDirectionString());       // NEW: print CW or CCW from our encoder
  Serial.println();

  s=0;
  directionm=0;

  // ===== NEW ==================================================
  resetQuadratureWindow();   // restart our count and timer so the next 5 s window lines up with the built-in encoder's
  // ===== END ===========================================================
  }
  delay(1);
}

if((s1==HIGH)&&(s2==HIGH)&&(s2m==LOW))                  //reading the direction of motor by cheaking which chanel follows which
{
  directionm=directionm+1;
}

if((s1==LOW)&&(s2==LOW)&&(s2m==HIGH))
{
  directionm=directionm+1;
}

s2m=s2;                                                 //memory of the previous builtin encoder chanel 2

if (directionm>100)
{
  dirm=0;
}
if (directionm<20)
{
  dirm=1;
}

b=millis();                                             //updating time

}
analogWrite(6,0);                                       //turning off the motor
exitt=1;                                                //changing the exit condition to prevent the motor to run after 15s
}

// runs once from setup(): turns the LEDs on, speeds up the ADC, grabs the starting state
void setupQuadrature() {
  pinMode(QUAD_LED_PIN, OUTPUT);           // A1 used as a digital output to power the LEDs
  digitalWrite(QUAD_LED_PIN, HIGH);        //  LEDs on
  ADCSRA = (ADCSRA & 0xF8) | 0x05;         //  ADC prescaler 32 (~26 us per read instead of ~112 us) so the loop samples fast
  delay(10);                               //  let the LEDs and phototransistors settle

  analogRead(QUAD_PIN_A);                  // dummy read (first read after switching channel can be off)
  int a = analogRead(QUAD_PIN_A);          // real read of channel A
  analogRead(QUAD_PIN_B);                  // dummy read
  int bb = analogRead(QUAD_PIN_B);         // real read of channel B

  quadA = (a > (QUAD_TH_HIGH + QUAD_TH_LOW) / 2);    // starting state, split at the midpoint
  quadB = (bb > (QUAD_TH_HIGH + QUAD_TH_LOW) / 2);
  quadState = (quadA ? 2 : 0) | (quadB ? 1 : 0);     // pack A and B into 2 bits
  quadPrevState = quadState;               // so the first pass doesn't see a fake edge

  quadNet = 0;                             // start counting from zero
  quadWindowStart = millis();              // start of the first measuring window
}

// every loop pass: read both phototransistors and turn them into clean 0/1 states
void readQuadratureChannels() {
  analogRead(QUAD_PIN_A);                  // dummy read to clear the ADC after a channel switch
  int a = analogRead(QUAD_PIN_A);          // raw channel A (0-1023)
  analogRead(QUAD_PIN_B);                  // dummy read
  int bb = analogRead(QUAD_PIN_B);         // raw channel B (0-1023)

  if (a > QUAD_TH_HIGH) quadA = true;      // hysteresis: only go "open" above the high threshold
  else if (a < QUAD_TH_LOW) quadA = false; // only go "blocked" below the low threshold (in between = keep old state)

  if (bb > QUAD_TH_HIGH) quadB = true;     // same for channel B
  else if (bb < QUAD_TH_LOW) quadB = false;

  quadState = (quadA ? 2 : 0) | (quadB ? 1 : 0);     // combine into a 2-bit state
}

// every loop pass: if the state changed, add +1 (CW step) or -1 (CCW step) to the count
void countQuadratureEdges() {
  if (quadState != quadPrevState) {                                  // only act when A or B changed
    quadNet += QUAD_TABLE[(quadPrevState << 2) | quadState];         // table says which way that step went
    quadPrevState = quadState;                                       // remember for next time
  }
}

// every loop pass: work out direction from which channel is leading
// (the table already encodes "A leads = CW", so the sign of the net count tells us the winner)
void updateQuadratureDirection() {
  quadCW = (quadNet >= 0);                 // more CW steps than CCW steps = CW
  if (QUAD_INVERT_DIR) quadCW = !quadCW;   // flip the label if it disagrees with BaseCodeOne's CW/CCW
}

// called in the 5 s print block: converts the count over the window into RPM
float calculateQuadratureRPM() {
  unsigned long elapsed = millis() - quadWindowStart;   // actual window length in ms (about 5000)
  if (elapsed == 0) return 0.0;                         // avoid divide by zero at the very first print
  float revs = (float)abs(quadNet) / (float)QUAD_CPR;   // revolutions in the window (magnitude only)
  return revs * 60000.0 / (float)elapsed;               // revs per ms -> revs per minute
}

// called in the 5 s print block: text for the display
const char* getQuadratureDirectionString() {
  return quadCW ? "CW" : "CCW";            // same style as BaseCodeOne's own direction print
}

// called in the 5 s print block right after printing: start a fresh window
// (same moment BaseCodeOne resets the built-in count 's', so both cover the same 5 s)
void resetQuadratureWindow() {
  quadNet = 0;                             // clear the count
  quadWindowStart = millis();              // restart the window timer
}
