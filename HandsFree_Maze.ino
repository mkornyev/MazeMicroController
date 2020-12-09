#include <Servo.h>

// MODES
bool DEBUG_COORDINATES = false; // Print output (coords computed first)
bool DEBUG_ANGLES = false; // (angles computed from coords)
bool LISTEN_INCOMING_MESSAGES = false; // Accept remote input? 

// CONSTANTS
const int X_SERVO_PIN = 6;
const int Y_SERVO_PIN = 7;

const int L_ECHO_PIN = 13;
const int L_TRIGGER_PIN = 12;
const int R_ECHO_PIN = 11;
const int R_TRIGGER_PIN = 10;

const int PHOTORESISTOR_PIN = A0;
const int LED_PIN_WIN = 9; // A flashing LED for finishing the maze 

// TUNE the following as needed
// Ctrl + F for "TUNE" to find all other tunable params
const float BASELINE = 70.0; // Triangle Baseline
const int MAX_ANGLE = 20;  // Max that the servo will rotate in either direction from STARTING_ANGLE
const int START_ANGLE = 15; // Starting angle for a servo
const int SERVO_INCREMENT_DELAY = 10; // Delay between 1-degree servo increments

int X = 35; // Starting X coordinate
int Y = 60; // Starting Y coord

const int X_MAX_DISTANCE = 44; 
const int X_MIN_DISTANCE = 25; 
const int Y_MAX_DISTANCE= 80;
const int Y_MIN_DISTANCE= 40;

// GLOBALS
Servo xServo;  // Servos
Servo yServo;
float leftDistance; // Hand distance in cm
float rightDistance;
int lAngle; // Corresponding servo angle
int rAngle;
float lightLevel = 1023; // kOhm

void setup()
{
  Serial.begin(115200);
  xServo.attach(X_SERVO_PIN);
  yServo.attach(Y_SERVO_PIN);
  pinMode(X_SERVO_PIN, OUTPUT);
  pinMode(Y_SERVO_PIN, OUTPUT);
  pinMode(LED_PIN_WIN, OUTPUT);
  pinMode(PHOTORESISTOR_PIN, INPUT);
  xServo.write(START_ANGLE);
  yServo.write(START_ANGLE);
}

void loop()
{
  if(LISTEN_INCOMING_MESSAGES) { // If listening for messages
    while(Serial.available()) {
      float lVal = Serial.parseFloat(); Serial.find(','); // Parse a value in range [-1.0, 1.0]
      float rVal = Serial.parseFloat(); Serial.find('\n');

      lAngle= fmap(lVal, -1.0, 1.0, -20, 20);  // Perform a float conversion 
      rAngle = fmap(rVal, -1.0, 1.0, -20, 20);
    }
  } else {
    // Get raw distances
    leftDistance = 0.01723 * handDistance(L_TRIGGER_PIN, L_ECHO_PIN);
    rightDistance = 0.01723 * handDistance(R_TRIGGER_PIN, R_ECHO_PIN);
  
    getCoordinates(); // set the X & Y coordinates
  
    int rawLAngle = map(X, X_MIN_DISTANCE, X_MAX_DISTANCE, -MAX_ANGLE, 35); // Using X & Y coordinates 
    int rawRAngle = map(Y, Y_MIN_DISTANCE, Y_MAX_DISTANCE, -MAX_ANGLE, MAX_ANGLE);
  
    // Use 2nd smoothing filter
    lAngle = getSmoothedValue(lAngle, rawLAngle, 1);   // TUNE: 1 langle-multiplier (1 = NO DELAY)
    rAngle = getSmoothedValue(rAngle, rawRAngle, 0.8); // TUNE: 0.8 rangle-multiplier
    
    // OR use a highpass filter (with smaller movement area, but more control)
  //  lAngle = highpass(rawLAngle);
  //  rAngle = highpass(rawRAngle);
  
    sanitizeAngles(); // Sanitize angle input 

    // Publish the calculated angles
    PUBLISH(lAngle, rAngle); 
  }

  // Move servos
  moveServo(xServo, START_ANGLE + lAngle);
  moveServo(yServo, START_ANGLE + rAngle);
 
//  pollPhotoresistor(); // See whether the phototransistor was activated & play win sequence

  if (DEBUG_ANGLES) {
    Serial.print("L: ");
    Serial.println(lAngle);
    Serial.print("R: ");
    Serial.println(rAngle);
//    Serial.print("LL: ");
//    Serial.println(lightLevel);
  }

  delay(100);
}

long handDistance(int TRIGGER_PIN, int ECHO_PIN) {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  return pulseIn(ECHO_PIN, HIGH);
}

// Simple smoothing function for a given value
int getSmoothedValue(float value, float input, float coeff) {
  float difference = input - value;
  return (value + (int)(coeff * difference));
}

// Simple highpass filter 
// - for supressing occasionall (low-freqency) erroneous vals
float highpass(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.04859958*z1 - 0.29614036*z2;
    output = 0.43284664*x + -0.86569329*z1 + 0.43284664*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.32091343*z1 - 0.63273879*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Keeps the calculated angles within their bounds
void sanitizeAngles() {
  if (lAngle > 35) { // Left
    lAngle = 35;
  }
  if (lAngle < -MAX_ANGLE) {
    lAngle = -MAX_ANGLE;
  }
  if (rAngle > MAX_ANGLE) { // Right
    rAngle = MAX_ANGLE;
  }
  if (rAngle < -MAX_ANGLE) {
    rAngle = -MAX_ANGLE;
  }
}

// Smooth/incremented servo movement
void moveServo(Servo &s, int angle) { // Pass sevo by reference 
  int previousAngle = s.read();

  if (angle > previousAngle)
  {
    for (int i = previousAngle; i <= angle; i++)
    {
      s.write(i);
      delay(SERVO_INCREMENT_DELAY);
    }
  }

  if (angle < previousAngle)
  {
    for (int i = previousAngle; i >= angle; i--)
    {
      s.write(i);
      delay(SERVO_INCREMENT_DELAY);
    }
  }
}

// Function to calculate the realtive X & Y distances of the sensed object 
// Using Heron's formula & some Pythagoras
void getCoordinates() {
  // Get Heron variables
  float a = float(leftDistance);    //d2 (vertex -> sensor B)
  float b = float(rightDistance);                       //d1 (vertex -> sensor A)
  float c = BASELINE;                               //baseline
  //float d = c*1.414;                              //display diagonal (square)
  float d = sqrt(150 * 150 + 100 * 100);            //diagonal (display + offset)
  float s = (a + b + c) / 2;                        //semi-perimeter

  if
  (
    (a < 0) ||          
    (b > d) ||          
    (a > d) ||          
    ((s - a) < 0) ||    
    ((s - b) < 0) ||
    ((s - c) < 0)
  )
  {
    Serial.println("BAD VAL"); 
    return; // Dont change the coordinates
  }

  float area = sqrt(s * (s - a) * (s - b) * (s - c));
  Y = getSmoothedValue(Y, (area * 2 / c), 0.9);      // TUNE: 0.9 y-multiplier
  X = getSmoothedValue(X, sqrt(b * b - Y * Y), 0.65); // TUNE: 0.65 x-multiplier

  if (DEBUG_COORDINATES) {
    Serial.print("     X: ");
    Serial.println(X);
    Serial.print("     Y: ");
    Serial.println(Y);
    Serial.println("");
  }
}

// Triggers the LED celebration when the phototransistor was triggered
void pollPhotoresistor() {
  lightLevel = (float)analogRead(PHOTORESISTOR_PIN); // Input range: 1023 - 319
 
  // Blink LED if the resistor was triggered
  if (lightLevel < 1000) {                          // TUNE: 1000 threshold
    for(int i=0; i<3; i++) {
      digitalWrite(LED_PIN_WIN, HIGH); delay(500);    
      digitalWrite(LED_PIN_WIN, LOW); delay(10);  
    }
  }
}

// Publishes message to MQTT via Serial
void PUBLISH(int a, int b) {
  if(DEBUG_COORDINATES || DEBUG_ANGLES) {
    Serial.println("ERROR: cannot print DEBUG and PUBLISH @ the same time");
    return; 
  }
  Serial.print(a); Serial.print(","); Serial.print(b); Serial.print("\n"); 
}

// fmap implementation from Garth Zeglin @ CMU w/var bounding
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  float divisor = in_max - in_min;
  if (divisor == 0.0) {
    return out_min;
  } else {
    float outval = (x - in_min) * (out_max - out_min) / divisor + out_min;
    
    if(outval > out_max){
      return out_max;
    } 
    if(outval < out_min){
      return out_min;
    }  
    return outval; 
  }
}
