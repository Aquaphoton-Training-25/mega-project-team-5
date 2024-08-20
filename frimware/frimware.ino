//F=Front, B=Back, L=Left, R=Right

// Front Motors 
#define motorFR1 4 
#define motorFR2 5
#define motorFL1 7
#define motorFL2 8

// Back Motors
#define motorB1 2  // b1 first mutual input
#define motorB2 3  // b2 second mutual input 

#define speedPin 6
const int lowSpeed = 0;
const int mediumSpeed = 1;
const int highSpeed = 2;

int speed = lowSpeed;

int motion;
// Ultrasonic sensors pins
#define TRIG_SIDE 12
#define ECHO_SIDE A2
#define TRIG_FRONT 13
#define ECHO_FRONT A3

// PID variables
float setpoint = 10.0; // Desired distance from the wall in cm
float Kp = 1.0;        // Proportional gain
float Ki = 0.01;       // Integral gain
float Kd = 0.01;       // Derivative gain

float previous_error = 0;
float integral = 0;
float distance;       // (PID input) Current distance from the wall
float output;         // PID output
unsigned int Last_Time = 0;
const unsigned int Sample_Time = 100; // Time interval between PID calculations in milliseconds

// Define mode variable (manual or autonomous)
int mode = -1;  // -1 indicates no mode selected yet, 0 = Manual, 1 = Autonomous
int motorSpeed = 255; // Default speed (maximum speed)

// Obstacle avoidance parameters
float obstacleDistance = 10.0; // Front distance in cm to detect an obstacle

// Voltage sensor reading

#define voltageReading A5

float vout,vValue;

// current reading

#define currentReading A4
float cValue =0.0;
float vRead =0.0;
const float sensitivity = 100.0,currentRead; // 100mV/A
// RGB
#define greenPin 9
#define bluePin 10
#define redPin 11


// Making a variable named motion
// Assigned to 1 when moving forward or after stopping
// Assigned to 0 when moving backward
void setup() {
  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(motorB1,OUTPUT);
  pinMode(motorB2,OUTPUT);
  pinMode(motorFR1,OUTPUT);
  pinMode(motorFR2,OUTPUT);
  pinMode(motorFL1,OUTPUT);
  pinMode(motorFL2,OUTPUT);
  pinMode(speedPin, OUTPUT);
  setSpeed('l');
  // Set voltage sensor pin
  pinMode(voltageReading,INPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_SIDE, OUTPUT);
  pinMode(ECHO_SIDE, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  
  // Set current sensor pin
  pinMode(currentReading,INPUT);

 // RGB pins
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(redPin, OUTPUT);

}
#define time_to_send_data 1500
long pevTime;

void loop() {
  // check the obstacle after the mode is selected
  if (Serial.available()>0){
    char recieved_command = Serial.read();
    switch(recieved_command){
      // Manual control
        case 'F':
        case 'B':
        case 'R':
        case 'L':
        case 'S':
        manualControl(recieved_command);
        break;
      // Mode set
        case 'M': mode = 0;
        break;
        case 'A': mode = 1;
        break;
      // Speed change
        case 'l':
        case 'm':
        case 'h':
        setSpeed(recieved_command);
        break;
      // Set distanse to the wall
        case 's':
        assignSetPoint();
        break;

    }
  }
  if (millis()-pevTime >= time_to_send_data){
    Serial.write('d');
    Serial.print(String(getUltrasonicDistance(TRIG_SIDE, ECHO_SIDE)));
    Serial.write(' ');
    Serial.write('c');
    Serial.print(current());
    Serial.write(' ');
    Serial.write('v');
    Serial.print(voltRead());
    Serial.write(' ');
  }
  if(mode == 1)
    autonomousControl();
}

// Function for manual mode
void manualControl(char motion) {
    switch (motion) {
      case 'F': 
        moveForward(); 
        break;
      case 'B': 
        moveBackward();
        break;
      case 'R': 
        rotateRight(); 
        break;
      case 'L': 
        rotateLeft(); 
        break;
      case 'S': 
        stopAllMotors(); 
    }

}

// Function for autonomous mode
void autonomousControl() {
  // Get the distance from the side ultrasonic sensor 
  distance = getUltrasonicDistance(TRIG_SIDE, ECHO_SIDE);

  // PID control equation
  unsigned int Now = millis();
  if (Now - Last_Time >= Sample_Time) {
    float error = setpoint - distance;
    integral += error * (Sample_Time / 1000.0);
    float derivative = (error - previous_error) / (Sample_Time / 1000.0);
    output = Kp * error + Ki * integral + Kd * derivative;

    previous_error = error;
    Last_Time = Now;

    // Control motors based on PID output
    controlMotors(output);
  }
  delay(100); 
}
void assignSetPoint(){
  String distance = Serial.readStringUntil(' ');
  setpoint = distance.toFloat();
}
// Function to control the motors based on PID output
void controlMotors(float PIDOutput) {
  int speed = constrain(abs(PIDOutput), 0, motorSpeed);
  analogWrite(speedPin, speed);
  if (PIDOutput > 0) {
    rotateRight();
    delay(100);
  } else if (PIDOutput < 0) {
    rotateLeft();
    delay(100);
  }
  moveForward();
}

// Function to get distance from an ultrasonic sensor
float getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delay(1);
  digitalWrite(trigPin, HIGH);
  delay(1);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  // Measures the time length the ECHO_PIN stays HIGH.
  // This represents the time it takes for the ultrasonic pulse to travel to the wall and back.
  // pulseIn() measures time in microseconds

  return duration * 0.0344 / 2; 
  // Divided by 2 for the time of the pulse traveling to the wall and back.
  // The multiplication by 0.0344 converts the time into distance in cm (since sound travels at 0.0344 cm/µs).
}

// Function to check if obstacle is detected
bool obstacleDetected() {
  float frontDistance = getUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  return frontDistance < obstacleDistance;
}

// get volt reading

float voltRead() {

 vValue = analogRead(voltageReading);
 vValue = (vValue / 1023.0) *5.0; // arduino voltage reading
 vout = vValue *5.0; // described below
 // the number 5 was deduced by the voltage divider rule
 // for futher info about schematic check https://www.datasheethub.com/wp-content/uploads/2022/10/arduino-25v-voltage-sensor-module.pdf
  return vout;
}

// get current readings
float current() {
  cValue = analogRead(currentReading);
  vRead = cValue * 5.0 / 1023; 
  float currentRead = (vRead-2.5)/(sensitivity/1000); // changing voltage read to current read
  return currentRead;
}
// speed set with RGB
void setSpeed(char speed) {
  switch(speed) {
     case 'l':
      analogWrite(speedPin,90);
      digitalWrite(greenPin,HIGH);
      digitalWrite(bluePin,LOW);
      digitalWrite(redPin,LOW);
     break;

     case 'm':
      analogWrite(speedPin,190);
      digitalWrite(greenPin,LOW);
      digitalWrite(bluePin,HIGH);
      digitalWrite(redPin,LOW);
     break;

     case 'h':
      analogWrite(speedPin,255);
      digitalWrite(greenPin,LOW);
      digitalWrite(bluePin,LOW);
      digitalWrite(redPin,HIGH);
      break;
   }
}
// Manual control functions

void moveForward() { 
   motion = 0;

   digitalWrite(motorB2,HIGH);
   digitalWrite(motorB1,LOW);
   digitalWrite(motorFR2,HIGH);
   digitalWrite(motorFR1,LOW);
   digitalWrite(motorFL2,HIGH);
   digitalWrite(motorFL1,LOW);
}

void moveBackward() {
   motion = 1;
   digitalWrite(motorB2,LOW);
   digitalWrite(motorB1,HIGH);
   digitalWrite(motorFR2,LOW);
   digitalWrite(motorFR1,HIGH);
   digitalWrite(motorFL2,LOW);
   digitalWrite(motorFL1,HIGH);
} 

void rotateRight() {
   digitalWrite(motorFR1,LOW);
   digitalWrite(motorFR2,LOW);
   digitalWrite(motorFL1,motion);
   digitalWrite(motorFL2,!motion);
}

void rotateLeft() {
   digitalWrite(motorFR1,motion);
   digitalWrite(motorFR2,!motion);
   digitalWrite(motorFL1,LOW);
   digitalWrite(motorFL2,LOW);
}

void stopAllMotors() {
   digitalWrite(motorB1,LOW);
   digitalWrite(motorB2,LOW);
   digitalWrite(motorFR1,LOW);
   digitalWrite(motorFR2,LOW);
   digitalWrite(motorFL1,LOW);
   digitalWrite(motorFL2,LOW);
  
   motion = 0; // To make rotating front wheels rotate in forward direction as default after stopping
}
