#include <SoftwareSerial.h> // libraries which are imported to the Sketch
#include <Servo.h>  // makes the servo Rudders such as turn R and L
#include <TinyGPS++.h> // coordinates with the GPS module

// --- Pin Map ---
const int bluetoothRX = 2; 
const int bluetoothTX = 3;
const int gpsRX = A0;        
const int gpsTX = A1;        
const int motorPin1 = 5;     
const int motorPin2 = 6;     
const int servoPin = 11;     
const int trigPin = 12;      
const int echoPin = 13;      
const int buzzerPin = 10; 

SoftwareSerial bluetooth(bluetoothRX, bluetoothTX);
SoftwareSerial gpsSerial(gpsRX, gpsTX);
TinyGPSPlus gps;
Servo rudder;

// --- Variables ---
int motorSpeed = 100;       // initial DC motor speed is initialized
bool isMoving = false;      
bool autoPilot = false;     //  making sure autopilot is not triggered as soon after the switch turned on
double targetLat = 0;       // variable used for the autopilot mode to coordinate the destination , these values are saved when H is pressed witht the controller
double targetLng = 0;     // double variable type used for higher precision than bool or int

void setup() {
  Serial.begin(9600); // 9600 is a timer for blaud rate - the speed at which the uno communicates with other components its the standard speed
  bluetooth.begin(9600);
  gpsSerial.begin(9600);
  
  pinMode(motorPin1, OUTPUT); // pin mapping of every components used to output from inputs
  pinMode(motorPin2, OUTPUT);  
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  pinMode(buzzerPin, OUTPUT);
  
  rudder.attach(servoPin);  // servo ideal angle 
  rudder.write(90);     
  
  // Startup beep
  digitalWrite(buzzerPin, HIGH); delay(100); digitalWrite(buzzerPin, LOW);
  
  bluetooth.listen(); 
  Serial.println("--- System Online ---");
}
// this is the priority LOOP 
void loop() {
  // 1. PRIORITY: ULTRASONIC SAFETY
  if (isMoving) {
    long distance = getDistance();
    if (distance > 2 && distance < 25) { 
      emergencyStop("OBSTACLE");
    }
  }

  // 2. PRIORITY: BLUETOOTH COMMANDS
  if (bluetooth.available() > 0) {
    executeCommand(bluetooth.read());
  }

  // 3. PRIORITY: GPS & AUTOPILOT
  if (autoPilot && isMoving) {
    updateGPS();      
    driveToTarget();  
  }
}

void executeCommand(char command) {
  // Confirmation beep
  digitalWrite(buzzerPin, HIGH); delay(30); digitalWrite(buzzerPin, LOW);

  switch (command) {  // Forward operation
    case 'F': 
      if (!isMoving) kickstart();  // kickstart for intial push due to resistance in the water ( only works for manual control)
      isMoving = true; 
      autoPilot = false; 
      forward(); 
      break;
    case 'S': 
      stopBoat(); 
      break;
    case 'L': rudder.write(40); break;
    case 'R': rudder.write(140); break; 
    case 'C': rudder.write(90); break;  
    case 'H': 
      updateGPS();  // nano starts listening to the GPS
      if (gps.location.isValid()) { // validates satellite 
        targetLat = gps.location.lat();
        targetLng = gps.location.lng();
        bluetooth.println("Target Saved!");
      } else {
        bluetooth.println("No GPS Lock!");
      }
      break;
    case 'G': 
      if (targetLat != 0) {
        autoPilot = true;
        isMoving = true;
        kickstart();
        forward();
        bluetooth.println("Autopilot ON");
      }
      break;
    case '+': motorSpeed = min(motorSpeed + 20, 255); if(isMoving) forward(); break;
    case '-': motorSpeed = max(motorSpeed - 20, 90); if(isMoving) forward(); break;
  }
}

// --- HELPER FUNCTIONS

void updateGPS() {  // at this point the arduino gahters as much as possible information from the GPS module for the next 80ms
  gpsSerial.listen(); 
  unsigned long start = millis();
  while (millis() - start < 80) { 
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  }
  bluetooth.listen(); 
}

void driveToTarget() {  // this method does all the calculations using the Haversine formula via the tinygps++ library
  if (!gps.location.isValid()) return;
  double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLng);
  
  if (dist < 4.0) { 
    emergencyStop("ARRIVED");
    return;
  }

  double courseTo = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLng); //this is calculated from the calculation which is the compass direction to the target
  double error = courseTo - gps.course.deg();  // this conditions determine the angle of servo to turn left or right
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  if (error > 15) rudder.write(140);  // if (error) is positive >15 the servo turns right (140)
  else if (error < -15) rudder.write(40); // if (error) is negative <-15 then servo turns left(40)
  else rudder.write(90); // when non of the previous conditions are met the servo turns to 90 
}

long getDistance() { // this method is used to find obstacles with the ultrasonic sensor
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); // burst of ultrasonic sound for 10microsec
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);  // pulsein measures how long it took for the sound to return
  if (duration == 0) return 999; // determine the area is clear (999)is distance to tell its safe to move
  return duration * 0.034 / 2; // the math of turning time to distance
}

void kickstart() { // this method is for kickstart the DC motor to overcome the resistane in water it puts is motor to maximum power 255 for the 250ms and then drops back to the initialized value (100)
  analogWrite(motorPin1, 255); // determines the speed - makes a voltage difference in both pins to let it spin
  digitalWrite(motorPin2, LOW); // determines the directions - flips the pins thousands of times every second
  delay(250); 
}

void forward() { // this method allows the motor to spin
  analogWrite(motorPin1, motorSpeed); 
  digitalWrite(motorPin2, LOW);
}

void stopBoat() {
  analogWrite(motorPin1, 0);
  digitalWrite(motorPin2, LOW);
  isMoving = false;
  autoPilot = false;
}

void emergencyStop(String reason) {
  stopBoat();
  digitalWrite(buzzerPin, HIGH);
  bluetooth.println("STOP: " + reason);
  delay(500);
  digitalWrite(buzzerPin, LOW);
}