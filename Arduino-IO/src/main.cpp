#include <Arduino.h>
#include <WiFi.h>
#include "PINOUT.h"
#include "SerialMessage.h"
#include "DiffDrive.h"

// wifi name nad password
const char* ssid = "ESP32-MiniBot";
const char* password = "RobotsRule123";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Sonar sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
//Servo servo;

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// Incriment / Decrement depending on encoder state during an interrupt
void leftEncoderInc(){
  if (digitalRead(LEFT_ENC_A_PIN) == digitalRead(LEFT_ENC_B_PIN)) {
    leftEncoderCount++;
    return;
  }
  leftEncoderCount--;
  
}

// Incriment / Decrement depending on encoder state during an interrupt
void rightEncoderInc(){
  if (digitalRead(RIGHT_ENC_A_PIN) == digitalRead(RIGHT_ENC_B_PIN)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

Motor leftMotor(LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACK_PIN, LEFT_MOTOR_PWM_PIN, 0,  &leftEncoderCount);
Motor rightMotor(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACK_PIN, RIGHT_MOTOR_PWM_PIN, 1, &rightEncoderCount);
DiffDrive wheels(&leftMotor, &rightMotor, 165); //TODO: Change this to the actual wheel separation

// Object to handle serial communication
SerialMessage ser;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");

  wheels.begin();
  wheels.setPID(3, 100, -3);
  // // attach the interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncoderInc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderInc, CHANGE);

  // servo.attach(servo_pin);
  // servo.write(90);
  // sonar.attachServo(servo);
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();

  // sonar.enableScanMode(false);
  Serial.println("Started");
}

// TODO: Finish writing this function
void doSerialCommand(int * args, int args_length) {
  switch (args[0]) {
    case MOTOR_READ:{
      Serial.print("!MTR,");
      // print the current pose
      float *pose = (wheels.getCurrentPose());
      for(auto i = 0; i < 3; i++) {
        Serial.print(pose[i],4);
        Serial.print(",");
      }
      // print the target pose
      pose = (wheels.getTargetPose());
      for(auto i = 0; i < 3; i++) {
        Serial.print(pose[i],4);
        Serial.print(",");
      }

      Serial.println(";");
      break;
    }
    case MOTOR_WRITE:{
      if(args_length < 3) break;
      Serial.print("!MTR_WRT,");
      for(int i = 1; i < args_length; i++) {
        Serial.print(args[i]);
        Serial.print(",");
      }
      Serial.println(";");
      // set the new target pose
      wheels.setTargetPose(args[1], args[2], args[3]);
      //wheels.print();
      break;
    }
    // case SONAR_READ:{ 
    //   Serial.print("!SNR,");
    //   Serial.print(sonar.getAngleIncrement());
    //   sonar.print();
    //   Serial.println(";");
    //   break;
    // }
    // case SONAR_WRITE:{
    //   if(args_length < 2) break;
    //   sonar.enableScanMode(args[1]==1);
    //   sonar.setAngleIncrement(args[2]);
    //   Serial.print("SNR,");
    //   Serial.print(args[1]==1);
    //   Serial.print(",");
    //   Serial.print(args[2]);
    //   Serial.println(";");
    //   break;
    // }
    case IR_READ:{
      Serial.println("IR_READ");
      break;
    }
    default:{
      Serial.println("ERR");
      break;
    }
  }
}

void updateWifi(){
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait until the client sends some data
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();

  client.println("<!DOCTYPE html><html>");
  client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");

  // Web Page Heading
  client.println("<body><h1>Welcome to MiniBot. This is still WIP</h1>");
  // display the current state of the robot
  client.print("Current Pose: ");
  float *pose = (wheels.getCurrentPose());
  for(auto i = 0; i < 3; i++) {
    client.print(pose[i],4);
    client.print(",");
  }
  client.println("<br>");
  client.print("Target Pose: ");
  pose = (wheels.getTargetPose());
  for(auto i = 0; i < 3; i++) {
    client.print(pose[i],4);
    client.print(",");
  }
  client.println("<br>");
  client.println("</head>");
  client.println("<body>");
  client.println("<h1>Robot Control</h1>");
  client.println("<form method=\"get\" action=\"/motor\">");
  client.println("dX Position: <input type=\"text\" name=\"speed\"><br>");
  client.println("dY Position: <input type=\"text\" name=\"speed\"><br>");
  client.println("dAngle: <input type=\"text\" name=\"angle\"><br>");
  client.println("<input type=\"submit\" value=\"Submit\">");
  client.println("</form>");
  client.println("</body></html>");

  // The HTTP response ends with another blank line
  client.println();

  // check if the submit button was pressed
  if(client.find("GET /motor?")) {
    // get the data out of the form once the user hits the submit button
    String data = client.readStringUntil('\r');
    int dx = data.substring(data.indexOf("speed=") + 6, data.indexOf("&")).toInt();
    int dy = data.substring(data.indexOf("speed=", data.indexOf("speed=") + 6) + 6, data.indexOf("&", data.indexOf("&") + 1)).toInt();
    int dangle = data.substring(data.indexOf("angle=", data.indexOf("angle=") + 6) + 6, data.indexOf("&", data.indexOf("&", data.indexOf("&") + 1) + 1)).toInt();
    Serial.print("X: ");
    Serial.println(dx);
    Serial.print("Y: ");
    Serial.println(dy);
    Serial.print("Angle: ");
    Serial.println(dangle);
    if(dx == 0 && dy == 0 && dangle == 0) {
      client.stop();
      return;
    }
    // set the new target pose
    wheels.setTargetPose(pose[0] + dx, pose[1] + dy, pose[2] + dangle);
    //wheels.print();
  }
}

unsigned long timer = 0;

void loop() {
  ser.update();
  if (ser.isNewData()) {
    int * args = ser.getArgs();
    int args_length = ser.getPopulatedArgs();

    // ser.printArgs();

    doSerialCommand(args, args_length);
    ser.clearNewData();
  }
  wheels.update();
  updateWifi();
  // if (millis() - timer < 5000) {
  //   wheels.update();
  // }
  // else{
  //   leftMotor.setVelocity(0);
  //   rightMotor.setVelocity(0);
  // }
  


  //sonar.update();
}
