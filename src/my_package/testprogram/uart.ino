

#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Serial configuration
#define BAUD_RATE 115200

// Servo objects (up to 6 joints)
Servo servos[6];

// Servo pin assignments - CHANGE THESE TO MATCH YOUR WIRING
const int servoPins[6] = {13, 12, 14, 27, 26, 25};

// Current joint angles
float currentAngles[6] = {90, 90, 90, 90, 90, 90};

String inputBuffer = "";

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Wait for serial connection
  delay(2000);
  
  // Attach servos
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);  // Center all servos
    currentAngles[i] = 90;
  }
  
  Serial.println("ESP32 Joint Controller Ready");
  Serial.println("Waiting for joint commands from Raspberry Pi...");
  Serial.println("Pin assignments:");
  for (int i = 0; i < 6; i++) {
    Serial.print("  J");
    Serial.print(i + 1);
    Serial.print(" -> GPIO ");
    Serial.println(servoPins[i]);
  }
}

void loop() {
  // Read incoming serial data
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      // Process complete command
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void processCommand(String jsonStr) {
  Serial.println("---");
  Serial.print("Received: ");
  Serial.println(jsonStr);
  
  // Parse JSON command
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    Serial.println("ERROR");
    return;
  }
  
  // Extract fields
  const char* action = doc["action"] | "unknown";
  int speed = doc["speed"] | 30;
  JsonObject joints = doc["joints"];
  
  Serial.print("Action: ");
  Serial.print(action);
  Serial.print(" | Speed: ");
  Serial.print(speed);
  Serial.println("%");
  
  // Execute joint movements
  if (joints.size() > 0) {
    moveJoints(joints, speed);
  } else {
    Serial.println("No joint data received");
  }
  
  // Send acknowledgment
  Serial.println("OK");
}

void moveJoints(JsonObject joints, int speed) {
  // Extract joint angles
  float targetAngles[6];
  bool hasTarget[6] = {false};
  
  // Parse J1-J6 from JSON
  if (joints.containsKey("J1")) {
    targetAngles[0] = joints["J1"];
    hasTarget[0] = true;
  }
  if (joints.containsKey("J2")) {
    targetAngles[1] = joints["J2"];
    hasTarget[1] = true;
  }
  if (joints.containsKey("J3")) {
    targetAngles[2] = joints["J3"];
    hasTarget[2] = true;
  }
  if (joints.containsKey("J4")) {
    targetAngles[3] = joints["J4"];
    hasTarget[3] = true;
  }
  if (joints.containsKey("J5")) {
    targetAngles[4] = joints["J5"];
    hasTarget[4] = true;
  }
  if (joints.containsKey("J6")) {
    targetAngles[5] = joints["J6"];
    hasTarget[5] = true;
  }
  
  // Calculate delay based on speed (higher speed = less delay)
  int delayMs = map(speed, 0, 100, 50, 5);
  
  // Find maximum movement distance
  int maxSteps = 0;
  for (int i = 0; i < 6; i++) {
    if (hasTarget[i]) {
      int steps = abs(targetAngles[i] - currentAngles[i]);
      if (steps > maxSteps) maxSteps = steps;
    }
  }
  
  // Smooth coordinated movement
  Serial.print("Moving servos: ");
  if (maxSteps > 0) {
    for (int step = 0; step <= maxSteps; step++) {
      for (int i = 0; i < 6; i++) {
        if (hasTarget[i]) {
          // Linear interpolation
          float progress = (float)step / maxSteps;
          float newAngle = currentAngles[i] + 
                          (targetAngles[i] - currentAngles[i]) * progress;
          
          // Constrain to servo limits
          newAngle = constrain(newAngle, 0, 180);
          servos[i].write((int)newAngle);
        }
      }
      delay(delayMs);
    }
    
    // Update current angles and print final positions
    for (int i = 0; i < 6; i++) {
      if (hasTarget[i]) {
        currentAngles[i] = constrain(targetAngles[i], 0, 180);
        Serial.print("J");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(currentAngles[i], 0);
        Serial.print("° ");
      }
    }
    Serial.println();
  } else {
    Serial.println("No movement needed");
  }
}