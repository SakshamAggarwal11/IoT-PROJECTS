/*
  5-Channel Analog IR Line Follower (REVISED & CORRECTED)
  Sensor: Using 5 sensors (A0-A4)
  Motor Driver: TB6612FNG
  Controller: Arduino Nano
  
  *** HARDWARE ***
  - Sensors 0-4 -> A0-A4
  - TB6612FNG STBY -> Arduino 5V
  - Left Motor -> D3, D2, D5
  - Right Motor -> D7, D4, D6
*/

// --- Motor Pins ---
#define rmf 7   // Right Motor Forward
#define rmb 4   // Right Motor Backward
#define lmf 3   // Left Motor Forward
#define lmb 2   // Left Motor Backward
#define rms 6   // Right Speed (PWM)
#define lms 5   // Left Speed (PWM)

// --- Buttons ---
#define button1 8   // Start Line Following
#define button2 9   // Motor Direction Test
#define button3 10  // Stop
#define button4 11  // Sensor Test

// --- LEDs ---
#define led13 13
#define led12 12

// --- Sensor Settings (5 SENSORS) ---
#define sensorNumber 5
#define samples_to_average 5

int sensor[sensorNumber];
int raw_readings[sensorNumber][samples_to_average];

// *** THRESHOLDS - UPDATED ***
// Based on Serial Monitor: White is ~990, Black is ~180. 
// Midpoint threshold is set to 585.
int threshold[sensorNumber] = {585, 585, 585, 585, 585};

// --- Tuning Parameters ---
int kp = 20;              // Proportional gain
int kd = 100;             // Derivative gain
int max_speed = 230;      
int left_motor_speed = 180;   
int right_motor_speed = 180;  
int turn_speed = 160;         

#define stop_timer 30

// --- PID Variables ---
// Center of 5 sensors (0-4) is 2.0
float center_point = 2.0;     
float calculated_pos;
float error, previous_error;
int PID;
int turn_value = 0; // 0=Straight, 1=Left, 2=Right

// --- Button States ---
bool button1_state = 1, button2_state = 1, button3_state = 1, button4_state = 1;

void setup() {
  // Motor Pins
  pinMode(lmf, OUTPUT); pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT); pinMode(rmb, OUTPUT);
  pinMode(rms, OUTPUT); pinMode(lms, OUTPUT);

  // Sensor Pins (A0-A4 only)
  for(int i = 0; i < sensorNumber; i++) {
    pinMode(A0 + i, INPUT);
  }

  // Buttons & LEDs
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(led13, OUTPUT);
  pinMode(led12, OUTPUT);

  Serial.begin(9600);
  Serial.println("=== 5-Sensor Line Follower Ready ===");
  Serial.println("Button 4: Sensor Test | Button 1: Line Follow | Button 2: Motor Test | Button 3: Stop");
}

void loop() {
  button_status();

  // Button 4: Sensor Test
  if (button4_state == LOW) {
    Serial.println("\n=== SENSOR TEST (5 Sensors) ===");
    Serial.println("Move bot over white and black line...\n");
    delay(500);
    
    while(button4_state == LOW) {
      sensor_test_display();
      button_status();
    }
    delay(300);
  }

  // Button 1: Start Line Follow
  if (button1_state == LOW) {
    Line_Follow();
    delay(300);
  }

  // Button 2: Motor Test
  if (button2_state == LOW) {
    Serial.println("Motor Test: Forward 1 second");
    motor(120, 120);
    delay(1000);
    motor(0, 0);
    delay(500);
    delay(300);
  }
}

void button_status() {
  button1_state = digitalRead(button1);
  button2_state = digitalRead(button2);
  button3_state = digitalRead(button3);
  button4_state = digitalRead(button4);
}

// --- Read Sensors with Averaging & Filtering ---
void read_sensor() {
  float weighted_sum = 0;
  int active_sensors = 0;
  
  for (int i = 0; i < sensorNumber; i++) {
    // Take multiple samples and average
    int sum = 0;
    for(int s = 0; s < samples_to_average; s++) {
      sum += analogRead(A0 + i);
      delayMicroseconds(50);
    }
    int avg_reading = sum / samples_to_average;
    
    // Apply threshold: Black = 1000, White = 0
    if(avg_reading < threshold[i]) {
      sensor[i] = 1000;  // Black line detected
    } else {
      sensor[i] = 0;     // White surface
    }
    
    weighted_sum += (float)sensor[i] * i;
    active_sensors += sensor[i];
  }
  
  // Calculate position (0 = leftmost, 2.0 = center, 4 = rightmost)
  if (active_sensors > 0) {
    calculated_pos = weighted_sum / active_sensors;
  } else {
    calculated_pos = 2.0; // Default to center
  }
}

// --- Sensor Test Display ---
void sensor_test_display() {
  read_sensor();  // <-- FIX: Added this so calculated_pos updates before printing

  Serial.print("Pos: ");
  Serial.print(calculated_pos, 1);
  Serial.print(" | ");
  
  // Read raw values
  for(int i = 0; i < sensorNumber; i++) {
    int raw = analogRead(A0 + i);
    
    // Display as [B] for black or [W] for white
    if(raw < threshold[i]) {
      Serial.print("[B]");
    } else {
      Serial.print("[W]");
    }
    Serial.print(raw); Serial.print("\t");
  }
  Serial.println();
  delay(200);
}

// --- Motor Drive ---
void motor(int left, int right) {
  // Set direction for right motor
  if (right > 0) {
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  } else {
    right = -right;
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
  }

  // Set direction for left motor
  if (left > 0) {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  } else {
    left = -left;
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }

  // Constrain speeds
  left = constrain(left, 0, max_speed);
  right = constrain(right, 0, max_speed);

  // Set PWM speeds
  analogWrite(lms, left);
  analogWrite(rms, right);
}

// --- MAIN LINE FOLLOWING ALGORITHM ---
void Line_Follow() {
  Serial.println("\n=== LINE FOLLOWING STARTED ===");
  digitalWrite(led13, HIGH);
  
  previous_error = 0;
  
  while (1) {
    read_sensor();

    // 1. PID Calculation
    error = center_point - calculated_pos;
    PID = error * kp + kd * (error - previous_error);
    previous_error = error;

    int right_motor = right_motor_speed - PID;
    int left_motor = left_motor_speed + PID;

    motor(left_motor, right_motor);

    // 2. Turn Memory Logic
    // Leftmost sensor (0) sees black -> Remember Left Turn
    if (sensor[0] > 500 && sensor[4] < 500) {
      turn_value = 1; 
    }
    // Rightmost sensor (4) sees black -> Remember Right Turn
    if (sensor[4] > 500 && sensor[0] < 500) {
      turn_value = 2; 
    }

    // 3. Count Active Sensors
    int sensor_sum = 0;
    for(int i = 0; i < sensorNumber; i++) {
      if(sensor[i] > 500) sensor_sum++;
    }

    // 4. Case: Lost Line (All White)
    if (sensor_sum == 0) { 
      
      // Case A: Sharp Left (based on memory)
      if (turn_value == 1) { 
        Serial.println("LEFT TURN");
        motor(-turn_speed, turn_speed); // Spin Left
        // Wait until middle sensors see the line
        do {
          read_sensor();
        } while (sensor[1] < 500 && sensor[2] < 500 && sensor[3] < 500); 
        turn_value = 0;
      }
      
      // Case B: Sharp Right (based on memory)
      else if (turn_value == 2) { 
        Serial.println("RIGHT TURN");
        motor(turn_speed, -turn_speed); // Spin Right
        do {
          read_sensor();
        } while (sensor[1] < 500 && sensor[2] < 500 && sensor[3] < 500);
        turn_value = 0;
      }
      
      // Case C: U-Turn (Dead End / No Memory)
      else { 
        Serial.println("U-TURN");
        motor(turn_speed, -turn_speed); // Spin Right (default)
        do {
          read_sensor();
        } while (sensor[1] < 500 && sensor[2] < 500 && sensor[3] < 500);
      }
    }
    
    // 5. Case: Stop Condition (All Black / T-intersection)
    else if (sensor_sum >= 4) {
      delay(stop_timer);
      read_sensor();
      
      // Re-count to confirm it wasn't noise
      sensor_sum = 0;
      for(int i = 0; i < sensorNumber; i++) {
        if(sensor[i] > 500) sensor_sum++;
      }
      
      // If still all black, STOP
      if(sensor_sum >= 4) {
        motor(0, 0);
        Serial.println("*** STOP LINE DETECTED ***");
        digitalWrite(led13, LOW);
        
        // Wait here until line is cleared
        while(sensor_sum >= 4) {
          read_sensor();
          sensor_sum = 0;
          for(int i = 0; i < sensorNumber; i++) {
            if(sensor[i] > 500) sensor_sum++;
          }
        }
      }
    }
    
    // 6. Emergency Stop Button
    button_status();
    if(button3_state == LOW) {
      motor(0, 0);
      Serial.println("MANUAL STOP");
      digitalWrite(led13, LOW);
      return; // Exit function
    }
  }
}
