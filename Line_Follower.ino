/*
  8-Channel Analog IR Line Follower (Complete)
  Sensor: Smartelex RLS08 (Using ALL 8 sensors: A0-A7)
  Motor Driver: TB6612FNG
  Controller: Arduino Nano
  
  *** HARDWARE ***
  - Sensors 0-7 -> A0-A7
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
#define button4 11  // Auto Calibration

// --- LEDs ---
#define led13 13
#define led12 12

// --- Sensor Settings ---
#define sensorNumber 8
int sensor[sensorNumber];      
int minValues[sensorNumber];   
int maxValues[sensorNumber];   
int threshold[sensorNumber];   

// --- Tuning Parameters (TUNE THESE) ---
int kp = 35;              
int kd = 280;             
int max_speed = 230;      
int left_motor_speed = 180;   
int right_motor_speed = 180;  
int turn_speed = 160;         

// Delays to debounce noise at intersections
#define stop_timer 30         

// --- PID Variables ---
// Center of 8 sensors (0-7) is 3.5
float center_point = 3.5;     
float calculated_pos;
float error, previous_error;
int PID;
int turn_value = 0; // 0=Straight, 1=Left, 2=Right

// --- Button States ---
bool button1_state = 1, button2_state = 1, button3_state = 1, button4_state = 1;
bool calibrated = false;

void setup() {
  // Motor Pins
  pinMode(lmf, OUTPUT); pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT); pinMode(rmb, OUTPUT);
  pinMode(rms, OUTPUT); pinMode(lms, OUTPUT);

  // Sensor Pins (Analog)
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
  Serial.println("=== 8-Sensor Line Follower Ready ===");
}

void loop() {
  button_status();

  // Button 4: Auto Calibration
  if (button4_state == LOW) {
    calibrate_sensors(); // This now spins the bot
    // Post-calibration sensor test loop
    while(button4_state == LOW) {
      sensor_test();
      button_status();
    }
  }

  // Button 1: Start Line Follow
  if (button1_state == LOW) {
    if (!calibrated) {
      Serial.println("ERROR: Press Button 4 to Calibrate first!");
      for(int k=0; k<5; k++) { digitalWrite(led13, HIGH); delay(100); digitalWrite(led13, LOW); delay(100); }
    } else {
      Line_Follow();
    }
  }

  // Button 2: Motor Test
  if (button2_state == LOW) {
    Serial.println("Motor Test: Spinning Forward");
    motor(120, 120);
    delay(1000);
    motor(0, 0);
    delay(500);
  }
}

void button_status() {
  button1_state = digitalRead(button1);
  button2_state = digitalRead(button2);
  button3_state = digitalRead(button3);
  button4_state = digitalRead(button4);
}

// --- Auto Calibration (Spins 360) ---
void calibrate_sensors() {
  Serial.println("\n=== AUTO CALIBRATION START ===");
  digitalWrite(led12, HIGH);
  
  // Initialize min/max
  for(int i = 0; i < sensorNumber; i++) {
    minValues[i] = 1023; 
    maxValues[i] = 0;    
  }

  // Start Spinning
  int cal_speed = 130; // Adjust if bot spins too fast/slow
  motor(cal_speed, -cal_speed); 

  unsigned long startTime = millis();
  // Spin for 4 seconds
  while (millis() - startTime < 4000) {
    for(int i = 0; i < sensorNumber; i++) {
      int val = analogRead(A0 + i);
      if (val > maxValues[i]) maxValues[i] = val;
      if (val < minValues[i]) minValues[i] = val;
    }
  }

  motor(0, 0);
  digitalWrite(led12, LOW);

  // Calculate Thresholds
  for(int i = 0; i < sensorNumber; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }

  calibrated = true;
  Serial.println("Calibration Complete.");
}

// --- Sensor Reading & Normalization ---
void read_sensor() {
  int raw_reading;
  float weighted_sum = 0;
  int active_sensors = 0;
  
  for (int i = 0; i < sensorNumber; i++) {
    raw_reading = analogRead(A0 + i);
    
    // Normalize to 0-1000
    sensor[i] = map(raw_reading, minValues[i], maxValues[i], 0, 1000);
    sensor[i] = constrain(sensor[i], 0, 1000);
    
    weighted_sum += (float)sensor[i] * i;
    active_sensors += sensor[i];
  }
  
  // Avoid division by zero
  if (active_sensors > 100) {
    calculated_pos = weighted_sum / active_sensors;
  }
}

// --- Debugging Display ---
void sensor_test() {
  read_sensor();
  Serial.print("Pos: "); Serial.print(calculated_pos);
  Serial.print(" | ");
  for (int i = 0; i < sensorNumber; i++) {
    Serial.print(sensor[i]); Serial.print("\t");
  }
  Serial.println();
  delay(100);
}

// --- Motor Drive ---
void motor(int left, int right) {
  if (right > 0) { digitalWrite(rmf, HIGH); digitalWrite(rmb, LOW); }
  else { right = -right; digitalWrite(rmf, LOW); digitalWrite(rmb, HIGH); }

  if (left > 0) { digitalWrite(lmf, HIGH); digitalWrite(lmb, LOW); }
  else { left = -left; digitalWrite(lmf, LOW); digitalWrite(lmb, HIGH); }

  left = constrain(left, 0, max_speed);
  right = constrain(right, 0, max_speed);

  analogWrite(lms, left);
  analogWrite(rms, right);
}

// --- MAIN ALGORITHM ---
void Line_Follow() {
  Serial.println("LINE FOLLOWING STARTED");
  digitalWrite(led13, HIGH);
  
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
    if (sensor[0] > 600 && sensor[7] < 400) {
      turn_value = 1; 
    }
    // Rightmost sensor (7) sees black -> Remember Right Turn
    if (sensor[7] > 600 && sensor[0] < 400) {
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
        motor(-turn_speed, turn_speed); // Spin Left
        // Wait until middle sensors (3 or 4) see the line
        do {
          read_sensor();
        } while (sensor[3] < 500 && sensor[4] < 500); 
        turn_value = 0;
      }
      
      // Case B: Sharp Right (based on memory)
      else if (turn_value == 2) { 
        motor(turn_speed, -turn_speed); // Spin Right
        do {
          read_sensor();
        } while (sensor[3] < 500 && sensor[4] < 500);
        turn_value = 0;
      }
      
      // Case C: U-Turn (Dead End)
      else { 
        motor(turn_speed, -turn_speed); // Spin Right (default)
        do {
          read_sensor();
        } while (sensor[3] < 500 && sensor[4] < 500);
      }
    }
    
    // 5. Case: Stop Condition (All Black / T-intersection)
    // Adjust logic depending on if you want to stop or cross T-intersections.
    // Here we check if *most* sensors are black.
    else if (sensor_sum >= 6) {
      delay(stop_timer);
      read_sensor();
      
      // Re-count to confirm it wasn't noise
      sensor_sum = 0;
      for(int i = 0; i < sensorNumber; i++) {
        if(sensor[i] > 500) sensor_sum++;
      }
      
      // If still all black, STOP
      if(sensor_sum >= 6) {
        motor(0, 0);
        Serial.println("STOP LINE DETECTED");
        digitalWrite(led13, LOW);
        
        // Wait here until line is cleared or just hang forever
        while(sensor_sum >= 6) {
          read_sensor();
           sensor_sum = 0;
           for(int i = 0; i < sensorNumber; i++) if(sensor[i]>500) sensor_sum++;
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
