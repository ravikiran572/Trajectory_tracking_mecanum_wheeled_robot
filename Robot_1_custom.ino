// Variables
volatile int pulseCount1 = 0, pulseCount2 = 0, pulseCount3 = 0, pulseCount4 = 0;
unsigned long prevMillis = 0;
float rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
float filteredRPM1 = 0, filteredRPM2 = 0, filteredRPM3 = 0, filteredRPM4 = 0;
float dir1 =0, dir2 =0, dir3 =0, dir4 =0;
const int pulsesPerRevolution = 330;
float dt = 0.02;
float x_in = 0, y_in = 0, phi = 0;

// Kalman filter variables
float Q = 1, R = 1;
float x_k1 = 0, x_k2 = 0, x_k3 = 0, x_k4 = 0;
float P_k1 = 1, P_k2 = 1, P_k3 = 1, P_k4 = 1;

// PID control variables
float lx = 0.09, ly = 0.095, r = 0.04;
float kp = 10, ki = 0, kd = 0;
float prevError1 = 0, prevError2 = 0, prevError3 = 0, prevError4 = 0;
float integral1 = 0, integral2 = 0, integral3 = 0, integral4 = 0;

// ISR for encoders (without IRAM_ATTR)
void pulseCounter1() { pulseCount1++; }
void pulseCounter2() { pulseCount2++; }
void pulseCounter3() { pulseCount3++; }
void pulseCounter4() { pulseCount4++; }


// Kalman filter function
float kalmanFilterUpdate(float z_k, float &x_k, float &P_k) {
    P_k = P_k + Q;
    float K_k = P_k / (P_k + R);
    x_k = x_k + K_k * (z_k - x_k);
    P_k = (1 - K_k) * P_k;
    return x_k;
}

// PID control function
float computePID(float desiredRPM, float currentRPM, float &prevError, float &integral) {
    float error = desiredRPM - currentRPM;
    integral += error * dt;
    integral = constrain(integral, 0, 50);
    float derivative = (error - prevError) / dt;
    prevError = error;
    return constrain(kp * error + ki * integral + kd * derivative, 0, 255);
}

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);
    pinMode(4, OUTPUT); pinMode(5, OUTPUT);
    pinMode(6, OUTPUT); pinMode(7, OUTPUT);
    pinMode(9, OUTPUT); pinMode(10, OUTPUT);
    pinMode(11, OUTPUT); pinMode(12, OUTPUT);
    pinMode(30,OUTPUT); pinMode(31,OUTPUT);pinMode(32,OUTPUT);pinMode(33,OUTPUT);
    pinMode(34,OUTPUT); pinMode(35,OUTPUT);pinMode(36,OUTPUT);pinMode(37,OUTPUT);
    digitalWrite(30,LOW);
    digitalWrite(31,HIGH);
    digitalWrite(32,LOW);
    digitalWrite(33,HIGH);
    digitalWrite(34,LOW);
    digitalWrite(35,HIGH);
    digitalWrite(36,LOW);
    digitalWrite(37,HIGH);
    attachInterrupt(digitalPinToInterrupt(21), pulseCounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(20), pulseCounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(19), pulseCounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(18), pulseCounter4, RISING);
}

void loop() {
    unsigned long currentMillis = millis();
    float timeInSeconds = currentMillis / 1000.0;
    
    float x_des_in = 1*sin(2*PI*(0.1/PI)*timeInSeconds);
    float y_des_in = 1*cos(2*PI*(0.1/PI)*timeInSeconds);
    float phi_des_in = 0;
    
    float x_dot_des_r = cos(phi)*(x_des_in - x_in) + sin(phi)*(y_des_in - y_in);
    float y_dot_des_r = -sin(phi)*(x_des_in - x_in) + cos(phi)*(y_des_in - y_in);
    
    float Vx_des = 0.1*x_dot_des_r/(cos(phi));
    float Vy_des = 0.1*y_dot_des_r/(cos(phi));
    float w_des = phi_des_in - phi;

//    float Vx_des = 0.1*sin(2*PI*(0.05/PI)*timeInSeconds);
//    float Vy_des = 0.1*cos(2*PI*(0.05/PI)*timeInSeconds);
//    float w_des = 0;
    
    float desiredRPM1 = (1/r)*(Vx_des-Vy_des-(lx+ly)*w_des);
    float desiredRPM2 = (1/r)*(Vx_des+Vy_des+(lx+ly)*w_des);
    float desiredRPM3 = (1/r)*(Vx_des+Vy_des-(lx+ly)*w_des);
    float desiredRPM4 = (1/r)*(Vx_des-Vy_des+(lx+ly)*w_des);
    
    desiredRPM1 = (30/PI)*desiredRPM1, desiredRPM2 = (30/PI)*desiredRPM2;
    desiredRPM3 = (30/PI)*desiredRPM3, desiredRPM4 = (30/PI)*desiredRPM4;  
    
    if (currentMillis - prevMillis >= dt*1000) {
        detachInterrupt(digitalPinToInterrupt(18));
        detachInterrupt(digitalPinToInterrupt(19));
        detachInterrupt(digitalPinToInterrupt(20));
        detachInterrupt(digitalPinToInterrupt(21));

        rpm1 = (pulseCount1 / dt) / pulsesPerRevolution * 60;
        rpm2 = (pulseCount2 / dt) / pulsesPerRevolution * 60;
        rpm3 = (pulseCount3 / dt) / pulsesPerRevolution * 60;
        rpm4 = (pulseCount4 / dt) / pulsesPerRevolution * 60;

        filteredRPM1 = kalmanFilterUpdate(rpm1, x_k1, P_k1);
        filteredRPM2 = kalmanFilterUpdate(rpm2, x_k2, P_k2);
        filteredRPM3 = kalmanFilterUpdate(rpm3, x_k3, P_k3);
        filteredRPM4 = kalmanFilterUpdate(rpm4, x_k4, P_k4);

        pulseCount1 = pulseCount2 = pulseCount3 = pulseCount4 = 0;
        prevMillis = currentMillis;

        attachInterrupt(digitalPinToInterrupt(21), pulseCounter1, RISING);
        attachInterrupt(digitalPinToInterrupt(20), pulseCounter2, RISING);
        attachInterrupt(digitalPinToInterrupt(19), pulseCounter3, RISING);
        attachInterrupt(digitalPinToInterrupt(18), pulseCounter4, RISING);
    }

    float control1 = computePID(abs(desiredRPM1), filteredRPM1, prevError1, integral1);
    float control2 = computePID(abs(desiredRPM2), filteredRPM2, prevError2, integral2);
    float control3 = computePID(abs(desiredRPM3), filteredRPM3, prevError3, integral3);
    float control4 = computePID(abs(desiredRPM4), filteredRPM4, prevError4, integral4);

    if (desiredRPM1 > 0){
      analogWrite(4, control1);
      analogWrite(5, 0);
      dir1 = 1;
    } else {
      analogWrite(4, 0);
      analogWrite(5, control1);
      dir1 = -1;
    }
    if (desiredRPM2 > 0){
      analogWrite(6, control2);
      analogWrite(7, 0);
      dir2 = 1;
    } else {
      analogWrite(6, 0);
      analogWrite(7, control2);
      dir2 = -1;
    }
    if (desiredRPM3 > 0){
      analogWrite(9, control3);
      analogWrite(10, 0);
      dir3 = 1;
    } else {
      analogWrite(9, 0);
      analogWrite(10, control3);
      dir3 = -1;
    }
    if (desiredRPM4 > 0){
      analogWrite(11, control4);
      analogWrite(12, 0);
      dir4 = 1;
    } else {
      analogWrite(11, 0);
      analogWrite(12, control4);
      dir4 = -1;
    }
    filteredRPM1 = (PI/30)*filteredRPM1, filteredRPM2 = (PI/30)*filteredRPM2;
    filteredRPM3 = (PI/30)*filteredRPM3, filteredRPM4 = (PI/30)*filteredRPM4;
    
    float Vx = r*(dir1*filteredRPM1 + dir2*filteredRPM2 + dir3*filteredRPM3 + dir4*filteredRPM4)/4;
    float Vy = r*(-dir1*filteredRPM1 + dir2*filteredRPM2 + dir3*filteredRPM3 - dir4*filteredRPM4)/4;
    float w = r*(-dir1*filteredRPM1 + dir2*filteredRPM2 - dir3*filteredRPM3 + dir4*filteredRPM4)/(4*(lx + ly));

    float xdot = Vx*cos(phi);
    float ydot = Vy*cos(phi);
    float phidot = w;
    
    x_in = x_in + xdot*dt;
    y_in = y_in + ydot*dt;
    phi = phi + phidot*dt;

    // Serial output for debugging
    Serial3.println(String(x_in) + "," + String(y_in));
    Serial.println(String(x_in) + ","  + String(y_in));
    
    // Stop if close to target (within 5cm)
//    if(sqrt(x_error*x_error + y_error*y_error) < 0.05) {
//        // Stop all motors
//        analogWrite(4, 0); analogWrite(5, 0);
//        analogWrite(6, 0); analogWrite(7, 0);
//        analogWrite(9, 0); analogWrite(10, 0);
//        analogWrite(11, 0); analogWrite(12, 0);
//        while(1); // Infinite loop to stop execution
//    }
}
