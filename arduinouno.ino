#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial espSerial(10, 11);

Servo servo1; 
Servo servo2; 
Servo servo3; 

const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;

const int PIN_SERVO1 = 6;
const int PIN_SERVO2 = 9;
const int PIN_SERVO3 = 7;

const int PIN_IR1 = 8;  
const int PIN_IR2 = 12; 

const int PIN_PROX_LOGAM = A5;

const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int OUT_PIN = A4; 

const int THRESHOLD_FILTER = 15; 
const int JUMLAH_SAMPEL = 10; 

bool statusHMI = false; 
bool statusEmergency = false; 

unsigned long waktuMulai = 0; 
unsigned long waktuDeteksiWarna = 0; 
int faseSistem = 0; 

bool servoAwal = true; 
bool menungguServo = false; 

bool tungguIR1 = false; 
bool tungguIR2 = false; 

bool modeLogam = false; 

int r_val = 0;
int b_val = 0;
int warnaVal = 0;
int pwmMotor = 0;

int statServo1 = 0;
int statServo2 = 0;
int statServo3 = 0;
int statProx = 0;
int statIR1 = 0;
int statIR2 = 0;
int statMotor = 0;

unsigned long timerKirimData = 0;
unsigned long timerServoAwal = 0; bool isServoAwalAktif = false;
unsigned long timerServoAkhir = 0; bool isServoAkhirAktif = false;
unsigned long timerServo1 = 0; bool isServo1Aktif = false;
unsigned long timerServo2 = 0; bool isServo2Aktif = false;
unsigned long timerLogam = 0; int stateLogam = 0; 

void setup() {
  Serial.begin(9600);    
  espSerial.begin(9600); 
  
  servo1.attach(PIN_SERVO1); servo1.write(180); 
  servo2.attach(PIN_SERVO2); servo2.write(180); 
  servo3.attach(PIN_SERVO3, 500, 2500); servo3.write(10);   
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT_PIN, INPUT);
  
  pinMode(PIN_IR1, INPUT); pinMode(PIN_IR2, INPUT); pinMode(PIN_PROX_LOGAM, INPUT); 
  
  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
}

int bacaFrekuensiWarna(int s2_val, int s3_val) {
  digitalWrite(S2, s2_val); digitalWrite(S3, s3_val); delay(10); 
  long totalFrekuensi = 0;
  for (int i = 0; i < JUMLAH_SAMPEL; i++) totalFrekuensi += pulseIn(OUT_PIN, LOW, 20000); 
  return totalFrekuensi / JUMLAH_SAMPEL; 
}

void kirimDataKeESP() {
  statProx  = (digitalRead(PIN_PROX_LOGAM) == LOW) ? 1 : 0;
  statIR1   = (digitalRead(PIN_IR1) == LOW) ? 1 : 0;
  statIR2   = (digitalRead(PIN_IR2) == LOW) ? 1 : 0;
  statMotor = (pwmMotor > 0) ? 1 : 0;

  espSerial.print(r_val); espSerial.print(","); espSerial.print(b_val); espSerial.print(",");
  espSerial.print(warnaVal); espSerial.print(","); espSerial.print(statServo1); espSerial.print(",");
  espSerial.print(statServo2); espSerial.print(","); espSerial.print(statServo3); espSerial.print(",");
  espSerial.print(statProx); espSerial.print(","); espSerial.print(statIR1); espSerial.print(",");
  espSerial.print(statIR2); espSerial.print(","); espSerial.println(statMotor);
}

void loop() {
  while (espSerial.available()) {
    char inChar = espSerial.read();
    
    if (inChar == 'E') { 
      statusEmergency = true;
      statusHMI = false; 
      faseSistem = 0; 
      
      // --- PERUBAHAN: Kembalikan servo ke posisi awal layaknya tombol OFF ---
      servo1.write(180); 
      servo2.write(180); 
      servo3.write(10);
      
      pwmMotor = 0; 
      analogWrite(ENA, 0); 
      
      statServo1 = 0; statServo2 = 0; statServo3 = 0;
      menungguServo = false; tungguIR1 = false; tungguIR2 = false; modeLogam = false; 
      isServoAwalAktif = false; isServoAkhirAktif = false;
      isServo1Aktif = false; isServo2Aktif = false; stateLogam = 0;
    } 
    else if (inChar == 'C') { 
      statusEmergency = false; 
    }

    if (!statusEmergency) {
      if (inChar == '1' && !statusHMI) { 
        statusHMI = true; servoAwal = true; 
      }
      else if (inChar == '0' && statusHMI) { 
        statusHMI = false; faseSistem = 0; 
        servo1.write(180); servo2.write(180); servo3.write(10);
        statServo1 = 0; statServo2 = 0; statServo3 = 0;
        menungguServo = false; tungguIR1 = false; tungguIR2 = false; modeLogam = false; 
        isServoAwalAktif = false; isServoAkhirAktif = false;
        isServo1Aktif = false; isServo2Aktif = false; stateLogam = 0;
      }
    }
  }

  if (statusEmergency) {
    pwmMotor = 0; analogWrite(ENA, 0);
    r_val = 0; b_val = 0; warnaVal = 0;
  } 
  else {
    if (statusHMI == true) {
      if (servoAwal && !isServoAwalAktif) {
        statServo3 = 1; kirimDataKeESP(); analogWrite(ENA, 0); servo3.write(158);
        isServoAwalAktif = true; timerServoAwal = millis(); 
      }
      if (isServoAwalAktif && (millis() - timerServoAwal >= 3000)) {
        servo3.write(10); statServo3 = 0; kirimDataKeESP(); servoAwal = false; isServoAwalAktif = false;
      }

      if (menungguServo && !isServoAkhirAktif && (millis() - waktuDeteksiWarna >= 5000)) {
        statServo3 = 1; kirimDataKeESP(); analogWrite(ENA, 0); servo3.write(158);
        isServoAkhirAktif = true; timerServoAkhir = millis(); 
      }
      if (isServoAkhirAktif && (millis() - timerServoAkhir >= 3000)) {
        servo3.write(10); statServo3 = 0; kirimDataKeESP(); menungguServo = false; isServoAkhirAktif = false;
      }

      if (tungguIR1 && !isServo1Aktif && digitalRead(PIN_IR1) == LOW) {
        statServo1 = 1; kirimDataKeESP(); servo1.write(0); isServo1Aktif = true; timerServo1 = millis(); 
      }
      if (isServo1Aktif && (millis() - timerServo1 >= 1000)) {
        servo1.write(180); statServo1 = 0; kirimDataKeESP(); tungguIR1 = false; isServo1Aktif = false;
      }

      if (tungguIR2 && !isServo2Aktif && digitalRead(PIN_IR2) == LOW) {
        statServo2 = 1; kirimDataKeESP(); servo2.write(0); isServo2Aktif = true; timerServo2 = millis(); 
      }
      if (isServo2Aktif && (millis() - timerServo2 >= 1000)) {
        servo2.write(180); statServo2 = 0; kirimDataKeESP(); tungguIR2 = false; isServo2Aktif = false;
      }

      if (modeLogam && stateLogam == 0 && digitalRead(PIN_IR2) == LOW) {
        stateLogam = 1; timerLogam = millis(); 
      }
      if (stateLogam == 1 && (millis() - timerLogam >= 1000)) {
        statServo3 = 1; kirimDataKeESP(); analogWrite(ENA, 0); servo3.write(158); stateLogam = 2; timerLogam = millis(); 
      }
      if (stateLogam == 2 && (millis() - timerLogam >= 3000)) {
        servo3.write(10); statServo3 = 0; kirimDataKeESP(); modeLogam = false; stateLogam = 0;
      }

      if (faseSistem == 0) {
        if (digitalRead(PIN_PROX_LOGAM) == LOW && !modeLogam) {
          pwmMotor = 0; faseSistem = 1; waktuMulai = millis(); modeLogam = true; warnaVal = 3;     
        }
        else if (!modeLogam) {
          int r_freq = bacaFrekuensiWarna(LOW, LOW);   
          int b_freq = bacaFrekuensiWarna(LOW, HIGH);  
          r_val = constrain(map(r_freq, 120, 25, 0, 100), 0, 100);
          b_val = constrain(map(b_freq, 120, 25, 0, 100), 0, 100);

          if (r_val > b_val && (r_val - b_val) > THRESHOLD_FILTER) warnaVal = 1;      
          else if (b_val > r_val && (b_val - r_val) > THRESHOLD_FILTER) warnaVal = 2; 
          else warnaVal = 0;
          
          if (warnaVal == 1 || warnaVal == 2) {
            pwmMotor = 0; faseSistem = 1; waktuMulai = millis(); waktuDeteksiWarna = millis(); menungguServo = true;         
            if (warnaVal == 1) tungguIR1 = true; else if (warnaVal == 2) tungguIR2 = true;
          } else pwmMotor = 200; 
        }
      }
      else if (faseSistem == 1) {
        pwmMotor = 0;
        if (millis() - waktuMulai >= 1000) { faseSistem = 2; waktuMulai = millis(); warnaVal = 0; pwmMotor = 200; }
      }
      else if (faseSistem == 2) {
        pwmMotor = 200; r_val = 0; b_val = 0; warnaVal = 0;
        if (millis() - waktuMulai >= 2000) faseSistem = 0; 
      }
    } else {
      pwmMotor = 0; r_val = 0; b_val = 0; warnaVal = 0;
    }
    
    if (isServoAwalAktif || isServoAkhirAktif || stateLogam == 2) pwmMotor = 0;
    analogWrite(ENA, pwmMotor); 
  }

  if (millis() - timerKirimData >= 100) {
    kirimDataKeESP();
    timerKirimData = millis();
  }
}
