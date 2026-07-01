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

// [TAMBAHAN BARU] Variabel Indikator Logika untuk dikirim ke ESP -> HMI
int statServo1 = 0;
int statServo2 = 0;
int statServo3 = 0;
int statProx = 0;
int statIR1 = 0;
int statIR2 = 0;
int statMotor = 0;

void setup() {
  Serial.begin(9600);    
  espSerial.begin(9600); 
  
  servo1.attach(PIN_SERVO1);
  servo1.write(180); 
  servo2.attach(PIN_SERVO2);
  servo2.write(180); 
  
  servo3.attach(PIN_SERVO3, 500, 2500);
  servo3.write(10);   
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT_PIN, INPUT);
  
  pinMode(PIN_IR1, INPUT);
  pinMode(PIN_IR2, INPUT);
  pinMode(PIN_PROX_LOGAM, INPUT); 
  
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

int bacaFrekuensiWarna(int s2_val, int s3_val) {
  digitalWrite(S2, s2_val);
  digitalWrite(S3, s3_val);
  delay(10); 
  long totalFrekuensi = 0;
  for (int i = 0; i < JUMLAH_SAMPEL; i++) {
    totalFrekuensi += pulseIn(OUT_PIN, LOW);
  }
  return totalFrekuensi / JUMLAH_SAMPEL; 
}

// [TAMBAHAN BARU] Fungsi standarisasi pengiriman data indikator ke ESP32
void kirimDataKeESP() {
  // Update status sensor instan secara real-time (Active LOW diubah menjadi High murni untuk HMI)
  statProx  = (digitalRead(PIN_PROX_LOGAM) == LOW) ? 1 : 0;
  statIR1   = (digitalRead(PIN_IR1) == LOW) ? 1 : 0;
  statIR2   = (digitalRead(PIN_IR2) == LOW) ? 1 : 0;
  statMotor = (pwmMotor > 0) ? 1 : 0;

  // Format transmisi serial: r,g,b, servo1, servo2, servo3, prox, ir1, ir2, motor
  espSerial.print(r_val); espSerial.print(",");
  espSerial.print(b_val); espSerial.print(",");
  espSerial.print(warnaVal); espSerial.print(",");
  espSerial.print(statServo1); espSerial.print(",");
  espSerial.print(statServo2); espSerial.print(",");
  espSerial.print(statServo3); espSerial.print(",");
  espSerial.print(statProx); espSerial.print(",");
  espSerial.print(statIR1); espSerial.print(",");
  espSerial.print(statIR2); espSerial.print(",");
  espSerial.println(statMotor);
}

void loop() {
  while (espSerial.available()) {
    char inChar = espSerial.read();
    if (inChar == '1') {
      statusHMI = true;  
      servoAwal = true; 
    } else if (inChar == '0') {
      statusHMI = false; 
      faseSistem = 0; 
      servo3.write(10);
      menungguServo = false;
      tungguIR1 = false; 
      tungguIR2 = false; 
      modeLogam = false; 
    }
  }

  if (statusHMI == true) {
    // 1. GERAKAN AWAL (Untuk Servo Pendorong/Servo3)
    if (servoAwal) {
      pwmMotor = 0;
      statServo3 = 1; // Set indikator aktif
      kirimDataKeESP(); // Kirim instan ke HMI sebelum delay

      analogWrite(ENA, 0); 
      servo3.write(158);
      delay(3000);         
      servo3.write(10);    
      
      statServo3 = 0; // Set indikator standby
      kirimDataKeESP(); 
      servoAwal = false;
    }

    // 2. CEK TIMER SERVO AKHIR (5 detik setelah deteksi warna untuk Servo3)
    if (menungguServo && (millis() - waktuDeteksiWarna >= 5000)) {
      pwmMotor = 0;
      statServo3 = 1; 
      kirimDataKeESP();

      analogWrite(ENA, 0); 
      servo3.write(158);
      delay(3000);         
      servo3.write(10);    

      statServo3 = 0; 
      kirimDataKeESP();
      menungguServo = false; 
    }

    // 3. LOGIKA PEMBACAAN SENSOR INFRARED & SERVO PEMILAH
    if (tungguIR1 && digitalRead(PIN_IR1) == LOW) {
      statServo1 = 1; // Servo 1 Aktif
      kirimDataKeESP();

      servo1.write(0);   
      delay(1000);
      servo1.write(180); 

      statServo1 = 0; // Servo 1 Selesai
      kirimDataKeESP();
      tungguIR1 = false; 
    }

    if (tungguIR2 && digitalRead(PIN_IR2) == LOW) {
      statServo2 = 1; // Servo 2 Aktif
      kirimDataKeESP();

      servo2.write(0);   
      delay(1000);
      servo2.write(180); 

      statServo2 = 0; // Servo 2 Selesai
      kirimDataKeESP();
      tungguIR2 = false; 
    }

    // Cek IR 2 Khusus Mode Logam
    if (modeLogam && digitalRead(PIN_IR2) == LOW) {
      delay(1000); 
      pwmMotor = 0;
      statServo3 = 1; // Servo 3 Aktif
      kirimDataKeESP();

      analogWrite(ENA, 0); 
      servo3.write(158); 
      delay(3000);         
      servo3.write(10);    

      statServo3 = 0; // Servo 3 Selesai
      kirimDataKeESP();
      modeLogam = false;   
    }

    // 4. LOGIKA FASE SISTEM & SENSOR WARNA
    if (faseSistem == 0) {
      if (digitalRead(PIN_PROX_LOGAM) == LOW && !modeLogam) {
        pwmMotor = 0;
        faseSistem = 1;
        waktuMulai = millis();
        modeLogam = true; 
        warnaVal = 3;     
      }
      else if (!modeLogam) {
        int r_freq = bacaFrekuensiWarna(LOW, LOW);   
        int b_freq = bacaFrekuensiWarna(LOW, HIGH);  
        
        r_val = constrain(map(r_freq, 120, 25, 0, 100), 0, 100);
        b_val = constrain(map(b_freq, 120, 25, 0, 100), 0, 100);

        if (r_val > b_val && (r_val - b_val) > THRESHOLD_FILTER) warnaVal = 1;      // Merah
        else if (b_val > r_val && (b_val - r_val) > THRESHOLD_FILTER) warnaVal = 2; // Biru
        else warnaVal = 0;
        
        if (warnaVal == 1 || warnaVal == 2) {
          pwmMotor = 0;
          faseSistem = 1; 
          waktuMulai = millis();
          waktuDeteksiWarna = millis(); 
          menungguServo = true;         

          if (warnaVal == 1) {
            tungguIR1 = true;
          } else if (warnaVal == 2) {
            tungguIR2 = true;
          }
          
        } else {
          pwmMotor = 200;
        }
      }
    }
    
    else if (faseSistem == 1) {
      pwmMotor = 0;
      if (millis() - waktuMulai >= 1000) {
        faseSistem = 2;        
        waktuMulai = millis(); 
        warnaVal = 0;          
        pwmMotor = 200;        
      }
    }
    
    else if (faseSistem == 2) {
      pwmMotor = 200; 
      r_val = 0; b_val = 0; warnaVal = 0;
      if (millis() - waktuMulai >= 2000) faseSistem = 0; 
    }
  } else {
    pwmMotor = 0; r_val = 0; b_val = 0; warnaVal = 0;
  }
  
  analogWrite(ENA, pwmMotor); 

  // Panggil fungsi rutin pengiriman berkala di akhir loop
  kirimDataKeESP();
  delay(100);
}
