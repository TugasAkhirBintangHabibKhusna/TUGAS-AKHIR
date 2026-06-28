#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial espSerial(10, 11);

// Deklarasi Servo
Servo servo1; // Servo pemilah warna merah
Servo servo2; // Servo pemilah warna biru
Servo servo3; // Servo pendorong (Kembali ke Uno)

// Deklarasi Pin Motor DC
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;

// Deklarasi Pin Servo
const int PIN_SERVO1 = 6;
const int PIN_SERVO2 = 9;
const int PIN_SERVO3 = 7;

// Deklarasi Pin Sensor Infrared
const int PIN_IR1 = 8;  // Sensor IR untuk warna merah
const int PIN_IR2 = 12; // Sensor IR untuk warna biru

// [TAMBAHAN] Deklarasi Pin Sensor Proximity Induktif (Logam)
const int PIN_PROX_LOGAM = A5;

// Deklarasi Pin Sensor Warna TCS3200
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int OUT_PIN = A4; 

const int THRESHOLD_FILTER = 15; 
const int JUMLAH_SAMPEL = 10; 

bool statusHMI = false; 
unsigned long waktuMulai = 0; 
unsigned long waktuDeteksiWarna = 0; // Timer untuk hitungan 5 detik setelah baca
int faseSistem = 0; 

bool servoAwal = true; 
bool menungguServo = false; // Penanda sedang menunggu waktu 5 detik untuk servo pendorong (servo3)

// Penanda menunggu sensor IR
bool tungguIR1 = false; // Menunggu IR1 untuk warna merah
bool tungguIR2 = false; // Menunggu IR2 untuk warna biru

// [TAMBAHAN] Penanda sistem sedang dalam siklus mendeteksi Logam
bool modeLogam = false; 

int r_val = 0;
int b_val = 0;
int warnaVal = 0;
int pwmMotor = 0;

void setup() {
  Serial.begin(9600);    
  espSerial.begin(9600); 
  
  // Inisialisasi Servo
  servo1.attach(PIN_SERVO1);
  servo1.write(180); // Standby di 180 derajat
  servo2.attach(PIN_SERVO2);
  servo2.write(180); // Standby di 180 derajat
  
  // [MODIFIKASI PRESISI] Batas minimum & maksimum mikrosekon khusus Servo 3 (500us - 2500us)
  servo3.attach(PIN_SERVO3, 500, 2500);
  servo3.write(10);   // Standby di 10 derajat
  
  // Inisialisasi Pin
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
  pinMode(PIN_PROX_LOGAM, INPUT); // [TAMBAHAN] Inisialisasi pin A5
  
  // Setting Skala Frekuensi Sensor Warna ke 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  // Setting Arah Awal Motor
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
      tungguIR1 = false; // Reset antrian IR
      tungguIR2 = false; // Reset antrian IR
      modeLogam = false; // [TAMBAHAN] Reset antrian logam saat HMI Off
    }
  }

  if (statusHMI == true) {
    // 1. GERAKAN AWAL (Untuk Servo Pendorong/Servo3)
    if (servoAwal) {
      analogWrite(ENA, 0); // [ATURAN] Motor berhenti saat Servo 3 bergerak
      servo3.write(158);
      delay(3000);         // Jeda 1 detik di posisi mendorong
      servo3.write(10);    // Kembali ke titik awal
      servoAwal = false;
    }

    // 2. CEK TIMER SERVO AKHIR (5 detik setelah deteksi warna untuk Servo3)
    if (menungguServo && (millis() - waktuDeteksiWarna >= 5000)) {
      analogWrite(ENA, 0); // [ATURAN] Motor berhenti saat Servo 3 bergerak
      servo3.write(158);
      delay(3000);         // Jeda 1 detik di posisi mendorong
      servo3.write(10);    // Kembali ke titik awal
      menungguServo = false; // Selesai
    }

    // 3. LOGIKA PEMBACAAN SENSOR INFRARED & SERVO PEMILAH
    // Cek IR 1 (Warna Merah) - Asumsi deteksi = LOW
    if (tungguIR1 && digitalRead(PIN_IR1) == LOW) {
      servo1.write(0);   // Bergerak ke 0
      delay(1000);
      servo1.write(180); // Kembali ke 180 (Standby)
      tungguIR1 = false; // Selesai memilah
    }

    // Cek IR 2 (Warna Biru) - Asumsi deteksi = LOW
    if (tungguIR2 && digitalRead(PIN_IR2) == LOW) {
      servo2.write(0);   // Bergerak ke 0
      delay(1000);
      servo2.write(180); // Kembali ke 180 (Standby)
      tungguIR2 = false; // Selesai memilah
    }

    // [TAMBAHAN] Cek IR 2 Khusus Mode Logam (Mengabaikan IR1 & Sensor Warna)
    if (modeLogam && digitalRead(PIN_IR2) == LOW) {
      delay(1000);         // Tunggu 1 detik barang sampai di depan pendorong
      analogWrite(ENA, 0); // [ATURAN] Motor berhenti saat Servo 3 bergerak
      servo3.write(158); 
      delay(3000);         // Jeda 1 detik di posisi mendorong
      servo3.write(10);    // Kembali ke titik awal
      modeLogam = false;   // Siklus logam selesai, kembali ke looping normal
    }

    // 4. LOGIKA FASE SISTEM & SENSOR WARNA
    if (faseSistem == 0) {
      
      // [TAMBAHAN] Prioritas Cek Logam di Fase 0 + KIRIM KODE 3 KE ESP
      if (digitalRead(PIN_PROX_LOGAM) == LOW && !modeLogam) {
        pwmMotor = 0;
        faseSistem = 1;
        waktuMulai = millis();
        modeLogam = true; // Mengunci sistem agar tidak membaca warna & IR1
        warnaVal = 3;     // <--- [KRISIAL] Agar angka 3 terkirim ke ESP32!
      }
      // Jika tidak ada logam, jalankan pembacaan warna normal
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
          waktuDeteksiWarna = millis(); // Kunci waktu deteksi di sini
          menungguServo = true;         // Aktifkan mode menunggu 5 detik (servo3)

          // Masukkan ke antrian tunggu sensor IR
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
  
  // Eksekusi putaran motor (Jika baru selesai jeda servo3, motor otomatis langsung menyala kembali di sini)
  analogWrite(ENA, pwmMotor); 

  espSerial.print(r_val); espSerial.print(","); espSerial.print(b_val); espSerial.print(","); espSerial.println(warnaVal);
  delay(100);
}
