#include <ModbusRTU.h>
#include <HX711.h> 
#include <Preferences.h> 

Preferences prefs; 

// =======================================================
// PEMETAAN ALAMAT REGISTER MODBUS
// =======================================================
#define REG_SENSOR_R    0  
#define REG_SENSOR_G    1  
#define REG_SENSOR_B    2  
#define REG_WARNA       3  
#define REG_ENCODER     4  
#define REG_BERAT_ASLI  5  
#define REG_BERAT_RESET 6  

#define REG_BOX_MERAH   7  
#define REG_BOX_BIRU    8  
#define REG_BOX_LOGAM   9  

// REGISTER INDIKATOR UNTUK HMI (4x10 - 4x16)
#define REG_IND_SERVO1   10  // Servo 1 Merah
#define REG_IND_SERVO2   11  // Servo 2 Biru
#define REG_IND_SERVO3   12  // Servo 3 Pendorong
#define REG_IND_PROX     13  // Sensor Proximity Induktif
#define REG_IND_IR1      14  // Sensor IR 1
#define REG_IND_IR2      15  // Sensor IR 2
#define REG_IND_MOTOR    16  // Motor DC Conveyor

// [TAMBAHAN LOGIKA BARU] REGISTER TRACKING BARANG
#define REG_SEQ_M1 17 // Merah start
#define REG_SEQ_M2 18 // Merah IR1
#define REG_SEQ_M3 19 // Merah Servo 1
#define REG_SEQ_B1 20 // Biru start
#define REG_SEQ_B2 21 // Biru IR1
#define REG_SEQ_B3 22 // Biru IR2
#define REG_SEQ_B4 23 // Biru Servo 2
#define REG_SEQ_L1 24 // Logam start
#define REG_SEQ_L2 25 // Logam IR1
#define REG_SEQ_L3 26 // Logam IR2

// PEMETAAN ALAMAT COIL (TOMBOL HMI)
#define REG_SAKLAR       0  
#define REG_SAKLAR_TARE  1  
#define REG_RESET_MERAH  2  
#define REG_RESET_BIRU   3  
#define REG_RESET_LOGAM  4  

// =======================================================
// PIN & SPESIFIKASI ENCODER & LOADCELL
// =======================================================
#define DE_RE_PIN 25
#define ENCODER_PIN_A 18 
#define ENCODER_PIN_B 19 
#define ENCODER_PPR 1600  

#define HX711_DOUT_PIN 22
#define HX711_SCK_PIN  23
HX711 scale;
float faktorKalibrasi = 501.3; 

ModbusRTU mb;

HardwareSerial SerialModbus(1);   
HardwareSerial SerialArduino(2);  

String inputString = "";
bool stringComplete = false;
bool lastSaklarState = false;

float beratAktualTerakhir = 0.0;
float offsetTareMonitor = 0.0;

volatile int32_t encoderCount = 0; 
unsigned long lastTime = 0;
const unsigned long interval = 500; 

float smoothedRpm = 0.0;
const float ALPHA = 0.50; 

uint16_t totalMerah = 0;
uint16_t totalBiru = 0;
uint16_t totalLogam = 0;
uint16_t warnaTerakhir = 0; 

// Variabel Tracking Sequence & Edge Detection
uint8_t seqMerah = 0;
uint8_t seqBiru = 0;
uint8_t seqLogam = 0;
int last_servo1 = 0;
int last_servo2 = 0;

void IRAM_ATTR readEncoder() {
  static uint8_t encoderState = 0;
  encoderState = (encoderState << 2) & 0x0F;
  encoderState |= (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);
  
  if (encoderState == 0b0001 || encoderState == 0b0111 || encoderState == 0b1110 || encoderState == 0b1000) {
    encoderCount++;
  } else if (encoderState == 0b0010 || encoderState == 0b1011 || encoderState == 0b1101 || encoderState == 0b0100) {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), readEncoder, CHANGE);

  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  scale.set_scale(faktorKalibrasi);

  SerialArduino.begin(9600, SERIAL_8N1, 16, 17);
  SerialModbus.begin(9600, SERIAL_8N1, 32, 33);

  mb.begin(&SerialModbus, DE_RE_PIN);
  mb.slave(1); 

  prefs.begin("hitungBox", false); 
  totalMerah = prefs.getUShort("merah", 0);
  totalBiru  = prefs.getUShort("biru", 0);
  totalLogam = prefs.getUShort("logam", 0);

  mb.addHreg(REG_SENSOR_R, 0);
  mb.addHreg(REG_SENSOR_G, 0);
  mb.addHreg(REG_SENSOR_B, 0);
  mb.addHreg(REG_WARNA, 0);
  mb.addHreg(REG_ENCODER, 0); 
  mb.addHreg(REG_BERAT_ASLI, 0);  
  mb.addHreg(REG_BERAT_RESET, 0); 
  mb.addHreg(REG_BOX_MERAH, totalMerah);   
  mb.addHreg(REG_BOX_BIRU, totalBiru);   
  mb.addHreg(REG_BOX_LOGAM, totalLogam);    

  // Daftarkan register indikator ke Modbus HMI
  mb.addHreg(REG_IND_SERVO1, 0);
  mb.addHreg(REG_IND_SERVO2, 0);
  mb.addHreg(REG_IND_SERVO3, 0);
  mb.addHreg(REG_IND_PROX, 0);
  mb.addHreg(REG_IND_IR1, 0);
  mb.addHreg(REG_IND_IR2, 0);
  mb.addHreg(REG_IND_MOTOR, 0);

  // Mendaftarkan register sekuens ke Modbus HMI
  mb.addHreg(REG_SEQ_M1, 0);
  mb.addHreg(REG_SEQ_M2, 0);
  mb.addHreg(REG_SEQ_M3, 0);
  mb.addHreg(REG_SEQ_B1, 0);
  mb.addHreg(REG_SEQ_B2, 0);
  mb.addHreg(REG_SEQ_B3, 0);
  mb.addHreg(REG_SEQ_B4, 0);
  mb.addHreg(REG_SEQ_L1, 0);
  mb.addHreg(REG_SEQ_L2, 0);
  mb.addHreg(REG_SEQ_L3, 0);

  mb.addCoil(REG_SAKLAR, false);
  mb.addCoil(REG_SAKLAR_TARE, false); 
  mb.addCoil(REG_RESET_MERAH, false); 
  mb.addCoil(REG_RESET_BIRU, false);  
  mb.addCoil(REG_RESET_LOGAM, false); 

  inputString.reserve(60); 
  Serial.println("ESP32 Ready - Indikator Modbus Aktif!");
}

void loop() {
  mb.task();  

  unsigned long currentTime = millis();

  // 1. KONTROL WAKTU KALKULASI RPM ENCODER
  if (currentTime - lastTime >= interval) {
    noInterrupts();
    int32_t count = encoderCount;
    encoderCount = 0; 
    interrupts();

    int16_t rawRpm = (int16_t)((count * 60000L) / (ENCODER_PPR * interval));

    if (rawRpm == 0) smoothedRpm = 0; 
    else smoothedRpm = (ALPHA * rawRpm) + ((1.0 - ALPHA) * smoothedRpm);

    int16_t finalRpm = (int16_t)round(smoothedRpm);
    mb.Hreg(REG_ENCODER, (uint16_t)finalRpm);

    lastTime = currentTime;
  }

  // 2. CEK TOMBOL UTAMA (0x0)
  bool currentSaklarState = mb.Coil(REG_SAKLAR);
  if (currentSaklarState != lastSaklarState) {
    if (currentSaklarState == true) SerialArduino.print('1');  
    else SerialArduino.print('0');  
    lastSaklarState = currentSaklarState;
  }

  // 3. BACA BALASAN DATA DARI ARDUINO
  while (SerialArduino.available()) {
    char inChar = (char)SerialArduino.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') { 
      inputString += inChar;
    }
  }

  if (stringComplete) {
    parseArduinoData(inputString);
    inputString = "";
    stringComplete = false;
  }

  // 4. CEK PEMICU RESET BERAT TIMBANGAN
  if (mb.Coil(REG_SAKLAR_TARE) == true) {
    offsetTareMonitor = beratAktualTerakhir;
    mb.Coil(REG_SAKLAR_TARE, false); 
  }

  // 5. RESET VARIABEL COUNTER 
  if (mb.Coil(REG_RESET_MERAH) == true) { 
    totalMerah = 0; 
    mb.Hreg(REG_BOX_MERAH, 0); 
    prefs.putUShort("merah", 0); 
    mb.Coil(REG_RESET_MERAH, false); 
  }
  if (mb.Coil(REG_RESET_BIRU) == true)  { 
    totalBiru = 0;  
    mb.Hreg(REG_BOX_BIRU, 0);  
    prefs.putUShort("biru", 0);  
    mb.Coil(REG_RESET_BIRU, false); 
  }
  if (mb.Coil(REG_RESET_LOGAM) == true) { 
    totalLogam = 0; 
    mb.Hreg(REG_BOX_LOGAM, 0); 
    prefs.putUShort("logam", 0); 
    mb.Coil(REG_RESET_LOGAM, false); 
  }

  // 6. BACA LOADCELL & UPDATE DISPLAY
  static unsigned long lastTimerLoadcell = 0;
  if (currentTime - lastTimerLoadcell >= 500) { 
    if (scale.is_ready()) {
      float beratGram = scale.get_units(1); 
      beratAktualTerakhir = beratGram; 
      float beratSetelahReset = beratGram - offsetTareMonitor;

      mb.Hreg(REG_BERAT_ASLI,  (uint16_t)((int16_t)round(beratGram * 10.0)));
      mb.Hreg(REG_BERAT_RESET, (uint16_t)((int16_t)round(beratSetelahReset * 10.0)));
    }
    lastTimerLoadcell = currentTime;
  }

  yield(); 
}

// =======================================================
// DEKODER DATA SERI BARU DARI ARDUINO + UPDATE KE HMI
// =======================================================
void parseArduinoData(String data) {
  data.trim(); 
  
  int values[10] = {0};
  int count = 0;
  int startIndex = 0;
  int commaIndex = data.indexOf(',');
  
  while (commaIndex != -1 && count < 9) {
    String token = data.substring(startIndex, commaIndex);
    token.trim();
    values[count++] = token.toInt();
    startIndex = commaIndex + 1;
    commaIndex = data.indexOf(',', startIndex);
  }
  if (startIndex < data.length()) {
    String token = data.substring(startIndex);
    token.trim();
    values[count++] = token.toInt();
  }

  // Jika minimal data warna inti terisi
  if (count >= 3) {
    uint16_t rVal = values[0];
    uint16_t bVal = values[1];
    uint16_t wVal = values[2];

    mb.Hreg(REG_SENSOR_R, rVal);
    mb.Hreg(REG_SENSOR_G, 0); 
    mb.Hreg(REG_SENSOR_B, bVal);
    mb.Hreg(REG_WARNA, wVal); 

    if (wVal != warnaTerakhir) { 
      if (wVal == 1) {       
        totalMerah++;
        mb.Hreg(REG_BOX_MERAH, totalMerah);
        prefs.putUShort("merah", totalMerah); 
        
        seqMerah = 1;
        mb.Hreg(REG_SEQ_M1, 1);
      } 
      else if (wVal == 2) {  
        totalBiru++;
        mb.Hreg(REG_BOX_BIRU, totalBiru);
        prefs.putUShort("biru", totalBiru);   
        
        seqBiru = 1;
        mb.Hreg(REG_SEQ_B1, 1);
      } 
      else if (wVal == 3) {  
        totalLogam++;
        mb.Hreg(REG_BOX_LOGAM, totalLogam);
        prefs.putUShort("logam", totalLogam); 
        
        seqLogam = 1;
        mb.Hreg(REG_SEQ_L1, 1);
      }
      warnaTerakhir = wVal; 
    }
  }

  // Jika data indikator lengkap 10 kolom, masukkan ke HMI
  if (count == 10) {
    mb.Hreg(REG_IND_SERVO1, values[3]);
    mb.Hreg(REG_IND_SERVO2, values[4]);
    mb.Hreg(REG_IND_SERVO3, values[5]);
    mb.Hreg(REG_IND_PROX,   values[6]);
    mb.Hreg(REG_IND_IR1,    values[7]);
    mb.Hreg(REG_IND_IR2,    values[8]);
    mb.Hreg(REG_IND_MOTOR,  values[9]);

    // ==========================================================
    // [PEMBARUAN LOGIKA] - EKSEKUSI SEQUENCE DENGAN FORCE LOW
    // ==========================================================
    int servo1 = values[3];
    int servo2 = values[4];
    int servo3 = values[5];
    int ir1    = values[7];
    int ir2    = values[8];

    // --- SEQUENCE MERAH ---
    if (seqMerah == 1 && ir1 == 1) {
      seqMerah = 2;
      mb.Hreg(REG_SEQ_M2, 1); // Reg 18 High
      mb.Hreg(REG_SEQ_M1, 0); // Reg 17 Pasti Low
    }
    else if (seqMerah == 2 && last_servo1 == 1 && servo1 == 0) { 
      seqMerah = 3;
      mb.Hreg(REG_SEQ_M3, 1); // Reg 19 High
      mb.Hreg(REG_SEQ_M2, 0); // Reg 18 Pasti Low
      mb.Hreg(REG_SEQ_M1, 0); // Reg 17 Pasti Low
    }

    // --- SEQUENCE BIRU ---
    if (seqBiru == 1 && ir1 == 1) {
      seqBiru = 2;
      mb.Hreg(REG_SEQ_B2, 1); // Reg 21 High
      mb.Hreg(REG_SEQ_B1, 0); // Reg 20 Pasti Low
    }
    else if (seqBiru == 2 && ir2 == 1) {
      seqBiru = 3;
      mb.Hreg(REG_SEQ_B3, 1); // Reg 22 High
      mb.Hreg(REG_SEQ_B2, 0); // Reg 21 Pasti Low
      mb.Hreg(REG_SEQ_B1, 0); // Reg 20 Pasti Low
    }
    else if (seqBiru == 3 && last_servo2 == 1 && servo2 == 0) { 
      seqBiru = 4;
      mb.Hreg(REG_SEQ_B4, 1); // Reg 23 High
      mb.Hreg(REG_SEQ_B3, 0); // Reg 22 Pasti Low
      mb.Hreg(REG_SEQ_B2, 0); // Reg 21 Pasti Low
      mb.Hreg(REG_SEQ_B1, 0); // Reg 20 Pasti Low
    }

    // --- SEQUENCE LOGAM ---
    if (seqLogam == 1 && ir1 == 1) {
      seqLogam = 2;
      mb.Hreg(REG_SEQ_L2, 1); // Reg 25 High
      mb.Hreg(REG_SEQ_L1, 0); // Reg 24 Pasti Low
    }
    else if (seqLogam == 2 && ir2 == 1) {
      seqLogam = 3;
      mb.Hreg(REG_SEQ_L3, 1); // Reg 26 High
      mb.Hreg(REG_SEQ_L2, 0); // Reg 25 Pasti Low
      mb.Hreg(REG_SEQ_L1, 0); // Reg 24 Pasti Low
    }

    // ==========================================================
    // [PEMBARUAN LOGIKA] - RESET GLOBAL JIKA SERVO 3 (PENDORONG) HIGH
    // ==========================================================
    if (servo3 == 1) {
      // Reset semua status tracking internal
      seqMerah = 0;
      seqBiru = 0;
      seqLogam = 0;

      // Matikan (LOW) paksa semua register dari 17 sampai 26
      mb.Hreg(REG_SEQ_M1, 0); // Reg 17
      mb.Hreg(REG_SEQ_M2, 0); // Reg 18
      mb.Hreg(REG_SEQ_M3, 0); // Reg 19
      mb.Hreg(REG_SEQ_B1, 0); // Reg 20
      mb.Hreg(REG_SEQ_B2, 0); // Reg 21
      mb.Hreg(REG_SEQ_B3, 0); // Reg 22
      mb.Hreg(REG_SEQ_B4, 0); // Reg 23
      mb.Hreg(REG_SEQ_L1, 0); // Reg 24
      mb.Hreg(REG_SEQ_L2, 0); // Reg 25
      mb.Hreg(REG_SEQ_L3, 0); // Reg 26
    }

    // Simpan status servo saat ini untuk iterasi berikutnya (penting untuk deteksi High->Low)
    last_servo1 = servo1;
    last_servo2 = servo2;
  }
}
