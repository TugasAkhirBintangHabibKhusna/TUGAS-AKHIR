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

#define REG_IND_SERVO1   10  
#define REG_IND_SERVO2   11  
#define REG_IND_SERVO3   12  
#define REG_IND_PROX     13  
#define REG_IND_IR1      14  
#define REG_IND_IR2      15  
#define REG_IND_MOTOR    16  

#define REG_SEQ_M1 17 
#define REG_SEQ_M2 18 
#define REG_SEQ_M3 19 
#define REG_SEQ_B1 20 
#define REG_SEQ_B2 21 
#define REG_SEQ_B3 22 
#define REG_SEQ_B4 23 
#define REG_SEQ_L1 24 
#define REG_SEQ_L2 25 
#define REG_SEQ_L3 26 

// PEMETAAN ALAMAT COIL (TOMBOL HMI)
#define REG_SAKLAR       0  
#define REG_SAKLAR_TARE  1  
#define REG_RESET_MERAH  2  
#define REG_RESET_BIRU   3  
#define REG_RESET_LOGAM  4  
#define REG_EMERGENCY    5  

// =======================================================
// PIN & SPESIFIKASI ENCODER, LOADCELL & RELAY
// =======================================================
#define DE_RE_PIN 25
#define ENCODER_PIN_A 18 
#define ENCODER_PIN_B 19 
#define ENCODER_PPR 1600  

#define HX711_DOUT_PIN 22
#define HX711_SCK_PIN  23
HX711 scale;
float faktorKalibrasi = 501.3; 

#define PIN_RELAY_IN1 13
#define PIN_RELAY_IN2 14
#define PIN_RELAY_IN3 27
#define PIN_RELAY_IN4 26

ModbusRTU mb;

HardwareSerial SerialModbus(1);   
HardwareSerial SerialArduino(2);  

String inputString = "";
bool stringComplete = false;

// Variabel Loadcell
float beratAktualTerakhir = 0.0;
float offsetTareMonitor = 0.0;
float smoothedBerat = 0.0; // Variabel penyimpan hasil filter loadcell

// Konstanta Filter (Alpha)
const float ALPHA = 0.15;       // Alpha untuk Encoder
const float ALPHA_BERAT = 0.15; // Alpha untuk Loadcell (0.0 - 1.0, makin kecil makin halus)

// Variabel Encoder
volatile int32_t encoderCount = 0; 
volatile bool isEmergencyAktif = false; 

unsigned long lastTime = 0;
const unsigned long interval = 500; 
float smoothedRpm = 0.0;

uint16_t totalMerah = 0;
uint16_t totalBiru = 0;
uint16_t totalLogam = 0;
uint16_t warnaTerakhir = 0; 

uint8_t statusMotor  = 0;
uint8_t statusServo3 = 0;
uint8_t statusProx   = 0;
uint16_t currentWarna = 0;

uint8_t seqMerah = 0;
uint8_t seqBiru = 0;
uint8_t seqLogam = 0;
int last_servo1 = 0;
int last_servo2 = 0;

bool lastEmergencyState = false; 
unsigned long timerAlarmEmergency = 0;
bool prosesTungguServo = false;

void IRAM_ATTR readEncoder() {
  if (isEmergencyAktif) return; 
  
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

  pinMode(PIN_RELAY_IN1, OUTPUT); pinMode(PIN_RELAY_IN2, OUTPUT);
  pinMode(PIN_RELAY_IN3, OUTPUT); pinMode(PIN_RELAY_IN4, OUTPUT);
  
  digitalWrite(PIN_RELAY_IN1, HIGH); digitalWrite(PIN_RELAY_IN2, HIGH);
  digitalWrite(PIN_RELAY_IN3, HIGH); digitalWrite(PIN_RELAY_IN4, HIGH);

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

  mb.addHreg(REG_SENSOR_R, 0); mb.addHreg(REG_SENSOR_G, 0); mb.addHreg(REG_SENSOR_B, 0);
  mb.addHreg(REG_WARNA, 0); mb.addHreg(REG_ENCODER, 0); mb.addHreg(REG_BERAT_ASLI, 0);  
  mb.addHreg(REG_BERAT_RESET, 0); mb.addHreg(REG_BOX_MERAH, totalMerah);   
  mb.addHreg(REG_BOX_BIRU, totalBiru); mb.addHreg(REG_BOX_LOGAM, totalLogam);    

  mb.addHreg(REG_IND_SERVO1, 0); mb.addHreg(REG_IND_SERVO2, 0); mb.addHreg(REG_IND_SERVO3, 0);
  mb.addHreg(REG_IND_PROX, 0); mb.addHreg(REG_IND_IR1, 0); mb.addHreg(REG_IND_IR2, 0); mb.addHreg(REG_IND_MOTOR, 0);

  mb.addHreg(REG_SEQ_M1, 0); mb.addHreg(REG_SEQ_M2, 0); mb.addHreg(REG_SEQ_M3, 0);
  mb.addHreg(REG_SEQ_B1, 0); mb.addHreg(REG_SEQ_B2, 0); mb.addHreg(REG_SEQ_B3, 0); mb.addHreg(REG_SEQ_B4, 0);
  mb.addHreg(REG_SEQ_L1, 0); mb.addHreg(REG_SEQ_L2, 0); mb.addHreg(REG_SEQ_L3, 0);

  mb.addCoil(REG_SAKLAR, false); mb.addCoil(REG_SAKLAR_TARE, false); 
  mb.addCoil(REG_RESET_MERAH, false); mb.addCoil(REG_RESET_BIRU, false);  
  mb.addCoil(REG_RESET_LOGAM, false); mb.addCoil(REG_EMERGENCY, false); 

  inputString.reserve(60); 
  Serial.println("ESP32 Ready - Indikator Modbus Aktif!");
}

void loop() {
  mb.task();  

  unsigned long currentTime = millis();
  
  bool currentEmergencyState = mb.Coil(REG_EMERGENCY);
  isEmergencyAktif = currentEmergencyState; 

  // --- LOGIKA KONTROL EMERGENCY ESP32 ---
  if (currentEmergencyState != lastEmergencyState) {
    if (currentEmergencyState == true) {
      SerialArduino.print('E'); 
      
      // PERUBAHAN: Paksa saklar ON/OFF menjadi OFF (false) agar tidak mengunci setelah reset
      mb.Coil(REG_SAKLAR, false); 
      
      // Mulai menghitung 1 detik sebelum membunyikan Alarm agar servo sempat kembali
      timerAlarmEmergency = millis();
      prosesTungguServo = true;
      
    } else {
      SerialArduino.print('C'); 
      prosesTungguServo = false;
    }
    lastEmergencyState = currentEmergencyState;
  }

  // 1. KONTROL ENCODER
  if (currentTime - lastTime >= interval) {
    if (!isEmergencyAktif) {
      noInterrupts();
      int32_t count = encoderCount;
      encoderCount = 0; 
      interrupts();

      int16_t rawRpm = (int16_t)((count * 60000L) / (ENCODER_PPR * interval));
      
      // ====================================================================================
      // ⬇⬇⬇ RUMUS ALPHA FILTER UNTUK ENCODER (RPM) ⬇⬇⬇
      // ====================================================================================
      if (rawRpm == 0) {
        smoothedRpm = 0; 
      } else {
        smoothedRpm = (ALPHA * rawRpm) + ((1.0 - ALPHA) * smoothedRpm);
      }
      // ====================================================================================

      int16_t finalRpm = (int16_t)round(smoothedRpm);
      mb.Hreg(REG_ENCODER, (uint16_t)finalRpm);
    } else {
      smoothedRpm = 0;
      mb.Hreg(REG_ENCODER, 0);
    }
    lastTime = currentTime;
  }

  // 2. CEK TOMBOL UTAMA & KIRIM STATUS 
  static unsigned long timerKirimStatus = 0;
  bool currentSaklarState = mb.Coil(REG_SAKLAR);
  
  if (currentTime - timerKirimStatus >= 200) {
    if (!isEmergencyAktif) {
      if (currentSaklarState == true) SerialArduino.print('1');  
      else SerialArduino.print('0');  
    }
    timerKirimStatus = currentTime;
  }

  // 2.1 LOGIKA EKSEKUSI RELAY (ALARM DELAY)
  if (isEmergencyAktif) {
    // Matikan semua aktuator normal
    digitalWrite(PIN_RELAY_IN1, HIGH);
    digitalWrite(PIN_RELAY_IN2, HIGH);
    digitalWrite(PIN_RELAY_IN3, HIGH);
    
    // PERUBAHAN: Tunggu 1 detik agar servo selesai balik ke Standby, baru nyalakan Relay Alarm (IN4)
    if (prosesTungguServo && (currentTime - timerAlarmEmergency >= 1000)) {
      digitalWrite(PIN_RELAY_IN4, LOW); // ON-kan Alarm
      prosesTungguServo = false;
    } else if (!prosesTungguServo) {
      digitalWrite(PIN_RELAY_IN4, LOW); // Pertahankan Alarm ON
    } else {
      digitalWrite(PIN_RELAY_IN4, HIGH); // Masih dalam masa jeda 1 detik, tahan dulu
    }
    
  } else {
    // Kondisi Normal
    digitalWrite(PIN_RELAY_IN4, HIGH); // Alarm Mati

    if (!currentSaklarState) {
      digitalWrite(PIN_RELAY_IN1, LOW);
      digitalWrite(PIN_RELAY_IN2, HIGH);
      digitalWrite(PIN_RELAY_IN3, HIGH);
    } else {
      if (statusMotor == 1) {
        digitalWrite(PIN_RELAY_IN1, HIGH);
        digitalWrite(PIN_RELAY_IN2, LOW);
        digitalWrite(PIN_RELAY_IN3, HIGH);
      } else {
        if (statusServo3 == 1 || currentWarna != 0 || statusProx == 1) {
          digitalWrite(PIN_RELAY_IN1, HIGH);
          digitalWrite(PIN_RELAY_IN2, HIGH);
          digitalWrite(PIN_RELAY_IN3, LOW);
        } else {
          digitalWrite(PIN_RELAY_IN1, HIGH);
          digitalWrite(PIN_RELAY_IN2, HIGH);
          digitalWrite(PIN_RELAY_IN3, HIGH);
        }
      }
    }
  }

  // 3. BACA BALASAN DATA DARI ARDUINO
  while (SerialArduino.available()) {
    char inChar = (char)SerialArduino.read();
    if (inChar == '\n') stringComplete = true;
    else if (inChar != '\r') inputString += inChar;
  }

  if (stringComplete) {
    parseArduinoData(inputString);
    inputString = ""; stringComplete = false;
  }

  if (mb.Coil(REG_SAKLAR_TARE) == true) {
    offsetTareMonitor = beratAktualTerakhir; mb.Coil(REG_SAKLAR_TARE, false); 
  }

  if (mb.Coil(REG_RESET_MERAH) == true) { totalMerah = 0; mb.Hreg(REG_BOX_MERAH, 0); prefs.putUShort("merah", 0); mb.Coil(REG_RESET_MERAH, false); }
  if (mb.Coil(REG_RESET_BIRU) == true)  { totalBiru = 0;  mb.Hreg(REG_BOX_BIRU, 0);  prefs.putUShort("biru", 0);  mb.Coil(REG_RESET_BIRU, false); }
  if (mb.Coil(REG_RESET_LOGAM) == true) { totalLogam = 0; mb.Hreg(REG_BOX_LOGAM, 0); prefs.putUShort("logam", 0); mb.Coil(REG_RESET_LOGAM, false); }

  static unsigned long lastTimerLoadcell = 0;
  if (!isEmergencyAktif) {
    if (currentTime - lastTimerLoadcell >= 500) { 
      if (scale.is_ready()) {
        float beratMentah = scale.get_units(1); // Murni pembacaan sensor
        
        // ====================================================================================
        // ⬇⬇⬇ RUMUS ALPHA FILTER UNTUK LOADCELL (BERAT) ⬇⬇⬇
        // ====================================================================================
        smoothedBerat = (ALPHA_BERAT * beratMentah) + ((1.0 - ALPHA_BERAT) * smoothedBerat);
        // ====================================================================================
        
        beratAktualTerakhir = smoothedBerat; 
        float beratSetelahReset = smoothedBerat - offsetTareMonitor;

        mb.Hreg(REG_BERAT_ASLI,  (uint16_t)((int16_t)round(smoothedBerat * 10.0)));
        mb.Hreg(REG_BERAT_RESET, (uint16_t)((int16_t)round(beratSetelahReset * 10.0)));
      }
      lastTimerLoadcell = currentTime;
    }
  }

  yield(); 
}

void parseArduinoData(String data) {
  data.trim(); 
  
  int values[10] = {0};
  int count = 0; int startIndex = 0;
  int commaIndex = data.indexOf(',');
  
  while (commaIndex != -1 && count < 9) {
    String token = data.substring(startIndex, commaIndex);
    token.trim(); values[count++] = token.toInt();
    startIndex = commaIndex + 1; commaIndex = data.indexOf(',', startIndex);
  }
  if (startIndex < data.length()) {
    String token = data.substring(startIndex);
    token.trim(); values[count++] = token.toInt();
  }

  if (count >= 3) {
    uint16_t rVal = values[0]; uint16_t bVal = values[1]; uint16_t wVal = values[2];
    currentWarna = wVal; 

    mb.Hreg(REG_SENSOR_R, rVal); mb.Hreg(REG_SENSOR_G, 0); 
    mb.Hreg(REG_SENSOR_B, bVal); mb.Hreg(REG_WARNA, wVal); 

    if (wVal != warnaTerakhir) { 
      if (wVal == 1) { totalMerah++; mb.Hreg(REG_BOX_MERAH, totalMerah); prefs.putUShort("merah", totalMerah); seqMerah = 1; mb.Hreg(REG_SEQ_M1, 1); } 
      else if (wVal == 2) { totalBiru++; mb.Hreg(REG_BOX_BIRU, totalBiru); prefs.putUShort("biru", totalBiru); seqBiru = 1; mb.Hreg(REG_SEQ_B1, 1); } 
      else if (wVal == 3) { totalLogam++; mb.Hreg(REG_BOX_LOGAM, totalLogam); prefs.putUShort("logam", totalLogam); seqLogam = 1; mb.Hreg(REG_SEQ_L1, 1); }
      warnaTerakhir = wVal; 
    }
  }

  if (count == 10) {
    mb.Hreg(REG_IND_SERVO1, values[3]); mb.Hreg(REG_IND_SERVO2, values[4]); mb.Hreg(REG_IND_SERVO3, values[5]);
    mb.Hreg(REG_IND_PROX,   values[6]); mb.Hreg(REG_IND_IR1,    values[7]); mb.Hreg(REG_IND_IR2,    values[8]);
    mb.Hreg(REG_IND_MOTOR,  values[9]);

    statusServo3 = values[5]; statusProx = values[6]; statusMotor = values[9];

    int servo1 = values[3]; int servo2 = values[4]; int servo3 = values[5];
    int ir1 = values[7]; int ir2 = values[8];

    if (seqMerah == 1 && ir1 == 1) { seqMerah = 2; mb.Hreg(REG_SEQ_M2, 1); mb.Hreg(REG_SEQ_M1, 0); }
    else if (seqMerah == 2 && last_servo1 == 1 && servo1 == 0) { seqMerah = 3; mb.Hreg(REG_SEQ_M3, 1); mb.Hreg(REG_SEQ_M2, 0); mb.Hreg(REG_SEQ_M1, 0); }

    if (seqBiru == 1 && ir1 == 1) { seqBiru = 2; mb.Hreg(REG_SEQ_B2, 1); mb.Hreg(REG_SEQ_B1, 0); }
    else if (seqBiru == 2 && ir2 == 1) { seqBiru = 3; mb.Hreg(REG_SEQ_B3, 1); mb.Hreg(REG_SEQ_B2, 0); mb.Hreg(REG_SEQ_B1, 0); }
    else if (seqBiru == 3 && last_servo2 == 1 && servo2 == 0) { seqBiru = 4; mb.Hreg(REG_SEQ_B4, 1); mb.Hreg(REG_SEQ_B3, 0); mb.Hreg(REG_SEQ_B2, 0); mb.Hreg(REG_SEQ_B1, 0); }

    if (seqLogam == 1 && ir1 == 1) { seqLogam = 2; mb.Hreg(REG_SEQ_L2, 1); mb.Hreg(REG_SEQ_L1, 0); }
    else if (seqLogam == 2 && ir2 == 1) { seqLogam = 3; mb.Hreg(REG_SEQ_L3, 1); mb.Hreg(REG_SEQ_L2, 0); mb.Hreg(REG_SEQ_L1, 0); }

    if (servo3 == 1) {
      seqMerah = 0; seqBiru = 0; seqLogam = 0;
      mb.Hreg(REG_SEQ_M1, 0); mb.Hreg(REG_SEQ_M2, 0); mb.Hreg(REG_SEQ_M3, 0);
      mb.Hreg(REG_SEQ_B1, 0); mb.Hreg(REG_SEQ_B2, 0); mb.Hreg(REG_SEQ_B3, 0); mb.Hreg(REG_SEQ_B4, 0);
      mb.Hreg(REG_SEQ_L1, 0); mb.Hreg(REG_SEQ_L2, 0); mb.Hreg(REG_SEQ_L3, 0);
    }
    last_servo1 = servo1; last_servo2 = servo2;
  }
}
