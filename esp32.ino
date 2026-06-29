#include <ModbusRTU.h>
#include <HX711.h> 
#include <Preferences.h> // [TAMBAHAN] Library untuk menyimpan data ke memori Flash ESP32

// Instance untuk memori internal
Preferences prefs; 

// =======================================================
// PEMETAAN ALAMAT REGISTER MODBUS (Sesuai Struktur Anda)
// =======================================================
#define REG_SENSOR_R    0  
#define REG_SENSOR_G    1  
#define REG_SENSOR_B    2  
#define REG_WARNA       3  
#define REG_ENCODER     4  
#define REG_BERAT_ASLI  5  // 4x5 -> Berat Asli (dikali 10)
#define REG_BERAT_RESET 6  // 4x6 -> Berat Setelah Reset (dikali 10)

#define REG_BOX_MERAH   7  // 4x7 -> Total Box Merah
#define REG_BOX_BIRU    8  // 4x8 -> Total Box Biru
#define REG_BOX_LOGAM   9  // 4x9 -> Total Box Logam

// PEMETAAN ALAMAT COIL (TOMBOL HMI)
#define REG_SAKLAR       0  // 0x0 -> Saklar Utama ke Arduino
#define REG_SAKLAR_TARE  1  // 0x1 -> Tombol Reset Berat Timbangan

#define REG_RESET_MERAH  2  // 0x2 -> Tombol Reset Counter Merah
#define REG_RESET_BIRU   3  // 0x3 -> Tombol Reset Counter Biru
#define REG_RESET_LOGAM  4  // 0x4 -> Tombol Reset Counter Logam

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

// VARIABEL TARE & COUNTER
float beratAktualTerakhir = 0.0;
float offsetTareMonitor = 0.0;

volatile int32_t encoderCount = 0; 
unsigned long lastTime = 0;
const unsigned long interval = 500; 

float smoothedRpm = 0.0;
const float ALPHA = 0.50; 

// VARIABEL COUNTER RAM BIASA
uint16_t totalMerah = 0;
uint16_t totalBiru = 0;
uint16_t totalLogam = 0;
uint16_t warnaTerakhir = 0; 

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

  // [TAMBAHAN] Buka ruang penyimpanan bernama "hitungBox"
  prefs.begin("hitungBox", false); 
  // Ambil nilai terakhir yang tersimpan sebelum mati. Kalau kosong/baru pertama kali, beri nilai 0.
  totalMerah = prefs.getUShort("merah", 0);
  totalBiru  = prefs.getUShort("biru", 0);
  totalLogam = prefs.getUShort("logam", 0);

  // Daftarkan Holding Register & Langsung isi dengan nilai yang berhasil diselamatkan dari memori flash
  mb.addHreg(REG_SENSOR_R, 0);
  mb.addHreg(REG_SENSOR_G, 0);
  mb.addHreg(REG_SENSOR_B, 0);
  mb.addHreg(REG_WARNA, 0);
  mb.addHreg(REG_ENCODER, 0); 
  mb.addHreg(REG_BERAT_ASLI, 0);  
  mb.addHreg(REG_BERAT_RESET, 0); 
  mb.addHreg(REG_BOX_MERAH, totalMerah); // Masukkan nilai terselamatkan ke HMI   
  mb.addHreg(REG_BOX_BIRU, totalBiru);   // Masukkan nilai terselamatkan ke HMI 
  mb.addHreg(REG_BOX_LOGAM, totalLogam); // Masukkan nilai terselamatkan ke HMI    

  // Daftarkan Coil Bit
  mb.addCoil(REG_SAKLAR, false);
  mb.addCoil(REG_SAKLAR_TARE, false); 
  mb.addCoil(REG_RESET_MERAH, false); 
  mb.addCoil(REG_RESET_BIRU, false);  
  mb.addCoil(REG_RESET_LOGAM, false); 

  inputString.reserve(40);
  Serial.println("ESP32 Aktif - Mode Proteksi Memori Nilai Warna Aktif!");
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

  // 3. BACA BALASAN DATA SENSOR DARI ARDUINO (Dibersihkan dari \r agar lancar)
  while (SerialArduino.available()) {
    char inChar = (char)SerialArduino.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') { // Mengabaikan karakter pembawa masalah
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

  // 5. RESET VARIABEL COUNTER (Juga menghapus nilai di memori flash menjadi 0)
  if (mb.Coil(REG_RESET_MERAH) == true) { 
    totalMerah = 0; 
    mb.Hreg(REG_BOX_MERAH, 0); 
    prefs.putUShort("merah", 0); // [MODIFIKASI] Reset flash merah
    mb.Coil(REG_RESET_MERAH, false); 
  }
  if (mb.Coil(REG_RESET_BIRU) == true)  { 
    totalBiru = 0;  
    mb.Hreg(REG_BOX_BIRU, 0);  
    prefs.putUShort("biru", 0);  // [MODIFIKASI] Reset flash biru
    mb.Coil(REG_RESET_BIRU, false); 
  }
  if (mb.Coil(REG_RESET_LOGAM) == true) { 
    totalLogam = 0; 
    mb.Hreg(REG_BOX_LOGAM, 0); 
    prefs.putUShort("logam", 0); // [MODIFIKASI] Reset flash logam
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
// BAGIAN PENGIRIMAN WARNA YANG DISAMAKAN & DIPERBAIKI (ANTI SPASI/GAIB)
// =======================================================
void parseArduinoData(String data) {
  data.trim(); // Menghapus spasi gaib di awal/akhir baris teks
  
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    String rStr = data.substring(0, firstComma);
    String bStr = data.substring(firstComma + 1, secondComma);
    String wStr = data.substring(secondComma + 1);

    // Pembersihan ekstra pada potongan teks string sebelum masuk ke HMI
    rStr.trim();
    bStr.trim();
    wStr.trim();

    uint16_t rVal = rStr.toInt();
    uint16_t bVal = bStr.toInt();
    uint16_t wVal = wStr.toInt();

    // Kirim langsung nilai murni ke register HMI Haiwell
    mb.Hreg(REG_SENSOR_R, rVal);
    mb.Hreg(REG_SENSOR_G, 0); 
    mb.Hreg(REG_SENSOR_B, bVal);
    mb.Hreg(REG_WARNA, wVal); // <--- Menjamin REG_WARNA terisi angka murni (1, 2, atau 3)

    // Logika penambahan counter biasa (RAM) berdasarkan wVal murni + Backup Flash otomatis
    if (wVal != warnaTerakhir) { 
      if (wVal == 1) {       
        totalMerah++;
        mb.Hreg(REG_BOX_MERAH, totalMerah);
        prefs.putUShort("merah", totalMerah); // [TAMBAHAN] Amankan nilai merah ke flash
      } 
      else if (wVal == 2) {  
        totalBiru++;
        mb.Hreg(REG_BOX_BIRU, totalBiru);
        prefs.putUShort("biru", totalBiru);   // [TAMBAHAN] Amankan nilai biru ke flash
      } 
      else if (wVal == 3) {  
        totalLogam++;
        mb.Hreg(REG_BOX_LOGAM, totalLogam);
        prefs.putUShort("logam", totalLogam); // [TAMBAHAN] Amankan nilai logam ke flash
      }
      warnaTerakhir = wVal; 
    }
  }
}
