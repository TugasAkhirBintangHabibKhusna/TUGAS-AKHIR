#include <ModbusRTU.h>

// Pemetaan Alamat Register sesuai Haiwell SCADA
#define REG_SENSOR_R 0  // Holding Register 4X - 0
#define REG_SENSOR_G 1  // Holding Register 4X - 1
#define REG_SENSOR_B 2  // Holding Register 4X - 2
#define REG_SAKLAR 0    // Coil 0X - 0 (Tombol ON/OFF dari HMI)
#define REG_WARNA 3     // Holding Register 4X - 3

#define DE_RE_PIN 25

ModbusRTU mb;

HardwareSerial SerialModbus(1);   // UART1 untuk HMI
HardwareSerial SerialArduino(2);  // UART2 untuk Arduino

String inputString = "";
bool stringComplete = false;

// Memori penyimpan status tombol HMI
bool lastSaklarState = false;

void setup() {
  Serial.begin(115200);

  // Komunikasi ke Arduino (Baudrate 9600)
  SerialArduino.begin(9600, SERIAL_8N1, 16, 17);

  // Komunikasi Modbus ke HMI Haiwell (Baudrate 9600)
  SerialModbus.begin(9600, SERIAL_8N1, 32, 33);

  mb.begin(&SerialModbus, DE_RE_PIN);
  mb.slave(1);

  mb.addHreg(REG_SENSOR_R, 0);
  mb.addHreg(REG_SENSOR_G, 0);
  mb.addHreg(REG_SENSOR_B, 0);
  mb.addCoil(REG_SAKLAR, false);
  mb.addHreg(REG_WARNA, 0);

  inputString.reserve(40);
  Serial.println("Sistem ESP32 Aktif - Siap Komunikasi 2 Arah");
}

void loop() {
  mb.task();  // Latar belakang Modbus

  // =======================================================
  // 1. CEK TOMBOL HMI & KIRIM PERINTAH KE ARDUINO
  // =======================================================
  bool currentSaklarState = mb.Coil(REG_SAKLAR);

  // Jika tombol HMI ditekan (berubah status)
  if (currentSaklarState != lastSaklarState) {
    if (currentSaklarState == true) {
      SerialArduino.print('1');  // Tembak sinyal HIGH ke Arduino
      Serial.println("HMI ON -> Kirim '1' ke Arduino");
    } else {
      SerialArduino.print('0');  // Tembak sinyal LOW ke Arduino
      Serial.println("HMI OFF -> Kirim '0' ke Arduino");
    }
    lastSaklarState = currentSaklarState;
  }

  // =======================================================
  // 2. BACA BALASAN DATA SENSOR DARI ARDUINO
  // =======================================================
  while (SerialArduino.available()) {
    char inChar = (char)SerialArduino.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    parseArduinoData(inputString);
    inputString = "";
    stringComplete = false;
  }

  yield();
}

void parseArduinoData(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    String rStr = data.substring(0, firstComma);
    String bStr = data.substring(firstComma + 1, secondComma);
    String wStr = data.substring(secondComma + 1);

    uint16_t rVal = rStr.toInt();
    uint16_t bVal = bStr.toInt();
    uint16_t wVal = wStr.toInt();

    mb.Hreg(REG_SENSOR_R, rVal);
    mb.Hreg(REG_SENSOR_G, 0);
    mb.Hreg(REG_SENSOR_B, bVal);
    mb.Hreg(REG_WARNA, wVal);
  }
}
