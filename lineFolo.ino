#include <Preferences.h>

Preferences preferences;

// ================= PIN DEFINITIONS (MENGGUNAKAN PIN ANDA) =================

// Motor Kiri 
const int pinPwmKiri  = 32;
const int pinDir1Kiri = 26;
const int pinDir2Kiri = 27;

// Motor Kanan 
const int pinPwmKanan  = 13;
const int pinDir1Kanan = 25;
const int pinDir2Kanan = 12;

// Sensor Garis (TCRT5000)
// Urutan: Kiri Pojok, Kiri, Kanan, Kanan Pojok
const int pinSensor[4] = {14, 35, 34, 33}; 
int nilaiSensor[4];
int minSensor[4] = {4095, 4095, 4095, 4095};
int maxSensor[4] = {0, 0, 0, 0};
int threshold[4];

// Sensor Ultrasonik (Memakai pin Anda)
const int trigDepan = 18;
const int echoDepan = 19;
const int trigKiri  = 16;
const int echoKiri  = 17;

// ================= VARIABEL KONTROL LINE FOLLOWER =================
// PID
float Kp = 40.0;
float Ki = 0.0;
float Kd = 70.0;
int error = 0;
int lastError = 0;
int kecepatanDasar = 150; 
int kecepatanMaks = 255;

// State/Mode Robot
enum ModeRobot { IDLE, KALIBRASI, DEBUG, AUTO_LINE, AVOIDER };
ModeRobot modeSaatIni = AUTO_LINE;

// Mapping (Konsep dasar)
const int MAX_STEPS = 50;
char routeMap[MAX_STEPS];
int stepCount = 0;
int currentStep = 0;
bool sedangDiPersimpangan = false; 

// ================= VARIABEL KONTROL WALL FOLLOWER (AVOIDER) =================
float Kp_Wall = 7.5;
float Ki_Wall = 0.0;
float Kd_Wall = 110.0;
float lastErrorWall = 0;
float integralWall = 0;
int kecepatanDasarWall = 90; // Kecepatan dasar KHUSUS wall follower

unsigned long waktuMulaiAvoider = 0;
unsigned long waktuPrintTerakhir = 0;
const int intervalPrint = 100;

// LOCK MODE WALL FOLLOWER (BUTA GARIS) SELAMA 2 DETIK (2000 ms)
const unsigned long DURASI_BLIND = 600;


void setup() {
  Serial.begin(115200);

  // Setup Motor
  pinMode(pinPwmKiri, OUTPUT); pinMode(pinDir1Kiri, OUTPUT); pinMode(pinDir2Kiri, OUTPUT);
  pinMode(pinPwmKanan, OUTPUT); pinMode(pinDir1Kanan, OUTPUT); pinMode(pinDir2Kanan, OUTPUT);

  // Setup Ultrasonik
  pinMode(trigDepan, OUTPUT); pinMode(echoDepan, INPUT);
  pinMode(trigKiri, OUTPUT); pinMode(echoKiri, INPUT);

  // Load Kalibrasi & Map dari NVRAM (Preferences)
  preferences.begin("kalibrasi", false);
  muatKalibrasi();

  String petaTersimpan = preferences.getString("savedMap", "");
  stepCount = preferences.getInt("mapSteps", 0);
  for (int i = 0; i < stepCount; i++) {
    routeMap[i] = petaTersimpan[i];
  }
  
  Serial.print("Map Dimuat dari Memori: ");
  Serial.println(petaTersimpan);

  tampilkanMenu();
}


void loop() {
  cekInputSerial();
  switch (modeSaatIni) {
    case KALIBRASI:
      jalankanKalibrasi();
      break;
    case DEBUG:
      tampilkanDebug();
      delay(500);
      break;
      
    case AUTO_LINE: {
      bacaGaris();
      int jarakDepan = bacaJarak(trigDepan, echoDepan);

      // Jika ada halangan di depan (mulai mode AVOIDER)
      if (jarakDepan < 16 && jarakDepan > 0) {
        motorBerhenti();
        Serial.println("Box terdeteksi! Masuk ke AVOIDER (Lock Buta Garis).");
        delay(100); 
        
        // [REVISI] Putar Kanan agar box berada di sebelah kiri sensor ultrasonik
        kendaliMotor(180, -180); 
        delay(260); 

        waktuMulaiAvoider = millis();
        lastErrorWall = 0;
        integralWall = 0;

        modeSaatIni = AVOIDER;
      } else {
        jalankanPID();
      }
      break;
    }
    
    case AVOIDER:
      wallFollower(); 
      break;
      
    case IDLE:
    default:
      motorBerhenti();
      break;
  }
}

// ================= FUNGSI UTAMA =================

void cekInputSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "1") {
      modeSaatIni = KALIBRASI;
      Serial.println("Memulai Kalibrasi...");
      delay(2000);
    } 
    else if (input == "2") {
      modeSaatIni = DEBUG;
      Serial.println("Mode Debug Aktif.");
    } 
    else if (input == "3") {
      currentStep = 0;
      modeSaatIni = AUTO_LINE;
      Serial.println("Robot Berjalan menggunakan Map...");
    } 
    else if (input == "4") {
      waktuMulaiAvoider = millis();
      lastErrorWall = 0;
      integralWall = 0;
      modeSaatIni = AVOIDER;
      Serial.println("Mode AVOIDER (Wall Follower PID) Aktif Manual.");
    }
    else if (input == "0") {
      modeSaatIni = IDLE;
      Serial.println("Robot Berhenti.");
    } 
    else if (input.startsWith("MAP:")) {
      String mapData = input.substring(4);
      stepCount = 0;
      for (int i = 0; i < mapData.length() && i < MAX_STEPS; i++) {
        routeMap[stepCount++] = mapData[i];
      }
      preferences.putString("savedMap", mapData);
      preferences.putInt("mapSteps", stepCount);
      Serial.print("Map Permanen Tersimpan: ");
      for (int i = 0; i < stepCount; i++) Serial.print(routeMap[i]);
      Serial.println();
    }
  }
}

void tampilkanMenu() {
  Serial.println("=== MENU ROBOT ===");
  Serial.println("Kirim '1' -> Kalibrasi Sensor");
  Serial.println("Kirim '2' -> Mode Debugging");
  Serial.println("Kirim '3' -> Start / Auto Mode");
  Serial.println("Kirim '4' -> Start Wall Follower PID Mode (AVOIDER)");
  Serial.println("Kirim '0' -> Stop / Idle");
  Serial.println("==================");
}

void jalankanKalibrasi() {
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < 4; i++) {
      int val = analogRead(pinSensor[i]);
      if (val < minSensor[i]) minSensor[i] = val;
      if (val > maxSensor[i]) maxSensor[i] = val;
    }
    kendaliMotor(70, -70);
  }
  motorBerhenti();

  for (int i = 0; i < 4; i++) {
    threshold[i] = (minSensor[i] + maxSensor[i]) / 2;
    String key = "th" + String(i);
    preferences.putInt(key.c_str(), threshold[i]);
  }
  Serial.println("Kalibrasi Selesai!");
  modeSaatIni = IDLE;
  tampilkanMenu();
}

void muatKalibrasi() {
  for (int i = 0; i < 4; i++) {
    String key = "th" + String(i);
    threshold[i] = preferences.getInt(key.c_str(), 2000);
  }
  Serial.println("Data Kalibrasi Dimuat.");
}

void bacaGaris() {
  int s0 = analogRead(pinSensor[0]) > threshold[0] ? 1 : 0; 
  int s1 = analogRead(pinSensor[1]) > threshold[1] ? 1 : 0; 
  int s2 = analogRead(pinSensor[2]) > threshold[2] ? 1 : 0; 
  int s3 = analogRead(pinSensor[3]) > threshold[3] ? 1 : 0;

  if (s0 == 1 && s3 == 1) {
    if (!sedangDiPersimpangan) { 
      sedangDiPersimpangan = true;
      if (currentStep < stepCount) {
        char aksi = routeMap[currentStep];
        eksekusiAksiMap(aksi); 
        currentStep++;
      }
    }
  } else {
    if (s0 == 0 && s3 == 0) sedangDiPersimpangan = false;
  }

  if      (s0 == 1 && s1 == 0 && s2 == 0 && s3 == 0) error = -3;
  else if (s0 == 1 && s1 == 1 && s2 == 0 && s3 == 0) error = -2;
  else if (s0 == 0 && s1 == 1 && s2 == 0 && s3 == 0) error = -1;
  else if (s0 == 0 && s1 == 1 && s2 == 1 && s3 == 0) error = 0;
  else if (s0 == 0 && s1 == 0 && s2 == 1 && s3 == 0) error = 1;
  else if (s0 == 0 && s1 == 0 && s2 == 1 && s3 == 1) error = 2;
  else if (s0 == 0 && s1 == 0 && s2 == 0 && s3 == 1) error = 3;
  else if (s0 == 0 && s1 == 0 && s2 == 0 && s3 == 0) {
    if (lastError > 0) error = 4;
    else error = -4;
  }
}

void jalankanPID() {
  int P = error;
  int D = error - lastError;
  int kecepatanPID = (Kp * P) + (Kd * D);
  lastError = error;

  int pwmKiri = kecepatanDasar + kecepatanPID;
  int pwmKanan = kecepatanDasar - kecepatanPID;

  pwmKiri = constrain(pwmKiri, -kecepatanMaks, kecepatanMaks);
  pwmKanan = constrain(pwmKanan, -kecepatanMaks, kecepatanMaks);

  kendaliMotor(pwmKiri, pwmKanan);
}

// ================= WALL FOLLOWER (AVOIDER) LOGIC =================
void wallFollower() {
  if (millis() - waktuMulaiAvoider > DURASI_BLIND) {
    int cekGaris = 0;
    for (int i = 0; i < 4; i++) {
      if (analogRead(pinSensor[i]) > threshold[i]) cekGaris++;
    }
    if (cekGaris >= 1) { 
      kembaliKeGaris(); // Panggil manuver pelurusan
      return; 
    }
  }

  int jarakKiri = bacaJarak(trigKiri, echoKiri);
  int jarakDepan = bacaJarak(trigDepan, echoDepan);

  // LOGIKA TIKUNGAN DALAM (INNER CORNER) - BELOK KANAN
  if (jarakDepan > 0 && jarakDepan < 12) {
    eksekusiTikunganKanan();
    return; 
  }

  // LOGIKA TIKUNGAN LUAR (OUTER CORNER) - HILANG TEMBOK > 60cm
  if (jarakKiri > 60 || jarakKiri == 999) {
    eksekusiTikunganKiri();
    return; 
  }

  // KONTROL PID 
  int targetJarak = 9;
  
  if (jarakKiri > 45) {
    jarakKiri = 45; 
  }

  float errorWall = targetJarak - jarakKiri;

  if (errorWall > 4.0) {
    errorWall = errorWall * 2.0;
  }

  integralWall = constrain(integralWall + errorWall, -50, 50); 
  float dWall = errorWall - lastErrorWall;
  
  int koreksi = (Kp_Wall * errorWall) + (Ki_Wall * integralWall) + (Kd_Wall * dWall);
  lastErrorWall = errorWall;

  int pwmKiri = kecepatanDasarWall + koreksi; 
  int pwmKanan = kecepatanDasarWall - koreksi;

  pwmKiri = constrain(pwmKiri, -220, 220);
  pwmKanan = constrain(pwmKanan, -220, 220);

  // DEADZONE FILTER
  if (pwmKiri > 0 && pwmKiri < 60) pwmKiri = 60;
  else if (pwmKiri < 0 && pwmKiri > -60) pwmKiri = -60;
  
  if (pwmKanan > 0 && pwmKanan < 60) pwmKanan = 60;
  else if (pwmKanan < 0 && pwmKanan > -60) pwmKanan = -60;

  kendaliMotor(pwmKiri, pwmKanan);

  if (millis() - waktuPrintTerakhir >= intervalPrint) {
    Serial.print("Jarak_Kiri:"); Serial.print(jarakKiri);
    Serial.print("\tTarget:"); Serial.print(targetJarak);
    Serial.print("\tError:"); Serial.print(errorWall);
    Serial.print("\tPWM_Kiri:"); Serial.print(pwmKiri);
    Serial.print("\tPWM_Kanan:"); Serial.println(pwmKanan);
    waktuPrintTerakhir = millis();
  }
}

// ================= FUNGSI MANUVER WALL FOLLOWER =================

void eksekusiTikunganKanan() {
  Serial.println("STATUS: Halangan Depan! Pivot Kanan...");
  
  motorBerhenti();
  delay(100);

  // [REVISI] Pivot Kanan
  kendaliMotor(160, -160);
  delay(350); 

  motorBerhenti();
  delay(100);

  lastErrorWall = 0; 
  integralWall = 0;
}

void eksekusiTikunganKiri() {
  Serial.println("STATUS: Lost > 60cm! Memulai Manuver Tikungan Kiri...");
  
  kendaliMotor(kecepatanDasarWall, kecepatanDasarWall);
  delay(150);

  // [REVISI] Pivot Kiri
  kendaliMotor(-160, 160);
  delay(200);

  Serial.println("STATUS: Maju Mencari Tembok & Scanning Garis...");
  kendaliMotor(kecepatanDasarWall, kecepatanDasarWall); 

  unsigned long waktuMulaiCari = millis();
  
  while (millis() - waktuMulaiCari < 4000) {
    
    if (millis() - waktuMulaiAvoider > DURASI_BLIND) {
      int cekGaris = 0;
      for (int i = 0; i < 4; i++) {
        if (analogRead(pinSensor[i]) > threshold[i]) cekGaris++;
      }
      if (cekGaris >= 1) { 
        kembaliKeGaris(); 
        return; 
      }
    }

    int cekJarakKiri = bacaJarak(trigKiri, echoKiri);
    int cekJarakDepan = bacaJarak(trigDepan, echoDepan); 
    
    if (cekJarakDepan > 0 && cekJarakDepan < 12) {
      Serial.println("STATUS: Awas Nabrak! Eksekusi Darurat Belok Kanan.");
      eksekusiTikunganKanan(); 
      break; 
    }

    if (cekJarakKiri >= 1 && cekJarakKiri <= 30) { 
      Serial.println("STATUS: Tembok Ditemukan (1-30cm)! Kembali ke PID.");
      break; 
    }
    
    delay(10);
  }

  motorBerhenti();
  delay(100);

  lastErrorWall = 0; 
  integralWall = 0;
}

// ================= FUNGSI UTILITAS =================

int bacaJarak(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); 
  int distance = duration * 0.034 / 2;
  return (distance == 0) ? 999 : distance; 
}

void kendaliMotor(int speedKiri, int speedKanan) {
  if (speedKiri >= 0) {
    digitalWrite(pinDir1Kiri, LOW); digitalWrite(pinDir2Kiri, HIGH);
  } else {
    digitalWrite(pinDir1Kiri, HIGH); digitalWrite(pinDir2Kiri, LOW);
    speedKiri = -speedKiri; 
  }
  analogWrite(pinPwmKiri, speedKiri);
  
  if (speedKanan >= 0) {
    digitalWrite(pinDir1Kanan, HIGH); digitalWrite(pinDir2Kanan, LOW);
  } else {
    digitalWrite(pinDir1Kanan, LOW); digitalWrite(pinDir2Kanan, HIGH);
    speedKanan = -speedKanan; 
  }
  analogWrite(pinPwmKanan, speedKanan);
}

void motorBerhenti() {
  kendaliMotor(0, 0);
}

void tampilkanDebug() {
  Serial.print("Sensor: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(analogRead(pinSensor[i]));
    Serial.print(" | ");
  }
  Serial.print("US Depan: "); Serial.print(bacaJarak(trigDepan, echoDepan));
  Serial.print(" | US Kiri: "); Serial.println(bacaJarak(trigKiri, echoKiri));
}

void kembaliKeGaris() {
  Serial.println("STATUS: Garis ditemukan! Memulai manuver pelurusan...");
  
  // 1. Maju sedikit agar as roda pas berada di atas/melewati garis
  kendaliMotor(130, 130);
  delay(30); 

  // 2. Pivot Kanan (-/+) agar robot menghadap ke depan sejajar dengan jalur
  kendaliMotor(-150, 150); 
  
  unsigned long waktuMulaiAlign = millis();
  // Loop memutar sampai salah satu sensor TENGAH (s1 atau s2) mendeteksi garis
  while (analogRead(pinSensor[1]) < threshold[1] && analogRead(pinSensor[2]) < threshold[2]) {
    if (millis() - waktuMulaiAlign > 1500) break; // Timeout 1.5 detik
    delay(10);
  }
  
  motorBerhenti();
  delay(80); 

  // 3. Reset semua nilai error agar PID mulai dari nol dengan mulus
  error = 0;
  lastError = 0;
  lastErrorWall = 0;
  integralWall = 0;
  
  // 4. Baru pindah ke mode AUTO_LINE
  modeSaatIni = AUTO_LINE;
  Serial.println("STATUS: Robot lurus, kembali ke AUTO_LINE.");
}

void eksekusiAksiMap(char aksi) {
  if (aksi == 'L') {
    kendaliMotor(-120, 120);
    delay(150);
    while (analogRead(pinSensor[1]) < threshold[1] && analogRead(pinSensor[2]) < threshold[2]);
  } 
  else if (aksi == 'R') {
    kendaliMotor(120, -120);
    delay(150);
    while (analogRead(pinSensor[1]) < threshold[1] && analogRead(pinSensor[2]) < threshold[2]);
  } 
  else if (aksi == 'S') {
    kendaliMotor(kecepatanDasar, kecepatanDasar);
    delay(150);
  }
}