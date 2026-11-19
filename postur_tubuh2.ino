// ────────────────────────────────────────────────────────────────
//  Smart Wearable Posture Corrector & Therapy Controller – ESP32
//  Terintegrasi Blynk Cloud (Non-Blocking & Anti-Lag)
// ────────────────────────────────────────────────────────────────

// ── 0. Konfigurasi Blynk Cloud (Wajib di bagian paling atas) ──
#define BLYNK_TEMPLATE_ID   "TMPL6fUo625Eu"
#define BLYNK_TEMPLATE_NAME "Posture-Mate"
#define BLYNK_AUTH_TOKEN    "GvagMYwKEAsuANsf-7LF093syU-Ilpv6"

#define BLYNK_PRINT Serial
#define BLYNK_TIMEOUT_MS 1000    // Batasi timeout jaringan Blynk maksimal 1 detik (anti-freeze)
#define BLYNK_HEARTBEAT  10      // Heartbeat 10 detik agar putus koneksi cepat terdeteksi

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "soc/soc.h"             // Library kontrol hardware ESP32
#include "soc/rtc_cntl_reg.h"    // Register pengatur daya & brownout

// ── Kredensial WiFi ──
char ssid[] = "wifi-iot";        // Ganti dengan Nama WiFi (SSID) kamu
char pass[] = "password-iot";    // Ganti dengan Password WiFi kamu

// ── Konfigurasi Pin Hardware ──
#define FLEX_PIN        34          // Pin ADC Analog (Sensor Flex - Input Only)
#define VIBRATOR_PIN    25          // Pin Modul Motor Getar
#define BUZZER_PIN      26          // Pin Buzzer Aktif
#define RELAY_ON_PIN    22          // IN1 Relay -> Tombol ON Terapi
#define RELAY_OFF_PIN   23          // IN2 Relay -> Tombol OFF Terapi

// Konfigurasi Logika Modul Relay (Active-LOW)
#define RELAY_ACTIVE    LOW         
#define RELAY_INACTIVE  HIGH        

// ── 1. Parameter Sensitivitas Punggung (Khusus Resistor 47kΩ & Elastic Strap) ──
const float DEADZONE_DIFF      = 1.5;  // Toleransi noise diam saat tegak (Diff <= 1.5 -> 0°)
const float MAX_DIFF_FOR_25DEG = 8.5;  // Penurunan ADC untuk deviasi maksimal (+25°); bungkuk ~3.2 poin sudah mulai timer 3 detik

// ── 2. Parameter Standar Penilaian Postur & Alarm ──
const int IDEAL_CURVATURE      = 25;   // Kurva tulang belakang ideal saat tegak = 25°
const int DEV_LOW              = 4;    // Deviasi >= 4°  -> Mulai condong
const int DEV_MEDIUM           = 6;    // Deviasi >= 6°  -> Postur "Sedang" (Mulai Timer Alarm)
const int DEV_HIGH             = 12;   // Deviasi >= 12° -> Postur "Ekstrem"
const int DEV_ALARM_OFF        = 4;    // Histeresis: Alarm mati & timer reset saat deviasi <= 4°
const unsigned long ALARM_DELAY_MS = 3000; // Buzzer & vibrator aktif jika bungkuk > 3 detik

// ── Variabel Sistem & Filter Anti-Spike (Dual-Baseline Otomatis) ──
float flatValue          = 0;          // Baseline tegak saat Alarm MATI
float flatValueWithAlarm = 0;          // Baseline tegak saat Buzzer & Getar MENYALA (terukur otomatis di setup)
float measuredAlarmDrop  = 0;          // Drop tegangan ADC akibat beban Buzzer & Getar (terukur otomatis)
float filteredADC        = 0;          // Nilai ADC setelah filter EMA
const float ALPHA        = 0.25;       // Filter EMA lebih stabil agar deviasi tidak mudah turun-naik mendadak
float adcHistory[5] = {0, 0, 0, 0, 0}; // Buffer Median 5 siklus untuk membuang spike drop WiFi
int adcHistoryIdx   = 0;

// ── State Machine Relay Non-Blocking (Agar Tidak Memblokir Sensor) ──
int activeRelayPin            = -1;
int relayClicksRemaining      = 0;
bool relayCurrentlyPressed    = false;
unsigned long relayNextAction = 0;

// ── Variabel Penilaian Postur & Durasi ──
unsigned long sitStartTime     = 0;
unsigned long sitDuration      = 0;
unsigned long slouchStartTime  = 0;
unsigned long slouchDuration   = 0;
unsigned long slouchAlarmStart = 0;
unsigned long alarmOnTimestamp = 0;
int postureScore               = 100;
bool isSlouchingAlarm          = false;

int currentDeviation          = 0;
int currentCurvature          = IDEAL_CURVATURE;
String postureStatus          = "Aman";

// Timer untuk Blynk
BlynkTimer timer;

// ── Fungsi Simulasi Klik Tombol via Relay (Non-Blocking, Tanpa delay()) ──
void clickRelay(int pin, int count) {
  // Pastikan relay sebelumnya dilepas jika ada perintah baru
  if (activeRelayPin != -1) {
    digitalWrite(activeRelayPin, RELAY_INACTIVE);
  }
  activeRelayPin        = pin;
  relayClicksRemaining  = count;
  relayCurrentlyPressed = true;
  digitalWrite(activeRelayPin, RELAY_ACTIVE); // Mulai klik ke-1 (Tekan tombol fisik)
  relayNextAction       = millis() + 300;     // Tahan selama 300 ms
}

void updateRelay() {
  if (activeRelayPin == -1 || relayClicksRemaining <= 0) return;
  if ((long)(millis() - relayNextAction) < 0) return;

  if (relayCurrentlyPressed) {
    // Lepas tombol fisik
    digitalWrite(activeRelayPin, RELAY_INACTIVE);
    relayCurrentlyPressed = false;
    relayClicksRemaining--;
    if (relayClicksRemaining > 0) {
      relayNextAction = millis() + 350; // Jeda 350 ms sebelum klik berikutnya
    } else {
      activeRelayPin = -1; // Selesai semua klik
    }
  } else {
    // Mulai klik berikutnya
    digitalWrite(activeRelayPin, RELAY_ACTIVE);
    relayCurrentlyPressed = true;
    relayNextAction = millis() + 300; // Tahan 300 ms
  }
}

// ── Fungsi Baca ADC Anti-Spike (Buang Lonjakan Drop WiFi) ──
float readFlexCleanADC() {
  const int N = 21;
  int samples[N];
  (void)analogRead(FLEX_PIN); // Buang 1 pembacaan awal untuk menstabilkan kapasitor ADC
  for (int i = 0; i < N; i++) {
    samples[i] = analogRead(FLEX_PIN);
    delayMicroseconds(500);
  }
  // Urutkan sampel (insertion sort) untuk memisahkan spike drop di bagian bawah
  for (int i = 1; i < N; i++) {
    int key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  // Ambil rata-rata bagian tengah-atas (indeks 10..16) agar spike drop WiFi terbuang
  long sum = 0;
  for (int i = 10; i <= 16; i++) {
    sum += samples[i];
  }
  return (float)sum / 7.0;
}

void resetAdcHistory(float val) {
  for (int i = 0; i < 5; i++) {
    adcHistory[i] = val;
  }
  adcHistoryIdx = 0;
}

void shiftAdcHistory(float delta) {
  for (int i = 0; i < 5; i++) {
    adcHistory[i] += delta;
  }
}

float getMedian5() {
  float tmp[5];
  for (int i = 0; i < 5; i++) tmp[i] = adcHistory[i];
  for (int i = 1; i < 5; i++) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[2]; // Nilai tengah dari 5 siklus terakhir
}

// ── Handler Blynk: Switch Tombol Terapi ON/OFF (Virtual Pin V4) ── 
BLYNK_WRITE(V4) {
  int switchState = param.asInt();
  
  if (switchState == 1) {
    Serial.println("[TERAPI] Blynk Switch ON -> Relay IN1 (Pin 22) 2x klik (Nyalakan Terapi)...");
    clickRelay(RELAY_ON_PIN, 2);
  } 
  else {
    Serial.println("[TERAPI] Blynk Switch OFF -> Relay IN2 (Pin 23) 2x klik (Matikan Terapi)...");
    clickRelay(RELAY_OFF_PIN, 2);
  }
}

// ── Fungsi Pembacaan Sensor & Algoritma Postur (Dipanggil tiap 100ms via Timer) ──
void readPostureData() {
  // A. Akuisisi Anti-Spike + Median 5 Siklus + Filter EMA
  float cleanADC = readFlexCleanADC();
  adcHistory[adcHistoryIdx] = cleanADC;
  adcHistoryIdx = (adcHistoryIdx + 1) % 5;
  float medianADC = getMedian5();
  filteredADC = (ALPHA * medianADC) + ((1.0 - ALPHA) * filteredADC);

  // B. Selisih ADC Satu Arah dengan Dual-Baseline Otomatis (Alarm OFF vs Alarm ON)
  // Saat Buzzer & Getar menyala, otomatis gunakan flatValueWithAlarm agar drop tegangan 15-25 poin tidak mengunci alarm
  float activeBase = isSlouchingAlarm ? flatValueWithAlarm : flatValue;
  float rawDiff = activeBase - filteredADC;
  if (rawDiff < 0) rawDiff = 0;

  // C. Hitung Deviasi Sudut & Kurva
  currentDeviation = 0;
  if (rawDiff > DEADZONE_DIFF) {
    float devFloat = ((rawDiff - DEADZONE_DIFF) / (MAX_DIFF_FOR_25DEG - DEADZONE_DIFF)) * 25.0;
    currentDeviation = constrain((int)round(devFloat), 0, 25);
  }
  currentCurvature = IDEAL_CURVATURE + currentDeviation;





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  currentDeviation = constrain(map(rawADC,1000,980,0,30),0,30);

//  currentDeviation = rawADC;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // D. Hitung Durasi Duduk
  sitDuration = (millis() - sitStartTime) / 1000;

  // E. Penilaian Status & Posture Score (Aman, Sedang, Ekstrem)
  int anglePenalty = 0;
  int durationPenalty = 0;

  if (currentDeviation >= DEV_HIGH) {
    postureStatus = "Ekstrem";
    if (slouchStartTime == 0) slouchStartTime = millis();
    slouchDuration = (millis() - slouchStartTime) / 1000;

    anglePenalty = map(currentDeviation, DEV_HIGH, 25, 45, 65);
    durationPenalty = slouchDuration * 2;
    postureScore = constrain(100 - (anglePenalty + durationPenalty), 0, 100);
  }
  else if (currentDeviation >= DEV_MEDIUM) {
    postureStatus = "Sedang";
    if (slouchStartTime == 0) slouchStartTime = millis();
    slouchDuration = (millis() - slouchStartTime) / 1000;

    anglePenalty = map(currentDeviation, DEV_MEDIUM, DEV_HIGH - 1, 25, 44);
    durationPenalty = slouchDuration * 1;
    postureScore = constrain(100 - (anglePenalty + durationPenalty), 0, 100);
  }
  else if (currentDeviation >= DEV_LOW) {
    postureStatus = "Aman"; // Masih batas toleransi ringan
    if (slouchStartTime == 0) slouchStartTime = millis();
    slouchDuration = (millis() - slouchStartTime) / 1000;

    anglePenalty = map(currentDeviation, DEV_LOW, DEV_MEDIUM - 1, 10, 24);
    durationPenalty = slouchDuration / 2;
    postureScore = constrain(100 - (anglePenalty + durationPenalty), 0, 100);
  }
  else {
    postureStatus = "Aman";
    slouchStartTime = 0;
    slouchDuration = 0;
    postureScore = 100;
  }

  // F. Keputusan Alarm Histeresis & Delay 3 Detik (Buzzer & Vibrator)
  if (isSlouchingAlarm) {
    // Tahan minimal 600ms setelah baru menyala agar tegangan kapasitor stabil sebelum cek mati
    if ((millis() - alarmOnTimestamp > 600) && (currentDeviation <= DEV_ALARM_OFF)) {
      isSlouchingAlarm = false;
      slouchAlarmStart = 0;
      // Geser state filter naik kembali sebesar measuredAlarmDrop saat beban alarm dilepas
      filteredADC += measuredAlarmDrop;
      shiftAdcHistory(measuredAlarmDrop);
    }
  } else {
    if (currentDeviation <= DEV_ALARM_OFF) {
      slouchAlarmStart = 0;
    } else {
      // Mulai hitung timer saat deviasi menyentuh DEV_MEDIUM (>= 6°)
      if (currentDeviation >= DEV_MEDIUM && slouchAlarmStart == 0) {
        slouchAlarmStart = millis();
      }
      // Selama belum kembali tegak (masih > 4°), lanjutkan hitung sampai > 3 detik
      if (slouchAlarmStart != 0 && (millis() - slouchAlarmStart > ALARM_DELAY_MS)) {
        isSlouchingAlarm = true;
        alarmOnTimestamp = millis();
        // Geser state filter turun sebesar measuredAlarmDrop mengikuti drop tegangan beban alarm
        filteredADC -= measuredAlarmDrop;
        shiftAdcHistory(-measuredAlarmDrop);
      }
    }
  }

  digitalWrite(VIBRATOR_PIN, isSlouchingAlarm ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, isSlouchingAlarm ? HIGH : LOW);

  // Debug Serial Monitor
  Serial.print("ADC: ");        Serial.print(filteredADC, 1);
  Serial.print(" (Base: ");     Serial.print(activeBase, 1);
  Serial.print(" | Diff: ");    Serial.print(rawDiff, 1);         Serial.print(")");
  Serial.print(" | Status: ");  Serial.print(postureStatus);
  Serial.print(" | Skor: ");    Serial.print(postureScore);
  Serial.print(" | Kurva: ");   Serial.print(currentCurvature);   Serial.print("°");
  Serial.print(" | Deviasi: "); Serial.print(currentDeviation);   Serial.print("°");
  Serial.print(" | Alarm: ");   Serial.println(isSlouchingAlarm ? "ON" : "OFF");
}

// ── Fungsi Pengiriman Data Berkala ke Blynk IoT (Dipanggil tiap 3000ms) ──
void sendDataToBlynk() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.virtualWrite(V0, postureStatus);    // V0: String ("Aman", "Sedang", "Ekstrem")
    Blynk.virtualWrite(V1, postureScore);     // V1: Integer (0 - 100)
    Blynk.virtualWrite(V2, currentCurvature); // V2: Integer (25 - 50°)
    Blynk.virtualWrite(V3, currentDeviation); // V3: Integer (0 - 25°)
  }
}

// ── Fungsi Reconnect Blynk Terkontrol (Timeout Singkat 500ms, Anti-Freeze) ──
void checkBlynkConnection() {
  if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
    Serial.println("[BLYNK] Koneksi terputus, mencoba sambung ulang singkat...");
    Blynk.connect(500); // Maksimal 500ms saja agar tidak memblokir pembacaan sensor
  }
}

void setup() {
  // Matikan Brownout Detector bawaan ESP32
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  Serial.begin(115200);

  // Inisialisasi Pin Output Relay 2 Channel (Anti-Glitch Active-LOW)
  digitalWrite(RELAY_ON_PIN, RELAY_INACTIVE); 
  digitalWrite(RELAY_OFF_PIN, RELAY_INACTIVE); 
  pinMode(RELAY_ON_PIN, OUTPUT);
  pinMode(RELAY_OFF_PIN, OUTPUT);

  pinMode(VIBRATOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(VIBRATOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(FLEX_PIN, ADC_11db);

  // ── 1. Koneksi WiFi & Blynk Terlebih Dahulu (Agar Tegangan Stabil Saat Kalibrasi) ──
  Serial.print("Menghubungkan ke WiFi ");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // Matikan modem sleep agar tidak ada lonjakan arus mendadak ke ADC
  WiFi.setTxPower(WIFI_POWER_15dBm); // Sedikit turunkan daya pancar agar aman untuk ADC & tetap kuat jaraknya
  WiFi.begin(ssid, pass);
  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startWifi) < 5000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.connect(2000); // Coba konek maksimal 2 detik
  }

  // ── 2. Kalibrasi Posisi Tegak dengan Tanda Bunyi Beep (Saat WiFi Sudah Aktif) ──
  Serial.println("\n=======================================================");
  Serial.println("   PERSIAPAN KALIBRASI (3 DETIK) - SIAPKAN POSISI TEGAK");
  Serial.println("=======================================================");
  unsigned long prepStart = millis();
  while (millis() - prepStart < 3000) {
    if (WiFi.status() == WL_CONNECTED && Blynk.connected()) Blynk.run();
    delay(50);
  }

  // Beep 1x: Tanda mulai merekam posisi tegak
  Serial.println("[KALIBRASI] Beep 1x -> Mulai kalibrasi! Tahan posisi tegak (2 detik)...");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(150); // Tunggu tegangan pulih setelah beep

  // Tahap 2A: Rekam Baseline Tegak saat Alarm MATI (1,6 detik)
  float sum = 0;
  for (int i = 0; i < 40; i++) {
    sum += readFlexCleanADC();
    if (i % 5 == 0 && WiFi.status() == WL_CONNECTED && Blynk.connected()) {
      Blynk.run(); // Jaga koneksi & beban arus WiFi tetap sama seperti saat loop()
    }
    delay(40);
  }
  flatValue = sum / 40.0;

  // Tahap 2B: Ukur OTOMATIS drop tegangan saat Buzzer & Vibrator MENYALA (tetap tahan posisi tegak!)
  Serial.println("[KALIBRASI] Mengukur drop tegangan Buzzer & Getar (tahan tegak 0.7 detik)...");
  digitalWrite(VIBRATOR_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(350); // Tunggu kapasitor & suplai stabil di kondisi berbeban

  float sumLoaded = 0;
  for (int i = 0; i < 15; i++) {
    sumLoaded += readFlexCleanADC();
    delay(20);
  }
  digitalWrite(VIBRATOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  delay(350); // Tunggu tegangan pulih kembali setelah beban dimatikan

  float loadedADC = sumLoaded / 15.0;
  measuredAlarmDrop = flatValue - loadedADC;
  if (measuredAlarmDrop < 0.0) measuredAlarmDrop = 0.0;
  if (measuredAlarmDrop > 60.0) measuredAlarmDrop = 60.0; // Batas aman
  flatValueWithAlarm = flatValue - measuredAlarmDrop;

  filteredADC = flatValue;
  resetAdcHistory(flatValue);
  sitStartTime = millis();

  // Beep 2x: Tanda kalibrasi selesai
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(120);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < 1) delay(120);
  }
  delay(200); // Tunggu tegangan pulih setelah beep 2x

  Serial.print("[KALIBRASI] Selesai (Beep 2x)! Base OFF: ");
  Serial.print(flatValue, 1);
  Serial.print(" | Base ON: ");
  Serial.print(flatValueWithAlarm, 1);
  Serial.print(" | Drop Beban: ");
  Serial.println(measuredAlarmDrop, 1);

  // ── 3. Interval Timer ──
  timer.setInterval(100L, readPostureData);       // Baca sensor tiap 100ms (10x per detik)
  timer.setInterval(3000L, sendDataToBlynk);      // Kirim ke Blynk tiap 3 detik
  timer.setInterval(10000L, checkBlynkConnection); // Cek reconnect Blynk tiap 10 detik (non-blocking)
}

void loop() {
  // Hanya jalankan Blynk.run() saat benar-benar connected agar TIDAK PERNAH memblokir sensor 5-15 detik
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.run();
  }
  updateRelay(); // Proses klik relay terapi secara non-blocking
  timer.run();
}