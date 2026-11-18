// ────────────────────────────────────────────────────────────────
//  Smart Wearable Posture Corrector & Therapy Controller – ESP32
//  Terintegrasi Blynk Cloud (Non-Blocking & Anti-Lag)
// ────────────────────────────────────────────────────────────────

// ── 0. Konfigurasi Blynk Cloud (Wajib di bagian paling atas) ──
#define BLYNK_TEMPLATE_ID   "TMPL6fUo625Eu"
#define BLYNK_TEMPLATE_NAME "New Template"
#define BLYNK_AUTH_TOKEN    "GvagMYwKEAsuANsf-7LF093syU-Ilpv6"

#define BLYNK_PRINT Serial

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
const float DEADZONE_DIFF      = 3.5;  // Toleransi napas/elastis saat tegak (Diff <= 3.5 -> 0°)
const float MAX_DIFF_FOR_25DEG = 18.0; // Penurunan ADC untuk deviasi maksimal (+25°)
const float ALARM_LOAD_OFFSET  = 4.0;  // Kompensasi tarikan ADC saat buzzer & vibrator menyala

// ── 2. Parameter Standar Penilaian Postur & Alarm ──
const int IDEAL_CURVATURE      = 25;   // Kurva tulang belakang ideal saat tegak = 25°
const int DEV_LOW              = 4;    // Deviasi >= 4°  -> Mulai condong
const int DEV_MEDIUM           = 6;    // Deviasi >= 6°  -> Postur "Sedang" (Mulai Timer Alarm)
const int DEV_HIGH             = 12;   // Deviasi >= 12° -> Postur "Ekstrem"
const int DEV_ALARM_OFF        = 4;    // Histeresis: Alarm mati & timer reset saat deviasi <= 4°
const unsigned long ALARM_DELAY_MS = 3000; // Buzzer & vibrator aktif jika bungkuk > 3 detik

// ── Variabel Sistem & Filter ──
float flatValue   = 0;                 // Nilai ADC saat kalibrasi tegak
float filteredADC = 0;                 // Nilai ADC setelah filter EMA
const float ALPHA = 0.25;              // Filter EMA lebih stabil agar deviasi tidak mudah turun-naik mendadak

// ── Variabel Penilaian Postur & Durasi ──
unsigned long sitStartTime     = 0;
unsigned long sitDuration      = 0;
unsigned long slouchStartTime  = 0;
unsigned long slouchDuration   = 0;
unsigned long slouchAlarmStart = 0;
int postureScore               = 100;
bool isSlouchingAlarm          = false;

int currentDeviation          = 0;
int currentCurvature          = IDEAL_CURVATURE;
String postureStatus          = "Aman";

// Timer untuk Blynk
BlynkTimer timer;

// ── Fungsi Simulasi Klik Tombol via Relay (Aman untuk Koneksi Blynk) ──
void clickRelay(int pin, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, RELAY_ACTIVE);   // Relay ON (Tekan tombol fisik)
    delay(300);                        // Tahan tekanan tombol 300 ms
    Blynk.run();                       // Jaga koneksi Blynk tetap hidup
    digitalWrite(pin, RELAY_INACTIVE); // Relay OFF (Lepas tombol fisik)
    if (i < count - 1) {
      delay(350);                      // Jeda antara klik ke-1 dan klik ke-2
      Blynk.run();
    }
  }
}

// ── Handler Blynk: Switch Tombol Terapi ON/OFF (Virtual Pin V4) ──
BLYNK_WRITE(V4) {
  int switchState = param.asInt();
  
  if (switchState == 1) {
    Serial.println("[TERAPI] Blynk Switch ON -> Relay IN1 (Pin 22) 2x klik (Nyalakan Terapi)...");
    clickRelay(RELAY_ON_PIN, 2);
    Serial.println("[TERAPI] Alat terapi telah aktif pada Level 1.");
  } 
  else {
    Serial.println("[TERAPI] Blynk Switch OFF -> Relay IN2 (Pin 23) 2x klik (Matikan Terapi)...");
    clickRelay(RELAY_OFF_PIN, 2);
    Serial.println("[TERAPI] Alat terapi telah dimatikan.");
  }
}

// ── Fungsi Pembacaan Sensor & Algoritma Postur (Dipanggil tiap 100ms via Timer) ──
void readPostureData() {
  // A. Akuisisi & Filtering Sinyal (EMA)
  (void)analogRead(FLEX_PIN); // Buang 1 pembacaan awal untuk menstabilkan kapasitor ADC
  long total = 0;
  for (int i = 0; i < 20; i++) {
    total += analogRead(FLEX_PIN);
    delayMicroseconds(500); // Lebih halus & tidak memblokir prosesor seperti delay()
  }
  float rawADC = (float)total / 20.0;
  filteredADC = (ALPHA * rawADC) + ((1.0 - ALPHA) * filteredADC);

  // B. Selisih ADC Dua Arah (Mutlak: tetap terbaca baik ADC turun maupun naik dari titik tegak)
  float rawDiff = fabs(flatValue - filteredADC);
  if (isSlouchingAlarm) {
    rawDiff -= ALARM_LOAD_OFFSET; // Kompensasi drop tegangan saat buzzer & vibrator ON
  }
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
  if (currentDeviation <= DEV_ALARM_OFF) {
    // Kembali tegak (<= 4°): reset timer dan matikan alarm
    slouchAlarmStart = 0;
    isSlouchingAlarm = false;
  } else {
    // Mulai hitung timer saat deviasi menyentuh DEV_MEDIUM (>= 6°)
    if (currentDeviation >= DEV_MEDIUM && slouchAlarmStart == 0) {
      slouchAlarmStart = millis();
    }
    // Selama belum kembali tegak (masih > 4°), lanjutkan hitung sampai > 3 detik
    if (slouchAlarmStart != 0 && !isSlouchingAlarm && (millis() - slouchAlarmStart > ALARM_DELAY_MS)) {
      isSlouchingAlarm = true;
    }
  }

  digitalWrite(VIBRATOR_PIN, isSlouchingAlarm ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, isSlouchingAlarm ? HIGH : LOW);

  // Debug Serial Monitor
  Serial.print("ADC: ");        Serial.print(filteredADC, 1);
  Serial.print(" (Base: ");     Serial.print(flatValue, 1);
  Serial.print(" | Diff: ");    Serial.print(rawDiff, 1);         Serial.print(")");
  Serial.print(" | Status: ");  Serial.print(postureStatus);
  Serial.print(" | Skor: ");    Serial.print(postureScore);
  Serial.print(" | Kurva: ");   Serial.print(currentCurvature);   Serial.print("°");
  Serial.print(" | Deviasi: "); Serial.print(currentDeviation);   Serial.print("°");
  Serial.print(" | Alarm: ");   Serial.println(isSlouchingAlarm ? "ON" : "OFF");
}

// ── Fungsi Pengiriman Data Berkala ke Blynk IoT (Dipanggil tiap 1000ms) ──
void sendDataToBlynk() {
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, postureStatus);    // V0: String ("Aman", "Sedang", "Ekstrem")
    Blynk.virtualWrite(V1, postureScore);     // V1: Integer (0 - 100)
    Blynk.virtualWrite(V2, currentCurvature); // V2: Integer (25 - 50°)
    Blynk.virtualWrite(V3, currentDeviation); // V3: Integer (0 - 25°)
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
  WiFi.begin(ssid, pass);
  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startWifi) < 5000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.connect(3000); // Coba konek maksimal 3 detik
  }

  // ── 2. Kalibrasi Posisi Tegak dengan Tanda Bunyi Beep (Saat WiFi Sudah Aktif) ──
  Serial.println("\n=======================================================");
  Serial.println("   PERSIAPAN KALIBRASI (3 DETIK) - SIAPKAN POSISI TEGAK");
  Serial.println("=======================================================");
  unsigned long prepStart = millis();
  while (millis() - prepStart < 3000) {
    if (WiFi.status() == WL_CONNECTED) Blynk.run();
    delay(50);
  }

  // Beep 1x: Tanda mulai merekam posisi tegak
  Serial.println("[KALIBRASI] Beep 1x -> Mulai kalibrasi! Tahan posisi tegak (2 detik)...");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(150); // Tunggu tegangan pulih setelah beep

  (void)analogRead(FLEX_PIN); // Stabilkan kapasitor ADC awal
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(FLEX_PIN);
    if (i % 10 == 0 && WiFi.status() == WL_CONNECTED) {
      Blynk.run(); // Jaga koneksi & beban arus WiFi tetap sama seperti saat loop()
    }
    delay(20);
  }
  flatValue = (float)sum / 100.0;
  filteredADC = flatValue;
  sitStartTime = millis();

  // Beep 2x: Tanda kalibrasi selesai
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(120);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < 1) delay(120);
  }

  Serial.print("[KALIBRASI] Selesai (Beep 2x)! Baseline Tegak: ");
  Serial.println(flatValue, 1);

  // ── 3. Interval Timer ──
  timer.setInterval(100L, readPostureData);  // Baca sensor tiap 100ms (10x per detik)
  timer.setInterval(3000L, sendDataToBlynk); // Kirim ke Blynk tiap 1 detik
}

void loop() {
  // Hanya jalankan Blynk & Timer di loop utama agar bebas lag/disconnect
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }
  timer.run();
}