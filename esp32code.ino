// ========= ESP32 HIGH-RATE GEIGER PEAK DETECTOR =========
// Detects valley -> peak even if pulses overlap.
// Prints: low,high
// No delay, ultra-fast loop, works at high CPM
// Baud: 115200

const int PIN = 36;

// ==== Tuning ====
int MIN_DELTA = 700;        // pulse = rise of this magnitude
int BASE_DECAY = 3;         // how fast baseline creeps up
int RESET_DROP = 30;        // how much ADC must fall to re-arm
int NOISE_FLOOR = 80;       // ignore below this
int REFRACTORY_US = 400;    // very short block to avoid double count
// =================

int baseline = 4095;
int peak = 0;
bool armed = true;
unsigned long lastPulseUs = 0;
int threshold = 750;       // Default threshold
const int adcPin = 36;     // VP pin (ADC1_CH0)
const int pulsePin = 15;   // Pin to pull low

bool inPulse = false;
int minVal = 4095;

unsigned long startTime;

void setup() {
Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("READY");
  analogReadResolution(12); // 0–4095
  pinMode(pulsePin, OUTPUT);
  digitalWrite(pulsePin, HIGH); // keep high initially
  startTime = millis();
}

void loop() {
  // --- Read ADC ---
  int v = analogRead(PIN);
  if (v < 0) v = 0;
  if (v < NOISE_FLOOR) v = NOISE_FLOOR;

  unsigned long nowUs = micros();
  unsigned long nowMs = millis();
  bool refractory = (nowUs - lastPulseUs) < REFRACTORY_US;

  // --- Update baseline ---
  if (v < baseline) baseline = v;
  else baseline += BASE_DECAY;

  int rise = v - baseline;


  if (armed && !refractory && rise >= MIN_DELTA) {
    peak = v;

    Serial.print(baseline);
    Serial.print(",");
    Serial.println(peak);

    lastPulseUs = nowUs;
    armed = false;
  }


  if (!armed && (peak - v) >= RESET_DROP) {
    baseline = v;
    peak = 0;
    armed = true;
  }


  unsigned long elapsed = nowMs - startTime;

  if (elapsed >= 10000 && elapsed < 10500) {
    digitalWrite(pulsePin, LOW);
  } else if (elapsed >= 10500) {
    digitalWrite(pulsePin, HIGH);
  }

  int av = analogRead(adcPin);  // second sample

  // Start of pulse
  if (!inPulse && av < threshold) {
    inPulse = true;
    minVal = av;
  }

  // Track minimum value
  if (inPulse) {
    if (av < minVal) minVal = av;

    // End of pulse
    if (av > threshold) {
      inPulse = false;
      Serial.println(minVal);
    }
  }
}
