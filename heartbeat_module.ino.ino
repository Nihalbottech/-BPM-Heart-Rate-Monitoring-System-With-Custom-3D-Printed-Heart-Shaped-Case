#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pulse sensor pin
const int pulsePin = A2;

// Peak detection / timing
unsigned long lastBeatTime = 0;
bool pulseDetected = false;
unsigned long ibi = 600; // inter-beat interval (ms) - initial guess (100 BPM)
int rates[10];           // last 10 IBI values for averaging
byte rateIndex = 0;
bool ratesFilled = false;
int BPM = 0;

// Adaptive thresholding
int signalMax = 512;
int signalMin = 512;
int threshold = 550;

// timing for readings
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 5; // ms between analog reads

// reset adaptation if long gap
unsigned long lastSignalUpdate = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 0);
  display.print("BMP");
  display.setTextSize(3);
  display.display();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
 display.setCursor(0, 30);
  display.print("MONITOR");
  display.display();
  delay(1000);

  // initialize rates
  for (int i = 0; i < 10; i++) rates[i] = 600;
  lastBeatTime = millis();
  lastSignalUpdate = millis();
}

void loop() {
  unsigned long now = millis();

  // sample at roughly sampleInterval ms
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;
    int signal = analogRead(pulsePin); // 0..1023
    // update max/min for adaptive threshold
    if (signal > signalMax) {
      signalMax = signal;
      lastSignalUpdate = now;
    }
    if (signal < signalMin) {
      signalMin = signal;
      lastSignalUpdate = now;
    }

    // recompute threshold from observed min/max
    // but avoid too-frequent changes; do it continuously here for simplicity
    threshold = signalMin + (signalMax - signalMin) / 2;

    // simple peak detection: detect rising edge above threshold with refractory period
    // refractory window to avoid double-counting (>= 250 ms)
    if (signal > threshold && !pulseDetected && (now - lastBeatTime) > 250) {
      pulseDetected = true;
      // compute IBI
      unsigned long thisBeat = now;
      ibi = thisBeat - lastBeatTime;
      lastBeatTime = thisBeat;

      if (ibi > 250 && ibi < 2000) { // plausible range 30..240 BPM
        rates[rateIndex] = ibi;
        rateIndex++;
        if (rateIndex >= 10) {
          rateIndex = 0;
          ratesFilled = true;
        }
        // compute average IBI
        int count = ratesFilled ? 10 : rateIndex;
        long sum = 0;
        for (int i = 0; i < count; i++) sum += rates[i];
        if (count > 0) {
          long avgIbi = sum / count;
          if (avgIbi > 0) BPM = (int)(60000L / avgIbi);
        }
      }
      Serial.print("Beat! IBI(ms): ");
      Serial.print(ibi);
      Serial.print("  BPM: ");
      Serial.println(BPM);

      // after a beat, reset observed peak/min to allow adaptation for next beat
      // (helps with baseline drift)
      signalMax = signal;
      signalMin = signal;
      lastSignalUpdate = now;
    }

    // when signal falls below threshold, allow next detection
    if (signal < threshold) {
      pulseDetected = false;
    }

    // if no significant signal changes for a while, nudge min/max to current signal to adapt
    if (now - lastSignalUpdate > 2000) {
      // decay toward center so threshold adapts slowly
      signalMax = signalMax - 1;
      signalMin = signalMin + 1;
      lastSignalUpdate = now;
    }

    // Display update every loop sample (lightweight)
    drawDisplay(BPM, signal, threshold);
  }

  // nothing else in loop
}

void drawDisplay(int bpmValue, int rawSignal, int th) {
  display.clearDisplay();

  // Title
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Pulse monitor");

  // Big BPM
  display.setTextSize(3);
  display.setCursor(0, 12);
  if (bpmValue > 0) {
    display.print(bpmValue);
  } else {
    display.print("--");
  }
  display.print(" ");

  display.setTextSize(2);
  display.setCursor(90, 28);
  display.print("BPM");

  // small raw-signal bar for quick debug
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("S:");
  display.print(rawSignal);
  display.setCursor(60, 52);
  display.print("T:");
  display.print(th);

  // draw bar representation
  int barWidth = map(constrain(rawSignal, 0, 1023), 0, 1023, 0, 120);
  display.fillRect(4, 40, barWidth, 8, SSD1306_WHITE);

  display.display();
}
