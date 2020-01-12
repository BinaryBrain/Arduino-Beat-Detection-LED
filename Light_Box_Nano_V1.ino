#define SAMPLEPERIODUS 300
#define DOUBLE_BEAT_THRESHOLD 200 // ms

// STROBE
#define STROBE_FREQ 9
#define STROBE_TIME 5
#define RANDOM_STROBE true
#define RANDOM_STROBE_CHANCES 20000 // 1/200

// FLASH MODE
// #define DECREASE_RATIO 0.02
#define DECREASE_RATIO 0.2

// CHANGE_COLOR MODE
#define MINIMUM_VALUE 5
#define DECREASE_VALUE 90

#define WHITE 6
#define RED 10
#define GREEN 11
#define BLUE 9

#define COLOR_BTN 12
#define BEAT_COLOR_BTN 8
#define MODE_BTN 7
#define STROBE_BTN 4

#define POT_THRESHOLD A5
#define POT_R A2
#define POT_G A3
#define POT_B A4
#define POT_W A1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

enum mode_enum {
  STATIC,
  FLASH,
  CHANGE_COLOR
};

mode_enum mode = FLASH;

bool modeBtnPushed = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Welcome!");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  pinMode(COLOR_BTN, INPUT);
  pinMode(BEAT_COLOR_BTN, INPUT);
  pinMode(MODE_BTN, INPUT);
  pinMode(STROBE_BTN, INPUT);

  // Set ADC to 77khz, max for 10bit
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
}

// 20 - 200hz Single Pole Bandpass IIR Filter
float bassFilter(float sample) {
  static float xv[3] = {0, 0, 0}, yv[3] = {0, 0, 0};
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = sample / 9.1f;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] = (xv[2] - xv[0])
          + (-0.7960060012f * yv[0]) + (1.7903124146f * yv[1]);
  return yv[2];
}

// 10hz Single Pole Lowpass IIR Filter
float envelopeFilter(float sample) { //10hz low pass
  static float xv[2] = {0, 0}, yv[2] = {0, 0};
  xv[0] = xv[1];
  xv[1] = sample / 160.f;
  yv[0] = yv[1];
  yv[1] = (xv[0] + xv[1]) + (0.9875119299f * yv[0]);
  return yv[1];
}

// 1.7 - 3.0hz Single Pole Bandpass IIR Filter
float beatFilter(float sample) {
  static float xv[3] = {0, 0, 0}, yv[3] = {0, 0, 0};
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = sample / 7.015f;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] = (xv[2] - xv[0])
          + (-0.7169861741f * yv[0]) + (1.4453653501f * yv[1]);
  return yv[2];
}

int powerW = 0;
int powerR = 150;
int powerG = 0;
int powerB = 0;

int powerBeatW = 0;
int powerBeatR = 100;
int powerBeatG = 255;
int powerBeatB = 255;

int currentBeatH = 180;

int h = 0;
int s = 255;
int v = 255;

double ratio = 0;

void strobe(double freq, double sec) {
  unsigned long initTime = millis();
  while (millis() < initTime + sec * 1000) {
    analogWrite(RED, 255);
    analogWrite(GREEN, 255);
    analogWrite(BLUE, 255);
    analogWrite(WHITE, 255);

    delay(500 / freq);

    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
    analogWrite(WHITE, 0);

    delay(500 / freq);
  }
}

void strobeColor(double freq, int sec) {
  unsigned long initTime = millis();
  while (millis() < initTime + sec * 1000) {
    analogWrite(RED, 255);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
    analogWrite(WHITE, 0);

    delay(333 / freq);

    analogWrite(RED, 0);
    analogWrite(GREEN, 255);
    analogWrite(BLUE, 0);
    analogWrite(WHITE, 0);


    delay(333 / freq);

    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 255);
    analogWrite(WHITE, 0);

    delay(333 / freq);
  }
}

void loop() {
  unsigned long time = micros(); // Used to track rate
  unsigned long lastBeatTime = millis();
  float sample, value, envelope, beat, thresh;
  thresh = 0.2f;
  unsigned char i;

  int power = 0;
  bool boom = false;

  for (i = 0; true; i++) {
    if (digitalRead(STROBE_BTN) == HIGH) {
      strobe(STROBE_FREQ, STROBE_TIME);
      // strobeColor(5, 6);
    }

    if (digitalRead(MODE_BTN) == HIGH) {
      if (!modeBtnPushed) {
        modeBtnPushed = true;
        Serial.println("pushed");

        switch (mode) {
          case STATIC:
            mode = FLASH;
            break;
          case FLASH:
            mode = CHANGE_COLOR;
            break;
          case CHANGE_COLOR:
            mode = STATIC;
            break;
        }
      }
    } else {
      modeBtnPushed = false;
    }

    if (digitalRead(COLOR_BTN) == HIGH || mode == STATIC) {
      powerW = analogRead(POT_W) / 4;
      powerR = analogRead(POT_R) / 4;
      powerG = analogRead(POT_G) / 4;
      powerB = analogRead(POT_B) / 4;

      // debug
      powerW = 50;
      powerR = 255;
      powerG = 40;
      powerB = 100;

      analogWrite(WHITE, powerW);
      analogWrite(RED, powerR);
      analogWrite(GREEN, powerG);
      analogWrite(BLUE, powerB);
      continue;
    } else if (digitalRead(BEAT_COLOR_BTN) == HIGH) {
      powerBeatW = analogRead(POT_W) / 4;
      powerBeatR = analogRead(POT_R) / 4;
      powerBeatG = analogRead(POT_G) / 4;
      powerBeatB = analogRead(POT_B) / 4;

      analogWrite(WHITE, powerBeatW);
      analogWrite(RED, powerBeatR);
      analogWrite(GREEN, powerBeatG);
      analogWrite(BLUE, powerBeatB);
      continue;
    }

    // Read ADC and center so +-512
    sample = (float)analogRead(0) - 503.f;

    // Filter only bass component
    value = bassFilter(sample);

    // Take signal amplitude and filter
    if (value < 0) value = -value;
    envelope = envelopeFilter(value);

    // Every 200 samples (25hz) filter the envelope
    if (i == 200) {
      // Filter out repeating bass sounds 100 - 180bpm
      beat = beatFilter(envelope);

      // Threshold it based on potentiometer on AN1
      //thresh = 0.02f * (float) analogRead(POT_THRESHOLD);
      thresh = 0.1f;

      // If we are above threshold, light up LED
      if (beat > thresh) {
        digitalWrite(LED_BUILTIN, HIGH);
        if (millis() > lastBeatTime + DOUBLE_BEAT_THRESHOLD) {
          ratio = 1;
        }

        lastBeatTime = millis();

        if (mode == CHANGE_COLOR) {
          if (millis() > lastBeatTime + DOUBLE_BEAT_THRESHOLD) {
            if (RANDOM_STROBE && random(RANDOM_STROBE_CHANCES) == 0) {
              strobeColor(STROBE_FREQ, STROBE_TIME);
            }
            h = random(120, 240);
            s = random(150, 255);

            s = 255;
            v = 255;
          }
        }
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }

      ratio -= DECREASE_RATIO;

      if (ratio <= 0) {
        ratio = 0;
      }

      if (mode == FLASH) {
        if (millis() > lastBeatTime + DOUBLE_BEAT_THRESHOLD) {
          currentBeatH = random(359);
        }

        hsv(currentBeatH, 255, 255 * ratio);
        // analogWrite(WHITE, powerBeatW * ratio + powerW * (1 - ratio));
        // analogWrite(RED, powerBeatR * ratio + powerR * (1 - ratio));
        // analogWrite(GREEN, powerBeatG * ratio + powerG * (1 - ratio));
        // analogWrite(BLUE, powerBeatB * ratio + powerB * (1 - ratio));

        // analogWrite(WHITE, 10);
        // analogWrite(BLUE, 255 * ratio + 50 * (1 - ratio));
        // analogWrite(GREEN, 50 * ratio + 100 * (1 - ratio));
        if (RANDOM_STROBE && random(RANDOM_STROBE_CHANCES) == 0) {
          strobeColor(STROBE_FREQ, STROBE_TIME);
        }
      } else if (mode == CHANGE_COLOR) {
        v -= DECREASE_VALUE;
        if (v < MINIMUM_VALUE) {
          v = MINIMUM_VALUE;
        }

        hsv(h, s, v);
      }

      //Reset sample counter
      i = 0;
    }

    // Consume excess clock cycles, to keep at 5000 hz
    for (unsigned long up = time + SAMPLEPERIODUS; time > 20 && time < up; time = micros());
  }
}

void hsv(int hue, int sat, int val) {
  // h: [0, 359]
  // s: [0, 255]
  // v: [0, 255]
  /* convert hue, saturation and brightness ( HSB/HSV ) to RGB
     The dim_curve is used only on brightness/value and on saturation (inverted).
     This looks the most natural.
  */
  int r;
  int g;
  int b;
  int base;

  if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
    r = val;
    g = val;
    b = val;
  } else  {

    base = ((255 - sat) * val) >> 8;

    switch (hue / 60) {
      case 0:
        r = val;
        g = (((val - base) * hue) / 60) + base;
        b = base;
        break;

      case 1:
        r = (((val - base) * (60 - (hue % 60))) / 60) + base;
        g = val;
        b = base;
        break;

      case 2:
        r = base;
        g = val;
        b = (((val - base) * (hue % 60)) / 60) + base;
        break;

      case 3:
        r = base;
        g = (((val - base) * (60 - (hue % 60))) / 60) + base;
        b = val;
        break;

      case 4:
        r = (((val - base) * (hue % 60)) / 60) + base;
        g = base;
        b = val;
        break;

      case 5:
        r = val;
        g = base;
        b = (((val - base) * (60 - (hue % 60))) / 60) + base;
        break;
    }
  }

  // Base color
  if (r < 40) {
    // r = 40;
  }
  if (g < 50) {
    // g = 50;
  }
  if (b < 20) {
    // b = 20;
  }

  analogWrite(RED,  r);
  analogWrite(GREEN, g);
  analogWrite(BLUE, b);
  analogWrite(WHITE, random(4));
}
