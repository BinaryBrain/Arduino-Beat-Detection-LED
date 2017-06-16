#define SAMPLEPERIODUS 200
#define DOUBLE_BEAT_THRESHOLD 100 // ms

// FLASH MODE
#define DECREASE_RATION 0.02

// CHANGE_COLOR MODE
#define MINIMUM_VALUE 5
#define DECREASE_VALUE 10

#define WHITE 6
#define RED 9
#define GREEN 10
#define BLUE 11

#define COLOR_BTN 12
#define BEAT_COLOR_BTN 8
#define MODE_BTN 7
#define BTN_4 4 // NOT USED YET

#define POT_THRESHOLD A1
#define POT_R A2
#define POT_G A3
#define POT_B A4
#define POT_W A5

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

mode_enum mode = CHANGE_COLOR;

bool modeBtnPushed = false;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  pinMode(COLOR_BTN, INPUT);
  pinMode(BEAT_COLOR_BTN, INPUT);
  pinMode(MODE_BTN, INPUT);
  pinMode(BTN_4, INPUT);

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
int powerR = 50;
int powerG = 0;
int powerB = 0;

int powerBeatW = 0;
int powerBeatR = 100;
int powerBeatG = 255;
int powerBeatB = 255;

int h = 0;
int s = 255;
int v = 255;

double ratio = 0;

void loop() {
  unsigned long time = micros(); // Used to track rate
  unsigned long lastBeatTime = millis();
  float sample, value, envelope, beat, thresh;
  thresh = 1;
  unsigned char i;

  int power = 0;
  bool boom = false;

  for (i = 0; true; i++) {
    if (digitalRead(MODE_BTN) == HIGH) {
      if (!modeBtnPushed) {
        modeBtnPushed = true;

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
      thresh = 0.02f * (float) analogRead(POT_THRESHOLD);

      // If we are above threshold, light up LED
      if (beat > thresh) {
        digitalWrite(LED_BUILTIN, HIGH);
        ratio = 1;

        if (mode == CHANGE_COLOR) {
          if (millis() > lastBeatTime + DOUBLE_BEAT_THRESHOLD) {
            lastBeatTime = millis();
            h = random(0, 359);
            s = random(200, 255);
            v = 255;
          }
        }
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }

      ratio -= DECREASE_RATION;

      if (ratio <= 0) {
        ratio = 0;
      }

      if (mode == FLASH) {
        analogWrite(WHITE, powerBeatW * ratio + powerW * (1 - ratio));
        analogWrite(RED, powerBeatR * ratio + powerR * (1 - ratio));
        analogWrite(GREEN, powerBeatG * ratio + powerG * (1 - ratio));
        analogWrite(BLUE, powerBeatB * ratio + powerB * (1 - ratio));
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

  analogWrite(RED, r);
  analogWrite(GREEN, g);
  analogWrite(BLUE, b);
}
