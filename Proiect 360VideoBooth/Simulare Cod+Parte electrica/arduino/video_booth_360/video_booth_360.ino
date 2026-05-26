/*
 * 360 Video Booth - Controller motor DC Valeo-Nidec 24V cu BTS7960
 *
 * Hardware:
 *   - Arduino Uno / Nano
 *   - Driver BTS7960 (43A) - puntea H dubla
 *   - Motor Valeo-Nidec DC 24V (cu reductor melcat integrat)
 *   - Sursa 24V / 10A
 *   - Buton START momentan (NO)
 *   - Buton E-STOP - vezi ESTOP_IS_NC mai jos:
 *       ESTOP_IS_NC = 0 -> buton NO (Wokwi sau buton apasare obisnuit)
 *       ESTOP_IS_NC = 1 -> buton fizic ciuperca NC (recomandat in productie, fail-safe)
 *   - Potentiometru 10k pentru reglaj viteza
 *   - LED stare
 *
 * Conexiuni BTS7960:
 *   RPWM  -> D9  (PWM rotire dreapta - sens normal)
 *   LPWM  -> D10 (PWM rotire stanga - sens invers, neutilizat)
 *   R_EN  -> D7  (enable jumatate dreapta)
 *   L_EN  -> D8  (enable jumatate stanga)
 *   VCC   -> 5V
 *   GND   -> GND
 *   B+    -> +24V sursa
 *   B-    -> GND sursa (comun cu Arduino)
 *   M+/M- -> bornele motorului
 *
 * Logica:
 *   - Apasare START -> rampa accelerare 2s -> rotire constanta 1 cicluri -> rampa decelerare 2s -> stop
 *   - E-STOP intrerupe imediat (motor in coasta / frana scurt-circuit)
 *   - Potentiometru regleaza viteza maxima (0..255 PWM)
 */

#define RPWM_PIN      9
#define LPWM_PIN      10
#define R_EN_PIN      7
#define L_EN_PIN      8
#define START_BTN     2
#define ESTOP_BTN     3
#define POT_PIN       A0
#define LED_PIN       13

// 1 = buton fizic ciuperca NC (productie, fail-safe);
// 0 = buton NO (simulare Wokwi sau buton apasare normal)
#define ESTOP_IS_NC   0

// 1 = include stepper simulat pentru vizualizare Wokwi (28BYJ-48 pe D4..D6+D11);
// 0 = Tinkercad (motor DC direct pe D9 via TIP120) sau productie (Valeo via BTS7960)
#define WOKWI_SIM     0

#if WOKWI_SIM
  #define SIM_IN1   4
  #define SIM_IN2   5
  #define SIM_IN3   6
  #define SIM_IN4   11
#endif

const unsigned long RAMP_UP_MS   = 2000;
const unsigned long RAMP_DOWN_MS = 2000;
const unsigned long CONST_MS     = 8000;   // ~8s la 15 RPM => 2 rotatii complete platforma
const uint8_t       PWM_MIN      = 40;     // PWM minim pentru a invinge inertia de pornire
const uint8_t       PWM_MAX_CAP  = 220;    // limita superioara siguranta (87% din 255)

enum State { IDLE, RAMP_UP, CONSTANT, RAMP_DOWN, ESTOP };
State state = IDLE;

unsigned long stateStart = 0;
uint8_t targetPwm = 0;
uint8_t currentMotorPwm = 0;   // PWM aplicat in acest moment motorului (pentru sim)

#if WOKWI_SIM
// Secventa half-step pentru 28BYJ-48 (8 stari)
const uint8_t HALF_STEP_SEQ[8] = {
  0b0001, 0b0011, 0b0010, 0b0110,
  0b0100, 0b1100, 0b1000, 0b1001
};
const uint8_t SIM_PINS[4] = { SIM_IN1, SIM_IN2, SIM_IN3, SIM_IN4 };
int           simStepIdx     = 0;
unsigned long simLastStepUs  = 0;

void simStepperInit() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(SIM_PINS[i], OUTPUT);
    digitalWrite(SIM_PINS[i], LOW);
  }
}

void simStepperUpdate() {
  if (currentMotorPwm < PWM_MIN) return;
  // perioada intre pasi (us): PWM mare -> rotatie rapida (perioada mica)
  unsigned long period = map(currentMotorPwm, PWM_MIN, PWM_MAX_CAP, 5000UL, 1200UL);
  unsigned long now    = micros();
  if (now - simLastStepUs < period) return;
  simLastStepUs = now;
  simStepIdx    = (simStepIdx + 1) & 7;
  uint8_t pattern = HALF_STEP_SEQ[simStepIdx];
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(SIM_PINS[i], (pattern >> i) & 0x01);
  }
}
#endif

void setup() {
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(ESTOP_BTN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

#if WOKWI_SIM
  simStepperInit();
#endif

  Serial.begin(9600);
  Serial.println(F("360 Video Booth - ready"));
}

uint8_t readTargetPwm() {
  int raw = analogRead(POT_PIN);                     // 0..1023
  long pwm = map(raw, 0, 1023, PWM_MIN, PWM_MAX_CAP);
  return (uint8_t) pwm;
}

void setMotorPwm(uint8_t pwm) {
  analogWrite(RPWM_PIN, pwm);
  analogWrite(LPWM_PIN, 0);
  currentMotorPwm = pwm;
}

void stopMotor() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  currentMotorPwm = 0;
}

bool startPressed() {
  static unsigned long lastDebounce = 0;
  if (digitalRead(START_BTN) == LOW && (millis() - lastDebounce) > 200) {
    lastDebounce = millis();
    return true;
  }
  return false;
}

bool estopActive() {
#if ESTOP_IS_NC
  // NC fizic: contact deschis = apasat => HIGH (cu pullup)
  return digitalRead(ESTOP_BTN) == HIGH;
#else
  // NO (Wokwi sim): apasat = scurtcircuit la GND => LOW
  return digitalRead(ESTOP_BTN) == LOW;
#endif
}

void loop() {
#if WOKWI_SIM
  simStepperUpdate();   // pasul motor simulat - apelat cat mai des
#endif

  if (estopActive() && state != ESTOP) {
    state = ESTOP;
    stopMotor();
    digitalWrite(LED_PIN, LOW);
    Serial.println(F("E-STOP!"));
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - stateStart;

  switch (state) {
    case IDLE:
      digitalWrite(LED_PIN, (now / 1000) % 2);   // pulseaza 1Hz
      stopMotor();
      if (startPressed()) {
        targetPwm = readTargetPwm();
        state = RAMP_UP;
        stateStart = now;
        Serial.print(F("START - target PWM: "));
        Serial.println(targetPwm);
      }
      break;

    case RAMP_UP: {
      digitalWrite(LED_PIN, HIGH);
      uint8_t pwm = map(elapsed, 0, RAMP_UP_MS, PWM_MIN, targetPwm);
      if (pwm > targetPwm) pwm = targetPwm;
      setMotorPwm(pwm);
      if (elapsed >= RAMP_UP_MS) {
        state = CONSTANT;
        stateStart = now;
        Serial.println(F("CONSTANT"));
      }
      break;
    }

    case CONSTANT:
      setMotorPwm(targetPwm);
      if (elapsed >= CONST_MS) {
        state = RAMP_DOWN;
        stateStart = now;
        Serial.println(F("RAMP DOWN"));
      }
      break;

    case RAMP_DOWN: {
      long pwm = (long)targetPwm - map(elapsed, 0, RAMP_DOWN_MS, 0, targetPwm - PWM_MIN);
      if (pwm < PWM_MIN) pwm = 0;
      setMotorPwm((uint8_t)pwm);
      if (elapsed >= RAMP_DOWN_MS) {
        stopMotor();
        state = IDLE;
        stateStart = now;
        Serial.println(F("IDLE"));
      }
      break;
    }

    case ESTOP:
      stopMotor();
      digitalWrite(LED_PIN, (now / 200) % 2);    // pulseaza rapid 5Hz
      if (!estopActive()) {
        state = IDLE;
        stateStart = now;
        Serial.println(F("E-STOP cleared"));
      }
      break;
  }
}
